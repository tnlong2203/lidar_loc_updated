#include "lidar_loc/scan/scan_manager.hpp"
#define DEG_TO_RAD M_PI / 180.0

ScanManager::ScanManager(ros::NodeHandle& nh, std::shared_ptr<MapManager> map_manager)
    : map_manager_(map_manager), lidar_x_(0.0), lidar_y_(0.0), lidar_yaw_(0.0)
{
    if(!map_manager_){
        throw std::runtime_error("[ScanManager]: MapManager is not initialized.");
    }

    nh.param<bool>("use_full_search", use_full_search_, false);
    nh.param<int>("radius_near_obstacles", radius_near_obstacles_, 2);
    nh.param<int>("static_obstacle_threshold", static_obstacle_threshold_, 5);
    nh.param<int>("deg_rotation", deg_rotation_, 4);

    cloud_filtered_ = std::make_shared<sensor_msgs::PointCloud2>();
    printf("[ScanManager]: Square radius for newly detected obstacle: [%d PIXELS]\n", radius_near_obstacles_);
    printf("[ScanManager]: Threshold to verify newly detected obstacle: [%d TIMES]\n", static_obstacle_threshold_);
    printf("[ScanManager]: Using [%s SEARCH] for lidar pose estimation.\n", use_full_search_ ? "FULL" : "SIMPLE");
    printf("[ScanManager]: Rotation step for FULL SEARCH: [%d DEGREES]\n", deg_rotation_);
    printf("[ScanManager]: Initialized.\n");

    // Initialize last_good_pose_ from parameter server
    double init_x = 0.0, init_y = 0.0, init_yaw = 0.0;
    nh.param("init_pose_x", init_x, 0.0);
    nh.param("init_pose_y", init_y, 0.0);
    nh.param("init_pose_yaw", init_yaw, 0.0);
    last_good_pose_ = {init_x, init_y, init_yaw};
}

// --- Pose locking variables ---
static double matching_score_threshold = 80.0; // can be set externally
static bool robot_is_moving = true;
bool last_good_pose_valid_ = false;

void ScanManager::setRobotMoving(bool moving) {
    robot_is_moving = moving;
}
void ScanManager::setMatchingScoreThreshold(double threshold) {
    matching_score_threshold = threshold;
}

void ScanManager::calculateLidarPose(
    const sensor_msgs::LaserScan::ConstPtr& laser_scan,
    const tf2::Transform& tf_map_to_laser, const std::string& laser_frame)
{
    auto filtered_scan_points = eraseNewlyDetectedObstacles(laser_scan, tf_map_to_laser);
    auto value = convertToPointCloud2(filtered_scan_points, laser_frame, laser_scan->header.stamp, 0, 255, 0);
    cloud_filtered_ = std::make_shared<sensor_msgs::PointCloud2>(value);

    // --- Strict pose locking logic ---
    if (!robot_is_moving && accurate_score_ >= matching_score_threshold && last_good_pose_valid_) {
        // Robot is stationary, score is high, and we have a valid pose: lock to last_good_pose_
        lidar_x_ = last_good_pose_[0];
        lidar_y_ = last_good_pose_[1];
        lidar_yaw_ = last_good_pose_[2];
        // Do NOT run scan-matching or update pose
    } else {
        // Otherwise, run scan-matching
        if (use_full_search_) {
            updateLidarPoseFull(filtered_scan_points);
        } else {
            updateLidarPoseSimple(filtered_scan_points);
        }
        // Save last good pose if score is high and robot is moving
        if (accurate_score_ >= matching_score_threshold && robot_is_moving) {
            last_good_pose_ = {lidar_x_, lidar_y_, lidar_yaw_};
            last_good_pose_valid_ = true;
        }
    }

    matching_scores_[0].print("[ScanManager]: Raw Scan");
    matching_scores_[1].print("[ScanManager]: Filtered Scan");
}

void ScanManager::setLidarPose(std::array<double, 3> pose) {
    lidar_x_ = pose[0];
    lidar_y_ = pose[1];
    lidar_yaw_ = pose[2];
}

std::array<double, 3> ScanManager::getLidarPose() const {
        return {lidar_x_, lidar_y_, lidar_yaw_};
}

sensor_msgs::PointCloud2 ScanManager::getCloudFiltered()const{
    if (cloud_filtered_) {
        return *cloud_filtered_;
    } else {
        throw std::runtime_error("[ScanManager]: Cloud filtered is not set.");
    }
}

std::vector<cv::Point2f> ScanManager::eraseNewlyDetectedObstacles(
    const sensor_msgs::LaserScan::ConstPtr& laser_scan,
    const tf2::Transform& tf_map_to_laser) 
    {

    matching_score raw_scan_score;
    double accurate_score = 0.0;
    std::vector<cv::Point2f> raw_scan_points;

    for (size_t i = 0; i < laser_scan->ranges.size(); ++i) {
        // 5.1 Get valid points
        double range = laser_scan->ranges[i];
        if (range < laser_scan->range_min || range > laser_scan->range_max) {
            continue; // Skip invalid points
        }
        double angle = laser_scan->angle_min + i * laser_scan->angle_increment;
        double px = range * cos(angle);
        double py = range * sin(angle);
        // Convert to map coordinates (meter -> cell)
        double mx = px / map_manager_->getResolution();
        double my = -py / map_manager_->getResolution();

        cv::Point2f scan_pt(mx, my);
        raw_scan_points.push_back(scan_pt);
        std::array<int, 2> map_cell = map_manager_->transformToMapCell(scan_pt, tf_map_to_laser);
        int map_x = map_cell[0];
        int map_y = map_cell[1];
        // Check match with obstacle in map
        if(map_manager_->isValidObstacleMapCroped(map_x, map_y))
            raw_scan_score.matched_points++;
        raw_scan_score.total_points++;

        std::pair<int, int> key(map_x, map_y);
        if (!map_manager_->isNearOccupied(map_x, map_y, radius_near_obstacles_)) {
            dynamic_obstacle_history_[key]++;
            if (dynamic_obstacle_history_[key] >= static_obstacle_threshold_)
                newly_detected_obstacles_.insert(key); // 100% is Dynamic Obstacle
        }
    }
    matching_score filtered_scan_score;
    std::vector<cv::Point2f> filtered_scan_points;
    for (const auto& pt : raw_scan_points) {
        std::array<int, 2> map_cell = map_manager_->transformToMapCell(pt, tf_map_to_laser);
        int map_x = map_cell[0];
        int map_y = map_cell[1];

        std::pair<int, int> key(map_x, map_y);
        // Filter: Just get key not being in newly_detected_obstacles_
        if (newly_detected_obstacles_.find(key) == newly_detected_obstacles_.end()) {
            filtered_scan_points.push_back(pt);
            // Check match with obstacle in map
            if(map_manager_->isValidObstacleMapCroped(map_x, map_y))
                filtered_scan_score.matched_points++;
            filtered_scan_score.total_points++;
        }
    }

    matching_scores_ = {raw_scan_score, filtered_scan_score};
    accurate_score_ = filtered_scan_score.getScore();
    return filtered_scan_points;
}

sensor_msgs::PointCloud2 ScanManager::convertToPointCloud2(
    const std::vector<cv::Point2f>& points,
    const std::string& frame_id,
    const ros::Time& stamp,
    uint8_t r, uint8_t g, uint8_t b)
{
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = stamp;
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(4, 
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud_msg, "rgb");

    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 |
                    static_cast<uint32_t>(b));
    float rgb_float;
    std::memcpy(&rgb_float, &rgb, sizeof(float)); // convert uint32 -> float

    for (const auto& pt : points) {
        *iter_x = pt.x * map_manager_->getResolution();
        *iter_y = -pt.y * map_manager_->getResolution();
        *iter_z = 0.0;
        *iter_rgb = rgb_float;

        ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
    }

    return cloud_msg;
}

float ScanManager::computeMatchingScore(float test_yaw, const std::vector<cv::Point2f>& filtered_scan_points)
{
    int score = 0;
    for (const auto& point : filtered_scan_points) {
        float rotated_x = point.x * cos(test_yaw) - point.y * sin(test_yaw);
        float rotated_y = point.x * sin(test_yaw) + point.y * cos(test_yaw);
        float px = rotated_x + lidar_x_;
        float py = lidar_y_ - rotated_y;
        int map_x = static_cast<int>(px);
        int map_y = static_cast<int>(py);

        score += map_manager_->getMapValueAtMapTemp(map_x, map_y);
    }
    return score;
}

float ScanManager::hillClimbYaw(float initial_yaw, float delta, int max_steps, const std::vector<cv::Point2f>& filtered_scan_points)
{
    float best_yaw = initial_yaw;
    int best_score = computeMatchingScore(best_yaw, filtered_scan_points);
    int step = 0;

    while (step < max_steps) {
        float cw_yaw = best_yaw + delta;
        float ccw_yaw = best_yaw - delta;

        int score_cw = computeMatchingScore(cw_yaw, filtered_scan_points);
        int score_ccw = computeMatchingScore(ccw_yaw, filtered_scan_points);

        if (score_cw > best_score) {
            best_score = score_cw;
            best_yaw = cw_yaw;
        } else if (score_ccw > best_score) {
            best_score = score_ccw;
            best_yaw = ccw_yaw;
        } else {
            break;  
        }
        step++;
    }

    return best_yaw;
}


void ScanManager::updateLidarPoseSimple(const std::vector<cv::Point2f>& filtered_scan_points){
    lidar_yaw_ = hillClimbYaw(lidar_yaw_, DEG_TO_RAD, 10, filtered_scan_points); // max 10 bước ±1°
    // Matching theo 5 offsets vị trí
    std::vector<cv::Point2f> point_sets;
    for (const auto& point : filtered_scan_points) {
        float rotated_x = point.x * cos(lidar_yaw_) - point.y * sin(lidar_yaw_);
        float rotated_y = point.x * sin(lidar_yaw_) + point.y * cos(lidar_yaw_);
        point_sets.emplace_back(rotated_x + lidar_x_, lidar_y_ - rotated_y);
    }

    std::vector<cv::Point2f> offsets = {
        {0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };

    int max_sum = 0;
    float best_dx = 0, best_dy = 0;

    for (const auto& offset : offsets) {
        int sum = 0;
        for (const auto& point : point_sets) {
            float px = point.x + offset.x;
            float py = point.y + offset.y;
            sum += map_manager_->getMapValueAtMapTemp(px, py);
        }

        if (sum > max_sum) {
            max_sum = sum;
            best_dx = offset.x;
            best_dy = offset.y;
        }
    }
    // Cập nhật pose sau tối ưu
    lidar_x_ += best_dx;
    lidar_y_ += best_dy;
    lidar_yaw_ = normalizeAngle(lidar_yaw_);
}

void ScanManager::updateLidarPoseFull(const std::vector<cv::Point2f>& filtered_scan_points){
    std::vector<std::vector<cv::Point2f>> point_sets;
    std::vector<float> yaw_offsets;
    for (int deg = -deg_rotation_; deg <= deg_rotation_; ++deg) {
        float offset_yaw = lidar_yaw_ + deg * DEG_TO_RAD;
        yaw_offsets.push_back(deg * DEG_TO_RAD);

        std::vector<cv::Point2f> rotated_points;
        for (const auto& point : filtered_scan_points) {
            float rotated_x = point.x * cos(offset_yaw) - point.y * sin(offset_yaw);
            float rotated_y = point.x * sin(offset_yaw) + point.y * cos(offset_yaw);
            rotated_points.emplace_back(rotated_x + lidar_x_, lidar_y_ - rotated_y);
        }
        point_sets.push_back(rotated_points);
    }

    int max_sum = 0;
    float best_dx = 0, best_dy = 0, best_dyaw = 0;
    std::vector<cv::Point2f> offsets = {{0,0}, {1,0}, {-1,0}, {0,1}, {0,-1}};
    for (int i = 0; i < offsets.size(); ++i){
        for (int j = 0; j < point_sets.size(); ++j){
            int sum = 0;
            for (const auto& point : point_sets[j]){
                float px = point.x + offsets[i].x;
                float py = point.y + offsets[i].y;
                sum += map_manager_->getMapValueAtMapTemp(px, py);
            }
            if (sum > max_sum){
                max_sum = sum;
                best_dx = offsets[i].x;
                best_dy = offsets[i].y;
                best_dyaw = yaw_offsets[j];
            }
        }
    }
    lidar_x_ += best_dx;
    lidar_y_ += best_dy;
    lidar_yaw_ += best_dyaw;
    lidar_yaw_ = normalizeAngle(lidar_yaw_);
}

void ScanManager::setLastGoodPose(const std::array<double, 3>& pose) {
    last_good_pose_ = pose;
    last_good_pose_valid_ = true;
}