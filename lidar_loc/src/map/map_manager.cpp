#include "lidar_loc/map/map_manager.hpp"

MapManager::MapManager():is_map_ready_(false){
    map_msg_ = std::make_shared<nav_msgs::OccupancyGrid>();
    map_cropped_ = std::make_shared<cv::Mat>();
    map_temp_ = std::make_shared<cv::Mat>();
    map_roi_info_ = std::make_shared<sensor_msgs::RegionOfInterest>();
}

float MapManager::getResolution() const {
    if (is_map_ready_) {
        return map_msg_->info.resolution;
    }
    printf("[MapManager]: Map is not ready, cannot get resolution.\n");
    return 0.0;
}
void MapManager::printOccupiedBoundingBoxCorners(const int xMin, const int xMax, const int yMin, const int yMax) const {
    double x_min = map_msg_->info.origin.position.x + xMin * map_msg_->info.resolution;
    double y_min = map_msg_->info.origin.position.y + yMin * map_msg_->info.resolution;
    double x_max = map_msg_->info.origin.position.x + xMax * map_msg_->info.resolution;
    double y_max = map_msg_->info.origin.position.y + yMax * map_msg_->info.resolution;
    printf("[MapManager]: Occupied bounding box corners:\n");
    printf("[MapManager]: y_min [%.2f]\n", y_min);
    printf("[MapManager]: x_min [%.2f]\n", x_min);
    printf("[MapManager]: y_max [%.2f]\n", y_max);
    printf("[MapManager]: x_max [%.2f]\n", x_max);

}
void MapManager::cropMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (!msg || msg->info.width <= 0 || msg->info.height <= 0) {
        printf("[MapManager]: Received invalid map message!\n");
        printf("[MapManager]: Width: %d <= 0, Height: %d <= 0\n", msg->info.width, msg->info.height);
        return;
    }
    is_map_ready_ = false; // Reset the map ready flag
    map_msg_ = std::make_shared<nav_msgs::OccupancyGrid>(*msg);
    std_msgs::Header header = map_msg_->header;
    nav_msgs::MapMetaData info = map_msg_->info;

    int xMax,xMin,yMax,yMin ;
    xMax=xMin= info.width/2;
    yMax=yMin= info.height/2;
    bool bFirstPoint = true;
    //map_raw is matrix 2D with (height x width) and each element is 8-bit unsigned integer
    cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128)); // 128 is gray; 0 is black; 255 is white
    // Find the occupied cells and calculate the bounding box
    for(int y = 0; y < info.height; y++)
    {
        for(int x = 0; x < info.width; x++)
        {
            int index = y * info.width + x;
            
            // map_msg_.data[index] = 0; // 0 is free space; 100 is occupied space; -1 is unknown
            map_raw.at<uchar>(y, x) = static_cast<uchar>(map_msg_->data[index]);

            // if the cell is occupied
            if(map_msg_->data[index] == 100)
            {
                if(bFirstPoint)
                {
                    xMax = xMin = x;
                    yMax = yMin = y;
                    bFirstPoint = false;
                    continue;
                }
                xMin = std::min(xMin, x);
                xMax = std::max(xMax, x);
                yMin = std::min(yMin, y);
                yMax = std::max(yMax, y);
            }
        }
    }
    printOccupiedBoundingBoxCorners(xMin, xMax, yMin, yMax);
    // center of the map
    int cen_x = (xMin + xMax)/2;
    int cen_y = (yMin + yMax)/2;
    // Cut map based on the bounding box
    int new_half_width = abs(xMax - xMin)/2 + 50;
    int new_half_height = abs(yMax - yMin)/2 + 50;
    int new_origin_x = cen_x - new_half_width;
    int new_origin_y = cen_y - new_half_height;

    int new_width = new_half_width*2;
    int new_height = new_half_height*2;

    // Ensure the new origin and size are within the bounds of the original map
    if(new_origin_x < 0) new_origin_x = 0;
    if((new_origin_x + new_width) > info.width) new_width = info.width - new_origin_x;
    if(new_origin_y < 0) new_origin_y = 0;
    if((new_origin_y + new_height) > info.height) new_height = info.height - new_origin_y;

    cv::Rect roi(new_origin_x, new_origin_y, new_width, new_height);
    cv::Mat roi_map = map_raw(roi).clone();

    map_cropped_ = std::make_shared<cv::Mat>(roi_map);

    // Update the map metadata
    map_roi_info_->x_offset = new_origin_x;
    map_roi_info_->y_offset = new_origin_y;
    map_roi_info_->width = new_width;
    map_roi_info_->height = new_height;
}
/**
 * @brief Create a gradient mask for the map
 * Gradient mask (101x101) is used to create a smooth transition between occupied and free space
 * Every occupied cell in the map will be replaced by a gradient mask (max: 255 and min: 0)
 */
bool MapManager::processMap() {
    if (map_cropped_->empty()) {
        printf("[MapManager]: Map is empty, cannot process.\n");
        return false;
    }

    map_temp_ = std::make_shared<cv::Mat>(cv::Mat::zeros(map_cropped_->size(), CV_8UC1));
    cv::Mat gradient_mask = createGradientMask(101);  // mask size 101x101

    for (int y = 0; y < map_cropped_->rows; y++){
        for (int x = 0; x < map_cropped_->cols; x++){
            if (map_cropped_->at<uchar>(y, x) == 100){  //100 is occupied
                int left = std::max(0, x - 50);
                int top = std::max(0, y - 50);
                int right = std::min(map_cropped_->cols - 1, x + 50);
                int bottom = std::min(map_cropped_->rows - 1, y + 50);

                cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
                cv::Mat region = (*map_temp_)(roi);

                int mask_left = 50 - (x - left);
                int mask_top = 50 - (y - top);
                cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
                cv::Mat mask = gradient_mask(mask_roi);

                cv::max(region, mask, region);
            }
        }
    }
    return true;
}

cv::Mat MapManager::createGradientMask(int size) {
    cv::Mat mask(size, size, CV_8UC1);
    int center = size / 2;
    for (int y = 0; y < size; y++){
        for (int x = 0; x < size; x++){
            double distance = std::hypot(x - center, y - center);
            int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
            mask.at<uchar>(y, x) = value;
        }
    }
    return mask;
    
}

std::array<int, 2> MapManager::transformToMapCell(const cv::Point2f& pt, const tf2::Transform& tf_map_to_laser){
    double px = static_cast<double>(pt.x * map_msg_->info.resolution);
    double py = static_cast<double>(-pt.y * map_msg_->info.resolution);
    tf2::Vector3 point_in_laser(px, py, 0.0);
    tf2::Vector3 point_in_map = tf_map_to_laser * point_in_laser;
    std::array<int, 2> map;
    map[0] = static_cast<int>((point_in_map.x() - map_msg_->info.origin.position.x) / map_msg_->info.resolution) - map_roi_info_->x_offset;
    map[1] = static_cast<int>((point_in_map.y() - map_msg_->info.origin.position.y) / map_msg_->info.resolution) - map_roi_info_->y_offset;
    return map;
}

bool MapManager::isValidObstacleMapCroped(int x, int y) const {
    int px = static_cast<int>(x);
    int py = static_cast<int>(y);
    if (px >= 0 && py >= 0 && px < map_cropped_->cols && py < map_cropped_->rows) {
        return map_cropped_->at<uchar>(py, px) == 100; // OpenCV: (row, col)
    }
    return false; // Out of bounds
}

//Square neighborhood search
bool MapManager::isNearOccupied(int map_x, int map_y, int radius) const{
    for (int dy = -radius; dy <= radius; ++dy){
        for (int dx = -radius; dx <= radius; ++dx){
            int nx = map_x + dx;
            int ny = map_y + dy;
            if(isValidObstacleMapCroped(nx, ny))
                return true;
        }
    }
    return false;
}

uchar MapManager::getMapValueAtMapTemp(int x, int y) const {
    int px = static_cast<int>(x);
    int py = static_cast<int>(y);
    if (px >= 0 && py >= 0 && px < map_temp_->cols && py < map_temp_->rows) {
        return map_temp_->at<uchar>(py, px); // OpenCV: (row, col)
    }
    return 0;
}

std::array<double, 3> MapManager::getInitialPose(const double map_x,const double map_y,const double map_yaw){
    if (!is_map_ready_) {
        printf("[MapManager ]: Map is not ready, cannot get initial pose.\n");
    }
    std::array<double, 3> initial_pose;
    initial_pose[0] = (map_x - map_msg_->info.origin.position.x)/map_msg_->info.resolution - map_roi_info_->x_offset;
    initial_pose[1] = (map_y - map_msg_->info.origin.position.y)/map_msg_->info.resolution - map_roi_info_->y_offset;
    initial_pose[2] = -map_yaw;
    return initial_pose;
}

std::array<double,3> MapManager::convertToMapFrame(const std::array<double,3>& pose){
    std::array<double, 3> pose_map_frame;
    pose_map_frame[0] = (pose[0] + map_roi_info_->x_offset) * map_msg_->info.resolution + map_msg_->info.origin.position.x;
    pose_map_frame[1]= (pose[1] + map_roi_info_->y_offset) * map_msg_->info.resolution + map_msg_->info.origin.position.y;
    pose_map_frame[2]= -pose[2];
    return pose_map_frame;
}

void MapManager::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    cropMap(msg);
    is_map_ready_ = processMap();
    printf("[MapManager ]: Map is %s\n", is_map_ready_ ? "[READY]" : "[NOT READY]");
    printf("[MapManager ]: Map resolution: [%.2f METER/PIXEL]\n", map_msg_->info.resolution);
    printf("[MapManager ]: Initialized.\n");
}

bool MapManager::is_ok() const {
    return is_map_ready_;
}
