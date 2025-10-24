#ifndef SCAN_MANAGER_HPP
#define SCAN_MANAGER_HPP

#include <sensor_msgs/LaserScan.h>
#include "lidar_loc/map/map_manager.hpp"
#include "lidar_loc/utils.hpp"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Transform.h>
#include <ros/ros.h>
class MapManager;
class ScanManager {
public:
    ScanManager(ros::NodeHandle& nh, std::shared_ptr<MapManager> map_manager = nullptr);
    void calculateLidarPose(
        const sensor_msgs::LaserScan::ConstPtr& laser_scan,
        const tf2::Transform& tf_map_to_laser, const std::string& laser_frame);

    void setLidarPose(std::array<double, 3> pose);
    std::array<double, 3> getLidarPose() const;
    sensor_msgs::PointCloud2 getCloudFiltered() const;

    void setRobotMoving(bool moving);
    void setMatchingScoreThreshold(double threshold);
    void setLastGoodPose(const std::array<double, 3>& pose);

private:
    std::shared_ptr<MapManager> map_manager_;
    float lidar_x_, lidar_y_, lidar_yaw_, deg_to_rad_;

    std::map<std::pair<int, int>, int> dynamic_obstacle_history_;
    std::set<std::pair<int, int>> newly_detected_obstacles_;  
    double accurate_score_;

    bool isNearOccupied(int map_x, int map_y, int radius);
       
    float computeMatchingScore(float test_yaw, const std::vector<cv::Point2f>& filtered_scan_points);
    float hillClimbYaw(float initial_yaw, float delta, int max_steps, const std::vector<cv::Point2f>& filtered_scan_points);

    struct matching_score {
        double getScore() const {
            return (total_points == 0) ? 0.0 : static_cast<double>(matched_points)*100 / total_points;
        }
        double score;
        int total_points;
        int matched_points;
        matching_score(): score(0), total_points(0), matched_points(0) {}
        void print(const char* name) const {
            printf("%s matching Score: %.2f%% (%d/%d)\n", name, getScore(), matched_points, total_points);
        }
    };
    std::array<matching_score, 2> matching_scores_;
    
    std::vector<cv::Point2f> eraseNewlyDetectedObstacles(
        const sensor_msgs::LaserScan::ConstPtr& laser_scan,
        const tf2::Transform& tf_map_to_laser);

    void updateLidarPoseSimple(const std::vector<cv::Point2f>& filtered_scan_points);
    void updateLidarPoseFull(const std::vector<cv::Point2f>& filtered_scan_points);
    
    sensor_msgs::PointCloud2 convertToPointCloud2(
        const std::vector<cv::Point2f>& points,
        const std::string& frame_id,
        const ros::Time& stamp, uint8_t r,
        uint8_t g, uint8_t b);

    bool use_full_search_;
    int radius_near_obstacles_;
    int static_obstacle_threshold_;
    int deg_rotation_;

    std::shared_ptr<sensor_msgs::PointCloud2> cloud_filtered_;
    std::array<double, 3> last_good_pose_ = {0.0, 0.0, 0.0};
};
#endif // SCAN_MANAGER_HPP