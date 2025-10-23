#ifndef LIDAR_LOCALIZATION_HPP
#define LIDAR_LOCALIZATION_HPP

#include <ros/ros.h>
#include "lidar_loc/odom/odom_manager.hpp"
#include "lidar_loc/map/map_manager.hpp"
#include "lidar_loc/scan/scan_manager.hpp"
#include "lidar_loc/utils.hpp"
// ROS message headers
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <vector>
#include <array>
#include <deque>
#include <tuple>
#include <set>
#include <map>
#include <memory>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "lidar_loc/time_executate.hpp"
class LidarFusionLocalization: public ExecutionTimer
{
public:
    LidarFusionLocalization(ros::NodeHandle& nh);
private:
    std::string base_frame_;
    std::string odom_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string laser_topic_;
    int duration_clear_costmap_;
    double init_x_, init_y_, init_yaw_; // Initial pose of the robot
    
    ros::Subscriber map_sub_;
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    ros::Subscriber scan_sub_;
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void clear_costmaps_duration(int duration);

    bool transformMapToLaserFrame(const ros::Time& stamp, tf2::Transform& tf_map_to_laser);
    std::array<double,3> pose_robot_lastest_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    ros::Subscriber initial_pose_sub_;
    int clear_countdown_;
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    ros::Subscriber odom_sub_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::Subscriber nav_goal_sub_;
    void navigationGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void sendNavigationGoal(double x, double y, double yaw);

    tf2::Transform getOdomToMapTransform(geometry_msgs::TransformStamped odom_to_laser, bool is_use_odom);
    void calculatePoseTF(bool is_use_odom);

    ros::ServiceClient clear_costmaps_client_;

    ros::Publisher pub_filtered_points_;

    bool is_publish_last_tf_;               

    bool low_confidence_timer_active = false;
    ros::Time low_confidence_start_time;
    ros::Duration low_confidence_duration = ros::Duration(3.0);; // Allow 3 seconds of low confidence before canceling navigation
    bool navigation_paused = false;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* move_base_client;
    move_base_msgs::MoveBaseGoal last_goal_;

    std::deque<std::tuple<float, float, float>> data_queue_;
    const size_t max_queue_size_;
    bool is_use_odometry_;

    // Smart pointer cho các thành phần có thể tạo/hủy động hoặc chia sẻ
    std::shared_ptr<MapManager> map_manager_;
    std::unique_ptr<ScanManager> scan_manager_;
    std::unique_ptr<OdomManager> odom_manager_;

};
#endif // LIDAR_LOCALIZATION_HPP
