#ifndef LIDAR_LOC_ODOM_LOC_HPP
#define LIDAR_LOC_ODOM_LOC_HPP
#include "ros/ros.h"
#include "lidar_loc/utils.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
class OdomManager
{
public:
    OdomManager();
    void setInitialPose(const std::array<double,3>& lidar_pose);
    tf2::Transform getOdomToMapTransform();
    void setUseOdometry(bool use_odometry);
    std::array<double, 3> getCurrentPose() const;
    std::array<double, 3> getDeltaPose() const;
    std::array<double, 3> getRealPoseRobot() const;
    int isStopped() const;
    void handleOdomUpdate(const nav_msgs::Odometry::ConstPtr& msg);

private:
    void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    std::array<double, 3> odom_pose_;
    std::array<double, 3> odom_pose_pre_;
    std::array<double, 3> delta_pose_;
    std::array<double, 3> lidar_pose_;
    bool first_time_ = true;
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    int is_stopped_;
    int isRobotStopped();
    std::deque<std::array<double, 3>> data_queue_;
    
    bool is_use_odometry_;  
};
#endif // LIDAR_LOC_ODOM_LOC_HPP