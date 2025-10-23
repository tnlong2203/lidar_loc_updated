#include "lidar_loc/odom/odom_manager.hpp"

#define DEBUG
OdomManager::OdomManager():
    lidar_pose_({0.0, 0.0, 0.0}),    odom_pose_({0.0, 0.0, 0.0}),    odom_pose_pre_({0.0, 0.0, 0.0}),
    delta_pose_({0.0, 0.0, 0.0}),    is_stopped_(-1),    is_use_odometry_(false)
{
    printf("[OdomManager]: Initialized.\n");
}


std::array<double, 3> OdomManager::getRealPoseRobot() const { return odom_pose_; }

std::array<double, 3> OdomManager::getDeltaPose() const { return delta_pose_;}
void OdomManager::setUseOdometry(bool use_odometry) { is_use_odometry_ = use_odometry; }


int OdomManager::isStopped() const { return is_stopped_; }
int OdomManager::isRobotStopped() 
{
    const size_t max_size = 10;
    data_queue_.push_back(odom_pose_);
    if(data_queue_.size() < max_size) return 2; // -1: unknow
    if (data_queue_.size() > max_size) data_queue_.pop_front();
    if(data_queue_.size() == max_size) {
        auto& first = data_queue_.front();
        auto& last = data_queue_.back();
        double dx = std::abs(std::get<0>(last) - std::get<0>(first));
        double dy = std::abs(std::get<1>(last) - std::get<1>(first));
        double d_theta = std::abs(std::get<2>(last) - std::get<2>(first));
        if (dx < 0.001 && dy < 0.001 && d_theta < 1 * M_PI / 180) {
            // data_queue_.clear();
            return 1;// 1: stop
        }
    }
    return 0; // 0: moving
}
void OdomManager::setInitialPose(const std::array<double,3>& lidar_pose)
{
    lidar_pose_[0] = lidar_pose[0];
    lidar_pose_[1] = lidar_pose[1];
    lidar_pose_[2] = lidar_pose[2];
    printf("[OdomManager]: Set InitialPose [%f; %f; %f]\n", lidar_pose[0], lidar_pose[1], lidar_pose[2]);
}
void OdomManager::handleOdomUpdate(const nav_msgs::Odometry::ConstPtr& msg) {
   auto odom_data = msg->pose.pose.position;
    odom_pose_[0] = odom_data.x;
    odom_pose_[1] = odom_data.y;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    odom_pose_[2] = yaw;

    if(first_time_) {
        odom_pose_pre_ = odom_pose_;
        first_time_ = false;
        return;
    }
    delta_pose_ = {odom_pose_[0] - odom_pose_pre_[0],
                odom_pose_[1] - odom_pose_pre_[1],
                odom_pose_[2] - odom_pose_pre_[2]};
    odom_pose_pre_ = odom_pose_;
    is_stopped_ = isRobotStopped();
    if(is_use_odometry_) {
        lidar_pose_[0] += delta_pose_[0];
        lidar_pose_[1] += delta_pose_[1];
        lidar_pose_[2] += delta_pose_[2];
        lidar_pose_[2] = normalizeAngle(lidar_pose_[2]);
    }
}