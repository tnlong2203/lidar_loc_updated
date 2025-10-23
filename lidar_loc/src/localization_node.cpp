#include "lidar_loc/lidar_fusion_localization.hpp"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lidar_loc");
    ros::NodeHandle nh;

    auto lidar_localization = std::make_unique<LidarFusionLocalization>(nh);
    ros::spin();

    return 0;
}