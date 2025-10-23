#ifndef   LIDAR_LOC_MAP_MANAGER_HPP
#define   LIDAR_LOC_MAP_MANAGER_HPP
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/RegionOfInterest.h>
#include <tf2/LinearMath/Transform.h>

class MapManager {
public:
    MapManager();
    
    void updateMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    std::array<double, 3> getInitialPose(const double map_x, const double map_y, const double map_yaw);
    std::array<double,3> convertToMapFrame(const std::array<double,3>& pose);
    bool is_ok() const;

    std::array<int, 2> transformToMapCell(const cv::Point2f& pt, const tf2::Transform& tf_map_to_laser);
    bool isValidObstacleMapCroped(int x, int y) const;
    uchar getMapValueAtMapTemp(int x, int y) const;
    bool isNearOccupied(int map_x, int map_y, int radius) const;
    float getResolution() const;
private:
    // void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void cropMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool processMap();
    void printOccupiedBoundingBoxCorners(const int xMin, const int xMax, const int yMin, const int yMax) const;
    cv::Mat createGradientMask(int size);
    
    std::shared_ptr<nav_msgs::OccupancyGrid> map_msg_; //nav_msgs::OccupancyGrid
    std::shared_ptr<cv::Mat> map_cropped_;
    std::shared_ptr<cv::Mat> map_temp_;
    std::shared_ptr<sensor_msgs::RegionOfInterest> map_roi_info_;

    bool is_map_ready_;
};
#endif // LIDAR_LOC_MAP_MANAGER_HPP