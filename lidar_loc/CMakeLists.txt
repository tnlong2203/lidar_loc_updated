cmake_minimum_required(VERSION 3.0.2)
project(lidar_loc)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  actionlib
  move_base_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs std_srvs actionlib move_base_msgs tf2_ros tf2_geometry_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lidar_loc_node
  src/lidar_fusion_localization.cpp
  src/localization_node.cpp
  src/map/map_manager.cpp
  src/odom/odom_manager.cpp
  src/scan/scan_manager.cpp
)

target_link_libraries(lidar_loc_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS lidar_loc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)