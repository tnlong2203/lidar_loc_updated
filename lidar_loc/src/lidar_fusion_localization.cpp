#include "lidar_loc/lidar_fusion_localization.hpp"

bool LidarFusionLocalization::transformMapToLaserFrame(tf2::Transform& tf_map_to_laser){
    // 1. Get the transform from map to base_link

    geometry_msgs::TransformStamped map_to_base;
    try {
        map_to_base = tfBuffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.5));
        pose_robot_lastest_[0] = map_to_base.transform.translation.x;
        pose_robot_lastest_[1]= map_to_base.transform.translation.y;
        tf2::Quaternion q;
        tf2::fromMsg(map_to_base.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        pose_robot_lastest_[2] = yaw;

        // Print for comparison
        printf("[LidarFusionLocalization]: POSE OF ROBOT: [x: %.5f; y: %.5f; yaw: %.5f] | GOOD_ROBOT_POSE: [x: %.5f; y: %.5f; yaw: %.5f]\n",
            pose_robot_lastest_[0], pose_robot_lastest_[1], pose_robot_lastest_[2],
            good_robot_pose_[0], good_robot_pose_[1], good_robot_pose_[2]);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("[LidarFusionLocalization]: Failed to get transform from map to base_frame: %s", ex.what());
        return false; // Return false if transform cannot be found
    }
    // 2. Get the transform from base_frame to laser_frame
    geometry_msgs::TransformStamped base_to_laser;
    try {
        base_to_laser = tfBuffer_.lookupTransform(base_frame_, laser_frame_, ros::Time(0), ros::Duration(0.5));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("[LidarFusionLocalization]: Failed to get transform from base_frame to laser_frame: %s", ex.what());
        return false; // Return false if transform cannot be found
    }
    // 3. Combine transforms to get map to laser_frame
    tf2::Transform tf_map_to_base, tf_base_to_laser;
    tf2::fromMsg(map_to_base.transform, tf_map_to_base);
    tf2::fromMsg(base_to_laser.transform, tf_base_to_laser);
    tf_map_to_laser = tf_map_to_base * tf_base_to_laser;
    return true;
}
void LidarFusionLocalization::clear_costmaps_duration(int duration)
{
    static ros::Time stop_start_time;
    static bool was_stopped = false;

    if (odom_manager_->isStopped()) { // Robot đang dừng
        if (!was_stopped) {
            stop_start_time = ros::Time::now();
            was_stopped = true;
        } else if ((ros::Time::now() - stop_start_time).toSec() >= duration) {
            std_srvs::Empty srv;
            clear_costmaps_client_.call(srv);
            was_stopped = false; 
            printf("[LidarFusionLocalization]: Costmaps cleared after %d seconds of robot stop.\n", duration);
        }
    } else {
        was_stopped = false;
    }
}
/**
 * @brief Processes new LaserScan data to update the lidar pose using scan matching.
 *
 * Workflow:
 * 1. Check if the map is ready.
 * 2. Obtain the transform from map to laser frame at the scan timestamp.
 * 3. Call ScanManager to perform scan matching:
 *    - Filter out dynamic obstacles from the scan data.
 *    - Rotate/translate the scan points around the current pose.
 *    - Compute the matching score on the soft map (map_temp_).
 *    - Optimize the pose (using hill climbing or brute-force).
 *    - Update the lidar pose with the optimal result.
 * 4. Publish the filtered pointcloud for debugging/visualization.
 * 5. If the robot has stopped long enough, automatically clear the costmaps to remove temporary dynamic obstacles.
 * 6. Update and broadcast the map→odom transform based on the new pose.
 *
 * @param msg The latest LaserScan data.
 */
void LidarFusionLocalization::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ExecutionTimer execution_timer("scanCallback");

    if (!map_manager_->is_ok()) {
        printf("[LidarFusionLocalization]: Map is not ready yet, cannot process scan data.\n");
        return;
    }
    tf2::Transform tf_map_to_laser;
    if(!transformMapToLaserFrame(tf_map_to_laser)){
        return; // Failed to get valid transform
    }
    bool robot_stopped = odom_manager_->isStopped();
    if (robot_stopped) {
        scan_manager_->setLidarPose(good_lidar_pose_); // Lock pose to good pose
        // Prevent scan matching update when stopped
        pub_filtered_points_.publish(scan_manager_->getCloudFiltered());
        clear_costmaps_duration(duration_clear_costmap_);
        calculatePoseTF(false);
        printf("____________________________________________\n");
        return;
    }
    scan_manager_->calculateLidarPose(msg, tf_map_to_laser, laser_frame_);
    pub_filtered_points_.publish(scan_manager_->getCloudFiltered());

    float score = scan_manager_->getAccurateScore();
    // Only update good pose if score is above threshold
    if (score > lidar_score_threshold_) {
        good_lidar_pose_ = scan_manager_->getLidarPose();
        good_robot_pose_ = pose_robot_lastest_;
    } else {
        // If score is too low, lock robot pose to last good pose to prevent drift
        scan_manager_->setLidarPose(good_lidar_pose_);
        pose_robot_lastest_ = good_robot_pose_;
    }

    clear_costmaps_duration(duration_clear_costmap_);//when robot is stopped, clear costmaps after 3 seconds

    calculatePoseTF(false);
    printf("____________________________________________\n");
}

void LidarFusionLocalization::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (!map_manager_->is_ok()) {
        printf("[LidarFusionLocalization]: Map is not ready yet, cannot set initial pose.\n");
        return;
    }
    double map_x = msg->pose.pose.position.x;
    double map_y = msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    auto initial_pose = map_manager_->getInitialPose(map_x, map_y, yaw);
    scan_manager_->setLidarPose(initial_pose);
    good_lidar_pose_ = initial_pose; // Set as good pose

    // clear_countdown_ = 30;
    is_publish_last_tf_ = false;
    printf("[LidarFusionLocalization]: initialPoseCallback: [x: %.5f; y: %.5f; yaw: %.5f]\n\n", map_x, map_y, yaw);
}
void LidarFusionLocalization::calculatePoseTF(bool is_use_odom) 
{
    if (!map_manager_->is_ok()){
        printf("[LidarFusionLocalization]: Map is not ready yet, cannot calculate pose TF.\n");
        return;
    }
    geometry_msgs::TransformStamped odom_to_laser;
    try {
        odom_to_laser = tfBuffer_.lookupTransform(odom_frame_, laser_frame_, ros::Time(0),ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("[LidarFusionLocalization]: Failed to get transform from odom_frame_ to laser_frame_: %s", ex.what());
        return;
    }
    tf2::Transform map_to_odom = getOdomToMapTransform(odom_to_laser, is_use_odom);
    geometry_msgs::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = ros::Time::now();
    map_to_odom_msg.header.frame_id = map_frame_;
    map_to_odom_msg.child_frame_id = odom_frame_;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);
    
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(map_to_odom_msg);
}

tf2::Transform LidarFusionLocalization::getOdomToMapTransform(geometry_msgs::TransformStamped odom_to_laser, bool is_use_odom)
{
    odom_manager_->setUseOdometry(is_use_odom);
    static std::array<double, 3> pose_robot_cur = {0,0,0};
    int robot_status = odom_manager_->isStopped();
    printf("[LidarFusionLocalization]: status %s; ", robot_status == 1?"[STOPED]":(robot_status == 0?"[MOVING]":"[UNKNOWN]"));
    printf("use is %s; ", is_use_odom == 1?"[ODOM]":"[LIDAR]");
    printf("update %s tf; \n", is_publish_last_tf_== 1?"[OLD]":"[NEW]");
    if(!is_use_odom){ 
        if(!is_publish_last_tf_){

            pose_robot_cur = map_manager_->convertToMapFrame(scan_manager_->getLidarPose());
        }else{
            // pose_robot_cur = pose_robot_lastest_;
        }
    }
    // UPDATE ROBOT POSE BASE ON ODOMETRY  
    else{
        // pose_robot_cur = odom_loc_->getCurrentPose();  
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, pose_robot_cur[2]);
    tf2::Transform map_to_base, odom_to_base_tf2;
    map_to_base.setOrigin(tf2::Vector3(pose_robot_cur[0], pose_robot_cur[1], 0));
    map_to_base.setRotation(q);

    tf2::fromMsg(odom_to_laser.transform, odom_to_base_tf2);
    return (map_to_base * odom_to_base_tf2.inverse());
}
/**
 * @brief Processes a new OccupancyGrid map message and resets the localization system.
 *
 * Workflow:
 * 1. Update the internal map in MapManager with the new OccupancyGrid data.
 * 2. Initialize the robot's pose to the configured initial position and orientation.
 *    - Create a PoseWithCovarianceStamped message using the initial (x, y, yaw) parameters.
 *    - Call initialPoseCallback to set the initial pose for scan matching and localization.
 * 3. Recalculate and broadcast the map→odom transform based on the new map and initial pose.
 *
 * This ensures that whenever a new map is received (e.g., after a map change or reload),
 * the localization system is synchronized with the latest map and starts from the correct initial pose.
 *
 * @param msg The latest OccupancyGrid map message.
 */
void LidarFusionLocalization::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_manager_->updateMap(msg);
    if(!map_manager_->is_ok()){
        printf("[LidarFusionLocalization]: Map is not ready yet, cannot calculate pose TF.\n");
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, init_yaw_);
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.pose.pose.position.x = init_x_;
    init_pose.pose.pose.position.y = init_y_;
    init_pose.pose.pose.position.z = 0.0;
    init_pose.pose.pose.orientation.x = q.x();
    init_pose.pose.pose.orientation.y = q.y();
    init_pose.pose.pose.orientation.z = q.z();
    init_pose.pose.pose.orientation.w = q.w();
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr init_pose_ptr(
        new geometry_msgs::PoseWithCovarianceStamped(init_pose));
    initialPoseCallback(init_pose_ptr);
    calculatePoseTF(false);
}

void LidarFusionLocalization::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_manager_->handleOdomUpdate(msg);
}

LidarFusionLocalization::LidarFusionLocalization(ros::NodeHandle& nh): 
    ExecutionTimer("LidarFusionLocalization"),tfBuffer_(), tfListener_(tfBuffer_), 
    max_queue_size_(10), pose_robot_lastest_({0.0, 0.0, 0.0}), is_set_initial_pose_(false)
{
    // Load parameters from the parameter server
    nh.param("init_pose_x", init_x_, -3.0);
    nh.param("init_pose_y", init_y_, 1.0);
    nh.param("init_pose_yaw", init_yaw_, 0.0);

    nh.param<std::string>("base_frame", base_frame_, "base_footprint");
    nh.param<int>("duration_clear_costmap", duration_clear_costmap_, 3);
    nh.param<std::string>("map_frame", map_frame_, "map");
    nh.param<std::string>("laser_frame", laser_frame_, "base_scan");
    nh.param<std::string>("odom_frame", odom_frame_, "odom");
    
    std::string map_topic = nh.param<std::string>("map_topic", "map");
    std::string laser_topic = nh.param<std::string>("laser_topic", "scan");
    std::string odom_topic = nh.param<std::string>("odom_topic", "odom");

    std::string initial_pose_topic = nh.param<std::string>("initial_pose_topic", "initialpose");
    std::string filtered_scan_topic = nh.param<std::string>("filtered_scan_topic", "filtered_scan_points");
    std::string move_base_goal_topic = nh.param<std::string>("move_base_goal_topic", "move_base_simple/goal");
    std::string move_base_action_name = nh.param<std::string>("move_base_action_name", "move_base");
    std::string move_base_clear_costmaps_service_name = nh.param<std::string>("move_base_clear_costmaps_service_name", "move_base/clear_costmaps");

    // Initialize subscribers and publishers
    map_sub_ = nh.subscribe(map_topic, 1, &LidarFusionLocalization::mapCallback, this);
    scan_sub_ = nh.subscribe(laser_topic, 1, &LidarFusionLocalization::scanCallback, this);
    odom_sub_ = nh.subscribe(odom_topic, 10, &LidarFusionLocalization::odomCallback, this);
    initial_pose_sub_ = nh.subscribe(initial_pose_topic, 1, &LidarFusionLocalization::initialPoseCallback, this);
    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>(filtered_scan_topic, 1);
    clear_costmaps_client_ = nh.serviceClient<std_srvs::Empty>(move_base_clear_costmaps_service_name);
    move_base_client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(move_base_action_name, true);

    map_manager_ = std::make_shared<MapManager>();
    scan_manager_ = std::make_unique<ScanManager>(nh, map_manager_);
    odom_manager_ = std::make_unique<OdomManager>();
    ros::Rate rate(10);
    while (ros::ok() && !tfBuffer_.canTransform("odom", "base_footprint", ros::Time(0), ros::Duration(0.1))) {
        printf("Waiting for transform from odom to base_footprint...\n");
        rate.sleep();
    }

    // Publish initial map->odom transform at startup
    geometry_msgs::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = ros::Time::now();
    map_to_odom_msg.header.frame_id = map_frame_;
    map_to_odom_msg.child_frame_id = odom_frame_;
    map_to_odom_msg.transform.translation.x = 0.0;
    map_to_odom_msg.transform.translation.y = 0.0;
    map_to_odom_msg.transform.translation.z = 0.0;
    map_to_odom_msg.transform.rotation.x = 0.0;
    map_to_odom_msg.transform.rotation.y = 0.0;
    map_to_odom_msg.transform.rotation.z = 0.0;
    map_to_odom_msg.transform.rotation.w = 1.0;
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(map_to_odom_msg);

    nh.param<double>("lidar_score_threshold", lidar_score_threshold_, 0.7);
    good_lidar_pose_ = {init_x_, init_y_, init_yaw_}; // Set initial pose as good pose
}


