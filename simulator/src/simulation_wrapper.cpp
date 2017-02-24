
#include "simulator/simulation_wrapper.h"

SimulationWrapper::SimulationWrapper() {
    joy_gain_x_ = joy_gain_y_ = joy_gain_theta_ = 0;
    init_map_ = false;
    pub_fake_localizer_ = true;
    pub_real_pose_ = true;
    odom_seq_ = 1;
    simulate_front_scan_ = true;
    simulate_rear_scan_ = true;
    simulate_front_scan_perfect=true;
    simulate_rear_scan_perfect=true;

}

SimulationWrapper::~SimulationWrapper() {
}

bool SimulationWrapper::init() {
    std::string path;

    robot_link_ = "/np";

    joy_gain_x_ = 1;
    joy_gain_theta_ = 2;


    laser_scanner_x_ = 1.2;
    laser_scanner_y_ = 0;
    laser_scanner_theta_ = 0;
    laser_scanner_rear_x_= 0.4;
    laser_scanner_rear_y_= -0.4;
    laser_scanner_rear_theta_ = M_PI;


    loadParams(path);
    if(simulate_front_scan_)
      laser_scan_front_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_front", 1);
    if(simulate_front_scan_perfect)
      laser_scan_front_pub_perfect = nh_.advertise<sensor_msgs::LaserScan>("scan_front_perfect", 1);
    if(simulate_rear_scan_)
      laser_scan_rear_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_rear", 1);
   if(simulate_rear_scan_perfect)
      laser_scan_rear_pub_perfect = nh_.advertise<sensor_msgs::LaserScan>("scan_rear_perfect", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &SimulationWrapper::joyCallback, this);
    vel_keyboard_sub = nh_.subscribe<geometry_msgs::TwistStamped>("key_board", 1, &SimulationWrapper::velKeyboardCallback, this);
    vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SimulationWrapper::velCallback, this);
    place_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("place_robot", 1, &SimulationWrapper::placeRobotCallback, this);
    place_rviz_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &SimulationWrapper::placeRobotRvizCallback, this);
    true_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("true_pose", 1);
    odom_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);


    robot_contour_pub_ = nh_.advertise<visualization_msgs::Marker>("/robot_contour", 1);
    robot_odom_contour_pub_ = nh_.advertise<visualization_msgs::Marker>("/robot_odom_contour", 1);
    // Load static map
    get_map_srv_client_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv_get_map;

    std::cout << "############# Wait for map #############" << std::endl;
    do {
        get_map_srv_client_.call(srv_get_map);
    } while (srv_get_map.response.map.data.size() == 0);
    mapCallback(boost::make_shared<nav_msgs::OccupancyGrid>(srv_get_map.response.map));


    Simulation::init();
    distance_map_ptr_->writeToPPM("received_map.ppm");


    sim_time_ = ros::Time::now();

    las_loc_update_ = ros::Time::now();


    return true;
}

void SimulationWrapper::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {

    std::cout << "############# Got Map #############" << std::endl;
    map_ptr_.reset(new tools::Map2D<float>());
    map_ptr_->setOffsetXY(map->info.origin.position.x, map->info.origin.position.y);
    map_ptr_->setResolution(map->info.resolution);
    map_ptr_->resize(map->info.width, map->info.height);
    map_ptr_->fill(map->data);
    std::cout << "Map size: " << map_ptr_->getMapSizeX() << ", " << map_ptr_->getMapSizeY() << std::endl;
}

void SimulationWrapper::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    robot_v_x_control_ = joy_gain_x_ * joy->axes[1];
    robot_v_y_control_ = joy_gain_y_ * joy->axes[0];
    robot_v_theta_control_ = joy_gain_theta_ * joy->axes[2];
}

void SimulationWrapper::velKeyboardCallback(const geometry_msgs::TwistStamped::ConstPtr& twi) {
    robot_v_x_control_ = joy_gain_x_ * twi->twist.linear.x;
    robot_v_y_control_ = 0;
    robot_v_theta_control_ = joy_gain_theta_ * twi->twist.angular.z;
}

void SimulationWrapper::velCallback(const geometry_msgs::Twist::ConstPtr& twi) {
    robot_v_x_control_ = twi->linear.x;
    robot_v_y_control_ = 0;
    robot_v_theta_control_ = twi->angular.z;

}

void SimulationWrapper::placeRobotCallback(const geometry_msgs::Pose2D::ConstPtr& pos) {
    robot_v_x_control_ = robot_v_y_control_ = robot_v_theta_control_ = 0;
    robot_x_ = pos->x;
    robot_y_ = pos->y;
    robot_theta_ = pos->theta;

    odom_x_ = odom_y_ = odom_theta_ = 0;
}

void SimulationWrapper::placeRobotRvizCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos) {
    robot_v_x_control_ = robot_v_y_control_ = robot_v_theta_control_ = 0;
    robot_x_ = pos->pose.pose.position.x;
    robot_y_ = pos->pose.pose.position.y;
    robot_theta_ = tf::getYaw(pos->pose.pose.orientation);
    odom_x_ = odom_y_ = odom_theta_ = 0;

}

bool SimulationWrapper::spinOnes() {
    //TODO: one simulation loop

    sim_time_ = ros::Time::now();

    updateVelocity(robot_v_x_control_, robot_v_y_control_, robot_v_theta_control_);

    Simulation::spinOnes();

    sendLaserScan();
    sendRobotContour();
    sendTF();
    sendPose();


    return true;
}

void SimulationWrapper::sendTF() {
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom_x_, odom_y_, 0.0));
    q.setRPY(0, 0, odom_theta_);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, sim_time_, "/odom", robot_link_));


    tf::Transform transform_gt;
    transform_gt.setOrigin(tf::Vector3(robot_x_, robot_y_, 0.0));
    q.setRPY(0, 0, robot_theta_);
    transform_gt.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_gt, sim_time_, "/map", "/true_pose"));
   

    tf::Transform transform_front_scan;
    transform_front_scan.setIdentity();
    transform_front_scan.setOrigin(tf::Vector3(laser_scanner_x_, laser_scanner_y_,0.0));
    q.setRPY(0, 0, laser_scanner_theta_);
    transform_front_scan.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_front_scan, sim_time_, robot_link_, "/scan_front_frame"));

    tf::Transform transform_front_scan_perfect;
    transform_front_scan_perfect.setIdentity();
    transform_front_scan_perfect.setOrigin(tf::Vector3(laser_scanner_x_, laser_scanner_y_,0.0));
    q.setRPY(0, 0, laser_scanner_theta_);
    transform_front_scan_perfect.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_front_scan_perfect, sim_time_, robot_link_, "/scan_front_frame_perfect"));


    tf::Transform transform_front_scan_simulation;
    transform_front_scan_simulation.setIdentity();
    transform_front_scan_simulation.setOrigin(tf::Vector3(laser_scanner_x_, laser_scanner_y_,0.0));
    q.setRPY(0, 0, laser_scanner_theta_);
    transform_front_scan_simulation.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_front_scan_simulation, sim_time_, robot_link_, "/scan_front_frame_simulation"));

    tf::Transform transform_rear_scan;
    transform_rear_scan.setIdentity();
    transform_rear_scan.setOrigin(tf::Vector3(laser_scanner_rear_x_, laser_scanner_rear_y_,0.0));
    q.setRPY(0, 0, laser_scanner_rear_theta_);
    transform_rear_scan.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_rear_scan, sim_time_, robot_link_, "/scan_rear_frame"));



    tf::Transform transform_rear_scan_perfect;
    transform_rear_scan_perfect.setIdentity();
    transform_rear_scan_perfect.setOrigin(tf::Vector3(laser_scanner_rear_x_, laser_scanner_rear_y_,0.0));
    q.setRPY(0, 0, laser_scanner_rear_theta_);
    transform_rear_scan_perfect.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_rear_scan_perfect, sim_time_, robot_link_, "/scan_rear_frame_perfect"));

    if(pub_real_pose_){
       br_.sendTransform(tf::StampedTransform(transform_gt * transform.inverse(), sim_time_, "/map", "/odom"));
    }else if (pub_fake_localizer_) {
        if (sim_time_ > las_loc_update_) {
            std::normal_distribution<double> distribution(0.0, 0.5);
            transform_noise_.setOrigin(tf::Vector3(distribution(generator_)*0.03, distribution(generator_)*0.03, 0.0));
            q.setRPY(0, 0, distribution(generator_)*0.01);
            transform_noise_.setRotation(q);

            las_loc_update_ = sim_time_ + ros::Duration(5.0);
        }
        br_.sendTransform(tf::StampedTransform(transform_gt * transform_noise_ * transform.inverse(), sim_time_, "/map", "/odom"));
    }

    // robot_v_x_control_ = robot_v_y_control_ = robot_v_theta_control_ = 0;
}

void SimulationWrapper::sendPose() {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";
    pose.header.seq = odom_seq_++;
    pose.header.stamp = sim_time_;

    pose.pose.position.x = robot_x_;
    pose.pose.position.y = robot_y_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_theta_);
    true_pose_pub_.publish(pose);


    nav_msgs::Odometry odom;
    odom.header = pose.header;
    odom.twist.twist.linear.x = robot_v_x_;
    odom.twist.twist.linear.y = robot_v_y_;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = robot_v_theta_forw;

    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_theta_);
    odom_pose_pub_.publish(odom);
}

void SimulationWrapper::sendLaserScan() {

  if(simulate_front_scan_){
    sensor_msgs::LaserScan scan;
    scan.angle_increment = laser_scanner_.increment();
    scan.angle_max = laser_scanner_.minAngle() + laser_scanner_.increment() + (laser_scan_ranges_.size() - 1);
    scan.angle_min = laser_scanner_.minAngle();
    scan.header.frame_id = "scan_front_frame";
    scan.header.stamp = sim_time_;
    scan.range_max = laser_scanner_.maxRange();
    scan.range_min = 0;
    scan.scan_time = 0;
    scan.time_increment = 0;
    scan.ranges = laser_scan_ranges_;
    laser_scan_front_pub_.publish(scan);
  }

    if(simulate_front_scan_perfect){
    sensor_msgs::LaserScan scan;
    scan.angle_increment = laser_scanner_perfect.increment();
    scan.angle_max = laser_scanner_perfect.minAngle() + laser_scanner_perfect.increment() + (laser_scan_ranges_perfect.size() - 1);
    scan.angle_min = laser_scanner_perfect.minAngle();
    scan.header.frame_id = "scan_front_frame_perfect";
    scan.header.stamp = sim_time_;
    scan.range_max = laser_scanner_perfect.maxRange();
    scan.range_min = 0;
    scan.scan_time = 0;
    scan.time_increment = 0;
    scan.ranges = laser_scan_ranges_perfect;
    laser_scan_front_pub_perfect.publish(scan);
  }

  if(simulate_rear_scan_){
    sensor_msgs::LaserScan scan_rear;
    scan_rear.angle_increment = laser_scanner_rear_.increment();
    scan_rear.angle_max = laser_scanner_rear_.minAngle() + laser_scanner_rear_.increment() + (laser_scan_ranges_.size() - 1);
    scan_rear.angle_min = laser_scanner_rear_.minAngle();
    scan_rear.header.frame_id = "scan_rear_frame";
    scan_rear.header.stamp = sim_time_;
    scan_rear.range_max = laser_scanner_rear_.maxRange();
    scan_rear.range_min = 0;
    scan_rear.scan_time = 0;
    scan_rear.time_increment = 0;
    scan_rear.ranges = laser_scan_rear_ranges_;

    laser_scan_rear_pub_.publish(scan_rear);
  }

   if(simulate_rear_scan_perfect){
    sensor_msgs::LaserScan scan_rear;
    scan_rear.angle_increment = laser_scanner_rear_perfect.increment();
    scan_rear.angle_max = laser_scanner_rear_perfect.minAngle() + laser_scanner_rear_perfect.increment() + (laser_scan_ranges_perfect.size() - 1);
    scan_rear.angle_min = laser_scanner_rear_perfect.minAngle();
    scan_rear.header.frame_id = "scan_rear_frame_perfect";
    scan_rear.header.stamp = sim_time_;
    scan_rear.range_max = laser_scanner_rear_perfect.maxRange();
    scan_rear.range_min = 0;
    scan_rear.scan_time = 0;
    scan_rear.time_increment = 0;
    scan_rear.ranges = laser_scan_rear_ranges_perfect;

    laser_scan_rear_pub_perfect.publish(scan_rear);
  }
}

void SimulationWrapper::sendRobotContour() {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/true_pose";
    marker.header.stamp = sim_time_;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::MODIFY;

    if (last_pose_collides_) {
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else {
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    geometry_msgs::Point point;
    for (unsigned int i = 0; i < robot_contour_wrt_base_link_.size(); i++) {
        point.x = robot_contour_wrt_base_link_[i][0];
        point.y = robot_contour_wrt_base_link_[i][1];
        marker.points.push_back(point);
    }
    point.x = robot_contour_wrt_base_link_[0][0];
    point.y = robot_contour_wrt_base_link_[0][1];
    marker.points.push_back(point);

    robot_contour_pub_.publish(marker);

    marker.header.frame_id = robot_link_;
    robot_odom_contour_pub_.publish(marker);
}
