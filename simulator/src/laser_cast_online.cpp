
#include "simulator/laser_cast_online.h"

caster_online::caster_online(){

    min_occupancy_ = 0.55;
    robot_dim_length_ = 1.0;
    robot_dim_width_ = 0.5;

    simulate_front_scan_ = true;
   // last_pose_collides_ = false;

    laser_scanner_x_ = 1.2;
    laser_scanner_y_ = 0;
    laser_scanner_theta_ = 0;


}

caster_online::~caster_online(){


}

void caster_online::init(){
     
    // Load static map
    get_map_srv_client_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv_get_map;

    std::cout << "############# Wait for map #############" << std::endl;
    do {
        get_map_srv_client_.call(srv_get_map);
    } while (srv_get_map.response.map.data.size() == 0);
    mapCallback(boost::make_shared<nav_msgs::OccupancyGrid>(srv_get_map.response.map));

    
    loadParams();
    distance_map_ptr_.reset(new tools::Map2D<float>());
    tools::map_tools::computeDistanceMap<float>(map_ptr_.get(), distance_map_ptr_.get(), min_occupancy_);
    collision_check_.init(distance_map_ptr_);
    laser_scanner_.init(sensor_max_range_, sensor_min_angle_, sensor_angle_increment_,
            sensor_no_of_rays_, sensor_noise_variance_, map_ptr_);



    if(simulate_front_scan_)
      laser_scan_front_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_front_simulation", 1);
       
     robot_contour_pub_predicted_= nh_.advertise<visualization_msgs::Marker>("/robot_contour_predicted", 1);

      corrected_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("corrected_pose", 1, &caster_online::poseCallback, this);

}
void caster_online::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    corrected_pose_=*msg;
    sim_time_=msg->header.stamp;
    robot_x_=msg->pose.position.x;
    robot_y_=msg->pose.position.y;
    robot_theta_= tf::getYaw(msg->pose.orientation);
   // std::cout<<"get new pose!!!!!"<<"\n";
sendRobotContour( corrected_pose_);

}

void caster_online::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {

    std::cout << "############# Got Map #############" << std::endl;
    map_ptr_.reset(new tools::Map2D<float>());
    map_ptr_->setOffsetXY(map->info.origin.position.x, map->info.origin.position.y);
    map_ptr_->setResolution(map->info.resolution);
    map_ptr_->resize(map->info.width, map->info.height);
    map_ptr_->fill(map->data);
    std::cout << "Map size: " << map_ptr_->getMapSizeX() << ", " << map_ptr_->getMapSizeY() << std::endl;
}

void caster_online::loadParams() {


    min_occupancy_ = 0.55;

    robot_dim_length_ = 0.4;
    robot_dim_width_ = 0.4;

    robot_x_ = 6;
    robot_y_ = 5;
    robot_theta_ = 0;



    // laser_sensor
    sensor_max_range_ = 50;
    sensor_min_angle_  = -M_PI / 2.0;
    sensor_no_of_rays_ = 181;
    sensor_angle_increment_ = M_PI / (sensor_no_of_rays_ - 1);
    sensor_noise_variance_ = 0.0002 / 2.0;
   

}

bool caster_online::spinOnes() {
    //TODO: one simulation loop

    double sin_theta = sin(robot_theta_);
    double cos_theta = cos(robot_theta_);
    double x,y,theta;

  if(simulate_front_scan_){
    x = robot_x_ + laser_scanner_x_ * cos_theta - laser_scanner_y_ * sin_theta;
    y = robot_y_ + laser_scanner_x_ * sin_theta + laser_scanner_y_ * cos_theta;
    theta = robot_theta_ + laser_scanner_theta_;
    if(theta > M_PI){
      theta -= 2 * M_PI;
    }
    if(theta < -M_PI){
      theta += 2 * M_PI;
    }
    laser_scan_ranges_ = laser_scanner_.scan(x,y,theta);
  }
   
    return true;
}
    
void caster_online::sendTF(){

/*
    std::string base_frame_id = "corrected_pose";
    std::string global_frame_id = "map";
    std::string odom_frame_id = "odom";
    tf::Stamped<tf::Pose> odom_to_map;

   tf::Transform  tmp_tf;
    //std::cout<<"I have a pose x "<<corrected_pose_.pose.position.x<<"\n";
    //std::cout<<"I have a pose y "<<corrected_pose_.pose.position.y<<"\n";

   tmp_tf.setOrigin(tf::Vector3(corrected_pose_.pose.position.x, corrected_pose_.pose.position.y, corrected_pose_.pose.position.z));

   tmp_tf.setRotation(tf::Quaternion( corrected_pose_.pose.position.x, corrected_pose_.pose.orientation.y, corrected_pose_.pose.orientation.z, corrected_pose_.pose.orientation.w));
   
    ros::Time stamp = corrected_pose_.header.stamp;
   // mrpt_bridge::convert(timeLastUpdate_, stamp);
   // mrpt_bridge::convert(robotPose, tmp_tf);
    try
    {
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(), stamp,  base_frame_id);
		//ROS_INFO("subtract global_frame (%s) from odom_frame (%s)", global_frame_id.c_str(), odom_frame_id.c_str());
        listenerTF_.transformPose(odom_frame_id, tmp_tf_stamped, odom_to_map);
    }
    catch(tf::TransformException)
    {
		//ROS_INFO("Failed to subtract global_frame (%s) from odom_frame (%s)", global_frame_id.c_str(), odom_frame_id.c_str());
        return;
    }

    tf::Transform latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));



    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Duration transform_tolerance_(0.5);
    ros::Time transform_expiration = (stamp + transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_id, odom_frame_id);
    tf_broadcaster_.sendTransform(tmp_tf_stamped);


*/
/*
//listen for the transform between odom and map (given because of true pose)
tf::StampedTransform transform_true_pose;
     listenerTF_.lookupTransform("/map", "/true_pose", sim_time_, transform_true_pose);





 tf::Quaternion q;
   

 br_.sendTransform(tf::StampedTransform(transform_true_pose , sim_time_, "/map", "/true_pose"));
    tf::Transform transform_front_scan;
    transform_front_scan.setIdentity();
    transform_front_scan.setOrigin(tf::Vector3(laser_scanner_x_, laser_scanner_y_,0.0));
    q.setRPY(0, 0, laser_scanner_theta_);
    transform_front_scan.setRotation(q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_front_scan, sim_time_,"/true_pose", "/scan_front_frame_simulation"));

*/
}
void caster_online::sendCast() {

    spinOnes();//update laser scan pose
    //sendTF();
    //sendLaserScan();

   
 
}

void  caster_online::sendLaserScan() {
    
    //for(int i=0; i<laser_scans.size();i++){
    sensor_msgs::LaserScan scan;
    scan.angle_increment = laser_scanner_.increment();
    scan.angle_max = laser_scanner_.minAngle() + laser_scanner_.increment() + (laser_scan_ranges_.size() - 1);
    scan.angle_min = laser_scanner_.minAngle();
    scan.header.frame_id = "scan_front_frame_simulation";
    scan.header.stamp = sim_time_;
    scan.range_max = laser_scanner_.maxRange();
    scan.range_min = 0;
    scan.scan_time = 0;
    scan.time_increment = 0;
    scan.ranges = laser_scan_ranges_;
    laser_scan_front_pub_.publish(scan);
    //}
}

void caster_online::sendRobotContour( geometry_msgs::PoseStamped pose) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time(0);
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::MODIFY;


    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
 

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    double theta =tf::getYaw(pose.pose.orientation);
    double x,y;
    geometry_msgs::Point point;
    //point 1 [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
     point.x = cos(theta)* (-0.5) -sin(theta)*(-0.55) +pose.pose.position.x;
     point.y = sin(theta)*(-0.5)+cos(theta)*(-0.55)+ pose.pose.position.y;
     marker.points.push_back(point);
    
 //point 2 [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
     point.x = cos(theta)* (-0.5) -sin(theta)*(0.55) +pose.pose.position.x;
     point.y = sin(theta)*(-0.5)+cos(theta)*(0.55)+ pose.pose.position.y;
     marker.points.push_back(point);

 //point 3 [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
     point.x = cos(theta)* (1.75) -sin(theta)*(0.55) +pose.pose.position.x;
     point.y = sin(theta)*(1.75)+cos(theta)*(0.55)+ pose.pose.position.y;
     marker.points.push_back(point);
 //point 4 [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
     point.x = cos(theta)* (1.75) -sin(theta)*(-0.55) +pose.pose.position.x;
     point.y = sin(theta)*(1.75)+cos(theta)*(-0.55)+ pose.pose.position.y;
     marker.points.push_back(point);
 //close contour [-0.5, -0.55], [-0.5, 0.55], [1.75, 0.55], [1.75, -0.55]
     point.x = cos(theta)* (-0.5) -sin(theta)*(-0.55) +pose.pose.position.x;
     point.y = sin(theta)*(-0.5)+cos(theta)*(-0.55)+ pose.pose.position.y;
     marker.points.push_back(point);

    robot_contour_pub_predicted_.publish(marker);

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "caster_online");
    caster_online data;
    data.init();
    ros::Rate r(20);
    //int counter = 0;
    ros::Duration(2).sleep();
    while (ros::ok()) {

    data.sendCast();   
    //data.sendTF();   
    // data.pubTruePose();


      data.sendLaserScan();
        ros::spinOnce();
     r.sleep();
    }

}

