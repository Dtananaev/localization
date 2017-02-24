
#include "simulator/laser_cast.h"

caster::caster(){

    min_occupancy_ = 0.55;
    robot_dim_length_ = 1.0;
    robot_dim_width_ = 0.5;

    simulate_front_scan_ = true;
   // last_pose_collides_ = false;

    laser_scanner_x_ = 1.2;
    laser_scanner_y_ = 0;
    laser_scanner_theta_ = 0;

    counter=0;
}

caster::~caster(){


}

void caster::init(){
       path_ = ros::package::getPath("minisim"); 
      if( load_data( path_)){
        std::cout<<"data loaded"<<"\n";
    }
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

    robot_contour_pub_= nh_.advertise<visualization_msgs::Marker>("/robot_contour_odom", 1);
    viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/odom_pose_viz", 1);    
    true_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("odom_pose", 1);

}
void caster::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {

    std::cout << "############# Got Map #############" << std::endl;
    map_ptr_.reset(new tools::Map2D<float>());
    map_ptr_->setOffsetXY(map->info.origin.position.x, map->info.origin.position.y);
    map_ptr_->setResolution(map->info.resolution);
    map_ptr_->resize(map->info.width, map->info.height);
    map_ptr_->fill(map->data);
    std::cout << "Map size: " << map_ptr_->getMapSizeX() << ", " << map_ptr_->getMapSizeY() << std::endl;
}

void caster::loadParams() {


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

bool caster::spinOnes() {
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
   

    // record simulated laser to the file laser
        std::ostringstream stream;

        std::copy(laser_scan_ranges_.begin(), laser_scan_ranges_.end(), std::ostream_iterator<float>(stream, ","));

        std::string s=stream.str();

         s.erase(s.length()-1);
         laser.push_back(s);
    return true;
}

void caster::sendTF() {

    robot_x_=true_pose[counter].x;
    robot_y_=true_pose[counter].y;
    robot_theta_= true_pose[counter].theta;
    spinOnes();//update laser scan pose

 
    sim_time_=ros::Time(0);
    tf::Quaternion q;
    tf::Transform transform_gt;
    transform_gt.setOrigin(tf::Vector3(robot_x_, robot_y_, 0.0));
    q.setRPY(0, 0, robot_theta_);
    transform_gt.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_gt, sim_time_, "/map", "/odom_pose"));
   
 
    tf::Transform transform_front_scan;
    transform_front_scan.setIdentity();
    transform_front_scan.setOrigin(tf::Vector3(laser_scanner_x_, laser_scanner_y_,0.0));
    q.setRPY(0, 0, laser_scanner_theta_);
    transform_front_scan.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_front_scan, sim_time_,"/odom_pose", "/scan_front_frame_simulation"));
      
     sendLaserScan();

     if(counter<true_pose.size()){

        pubOdomPose();
        counter++;
    }
 if(counter==true_pose.size()){
            counter++;
     std::cout<<"laser size "<<laser.size()<<"\n";
    std::string name_="laser";
    if(save_to_file( path_ , name_  )){  
     std::cout<<"DATA SAVED"<<"\n";  
        ros::shutdown(); 
    }

}
}

bool caster::load_data(std::string path){
    pose temp;

    std::ifstream file_stream_pose(path +"/"+ "odom_pose.txt");
    if (file_stream_pose.is_open()) { // check if file exsist
         std::string line;
         while (std::getline(file_stream_pose, line)) {
            std::istringstream file_stream_pose(line);
             double x, y, theta;
             
            file_stream_pose>>x>>y>>theta;
            temp.x=x;
            temp.y=y;
            temp.theta=theta;
            true_pose.push_back(temp);
         }
         file_stream_pose.close();
    }else{
       return false;
    }

   return true;
}


void caster::sendRobotContour( geometry_msgs::PoseStamped pose) {

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

    robot_contour_pub_.publish(marker);

}
void caster::pubOdomPose() {
    //the pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";

    //todo mark current goal and plath path
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5);
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 1.0;
 



    //the pose
    pose.pose.position.x =  robot_x_;
    pose.pose.position.y = robot_y_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_theta_);
        
        viz_pub_.publish(marker);
        true_pose_pub_.publish(pose);
        sendRobotContour(pose);

    

}

void caster::sendLaserScan() {
    
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



bool caster::save_to_file(std::string path, std::string name ){
    
   
    std::string laser_file_name = path + "/" +name + "_simulated.txt";
   

    std::ofstream laserFile;
    laserFile.open (laser_file_name, std::ios::out | std::ios::trunc );
    if (laserFile.is_open()){ 
         for(int i =0 ; i<laser.size(); i++){
             // laserFile<<save_laser[i].time<<" "<<save_laser[i].result<<"\n";
               laserFile<<laser[i]<<"\n";
        } 
    }   
    else{
        return false;
    }
  laserFile.close();

 return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "caster");
    caster data;
    data.init();
    ros::Rate r(1000);
    //int counter = 0;
        ros::Duration(3).sleep();
    while (ros::ok()) {
  //      ros::spinOnce();
    data.sendTF();      
    // data.pubTruePose();


     //  data.sendLaserScan();
    
     r.sleep();
    }

}

