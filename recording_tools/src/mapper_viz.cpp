

#include "recording_tools/mapper_viz.h"
#include <tf/transform_broadcaster.h>
mapper_viz::mapper_viz(){
    // laser_sensor
    sensor_max_range_ = 50;
    sensor_min_angle_ = -M_PI / 2.0;
    sensor_no_of_rays_ = 180;
    sensor_angle_increment_ = M_PI / (sensor_no_of_rays_ - 1);
    sensor_noise_variance_ = 0.0002 / 2.0;

    laser_scanner_x_ = 1.2;
    laser_scanner_y_ = 0;
    laser_scanner_theta_ = 0;
    counter=0;
}
mapper_viz::~mapper_viz(){
}



void mapper_viz::sendTF() {
        tf::Quaternion q;
  //  for(int i=0; i<true_pose.size();i++){
    sim_time_=ros::Time(0);
    tf::Transform transform_gt;
    transform_gt.setOrigin(tf::Vector3(true_pose[counter].x, true_pose[counter].y, 0.0));
    q.setRPY(0, 0, true_pose[counter].theta);
    transform_gt.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_gt, sim_time_, "/map", "/true_pose"));
   

  robot_x_=true_pose[counter].x;
  robot_y_=true_pose[counter].y;
  robot_theta_=true_pose[counter].theta;



    tf::Transform transform_front_scan;
    transform_front_scan.setIdentity();
    transform_front_scan.setOrigin(tf::Vector3(laser_scanner_x_, laser_scanner_y_,0.0));
    q.setRPY(0, 0, laser_scanner_theta_);
    transform_front_scan.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform_front_scan, sim_time_, "/true_pose", "/scan_front_frame"));

   sendLaserScan(counter);
        //ros::Duration(0.1).sleep();
  // }
    
    if(counter<true_pose.size()){
    pubTruePose();
    counter++;
    }
}


void mapper_viz::init(){

 
      path_ = ros::package::getPath("deep_localization"); 
      if( load_data( path_)){
        std::cout<<"data loaded"<<"\n";
    }


     viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/true_pose_viz", 1);
     robot_contour_pub_= nh_.advertise<visualization_msgs::Marker>("/robot_contour", 1);    
     true_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("true_pose", 1);
     laser_scan_front_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_front", 1);

}

bool mapper_viz::load_data(std::string path){
    pose temp;

    std::ifstream file_stream_pose(path +"/"+ "data_pose.txt");
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

   std::ifstream fileP_stream_pose(path +"/"+"map_scans.txt");
   std::string delim =" ";
   //std::string delim =",";
    if (fileP_stream_pose.is_open()) { // check if file exsist
         std::string line;
         std::vector<float> one_row;
         while (std::getline(fileP_stream_pose, line)) {
            one_row.clear();
            std::istringstream fileP_stream_pose(line);
                        auto start=line.find("");
                        auto end = line.find(delim);
                   while(end != std::string::npos){                    
                       std::string temp1=line.substr(start, end - start);
                       float ls= atof(temp1.c_str());
                       one_row.push_back(ls);
                        //message
                       // std::cout <<"LASER SCANS: "<<line.substr(start, end - start) << std::endl;
                        start = end + delim.length();
                         end = line.find(delim, start);
                           if(line=="\n"){
                                std::cout<<"end of line"<<"\n";
                            }
                      
                    } 
             std::cout <<"SIZE of One raw: "<< one_row.size()<<"\n"; 
            laser_scans.push_back(one_row);
         }
         fileP_stream_pose.close();

   }else{
       return false;
    }
   return true;



}

void mapper_viz::sendRobotContour( geometry_msgs::PoseStamped pose) {

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



void mapper_viz::pubTruePose() {
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

void mapper_viz::sendLaserScan(int i) {
    
    //for(int i=0; i<laser_scans.size();i++){
    sensor_msgs::LaserScan scan;
    scan.angle_increment = sensor_angle_increment_;
    scan.angle_max = sensor_min_angle_ + sensor_angle_increment_ + (laser_scans[i].size() - 1);
    scan.angle_min =  sensor_min_angle_;
    scan.header.frame_id ="scan_front_frame";
    scan.header.stamp = sim_time_;
    scan.range_max = sensor_max_range_;
    scan.range_min = 0;
    scan.scan_time = 0;
    scan.time_increment = 0;
    scan.ranges = laser_scans[i];
    laser_scan_front_pub_.publish(scan);
    //}
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "mapper_viz");
    mapper_viz data;
    data.init();
    ros::Rate r(10);
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


