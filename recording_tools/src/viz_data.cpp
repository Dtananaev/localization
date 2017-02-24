

#include "recording_tools/viz_data.h"

viz::viz(){
}
viz::~viz(){
}

void viz::init(){

    nh_.param<int>("speed", speed_, 1);
      //for true pose


      path_ = ros::package::getPath("deep_localization");
      if( load_data( path_)){
        std::cout<<"data loaded"<<"\n";
    }
     viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/true_pose_viz", 1);
     viz_pub_predicted_ = nh_.advertise<visualization_msgs::Marker>("/predicted_pose_viz", 1);
    robot_contour_pub_= nh_.advertise<visualization_msgs::Marker>("/robot_contour", 1);
            robot_contour_pub_predicted_= nh_.advertise<visualization_msgs::Marker>("/robot_contour_predicted", 1);
     true_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("true_pose", 1);
     predicted_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("predicted_pose", 1);

     pose_odom_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("odom_pose", 1);
     viz_odom_pub_ = nh_.advertise<visualization_msgs::Marker>("/odom_pose_viz", 1);
     robot_contour_odom_pub_= nh_.advertise<visualization_msgs::Marker>("/odom_robot_contour", 1);

}

bool viz::load_data(std::string path){
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

   std::ifstream fileP_stream_pose(path +"/"+"predicted_pose.txt");
    if (fileP_stream_pose.is_open()) { // check if file exsist
         std::string line;
         while (std::getline(fileP_stream_pose, line)) {
            std::istringstream fileP_stream_pose(line);
             double x, y, theta;

            fileP_stream_pose>>x>>y>>theta;
            temp.x=x;
            temp.y=y;
            temp.theta=theta;
            predicted_pose.push_back(temp);
         }
         fileP_stream_pose.close();

   }else{
       return false;
    }

    std::ifstream file_stream_odom_pose(path +"/"+ "odom_pose.txt");
    if (file_stream_odom_pose.is_open()) { // check if file exsist
         std::string line;
         while (std::getline(file_stream_odom_pose, line)) {
            std::istringstream file_stream_odom_pose(line);
             double x, y, theta;

            file_stream_odom_pose>>x>>y>>theta;
            temp.x=x;
            temp.y=y;
            temp.theta=theta;
            odom_pose.push_back(temp);
         }
         file_stream_odom_pose.close();
    }else{
       return false;
    }
   return true;

}

void viz::sendRobotContour( geometry_msgs::PoseStamped pose) {

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

void viz::sendRobotContourPredicted( geometry_msgs::PoseStamped pose) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time(0);
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::MODIFY;


    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
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

void viz::sendRobotContourOdom( geometry_msgs::PoseStamped pose) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time(0);
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::MODIFY;


    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;


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

    robot_contour_odom_pub_.publish(marker);

}

void viz::pubTruePose() {
    //the pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";

    //todo mark current goal and plath path
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(speed_);
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

    //for(int i=0; i<true_pose.size(); i++){
    if(true_pose.size()>=0){

    //the pose
    pose.pose.position.x = true_pose[0].x;
    pose.pose.position.y = true_pose[0].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(true_pose[0].theta);

    marker.pose.position.x = true_pose[0].x;
    marker.pose.position.y = true_pose[0].y;
    marker.pose.position.z = 0;
    true_pose.erase(true_pose.begin());

        viz_pub_.publish(marker);
         true_pose_pub_.publish(pose);
        sendRobotContour(pose);
    }



}




void viz::pubPredictPose() {
      //the pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";



    //todo mark current goal and plath path
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(speed_);
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
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;


    if(predicted_pose.size()>=0){

   //the pose
    pose.pose.position.x = predicted_pose[0].x;
    pose.pose.position.y = predicted_pose[0].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(predicted_pose[0].theta);

    marker.pose.position.x = predicted_pose[0].x;
    marker.pose.position.y = predicted_pose[0].y;
    marker.pose.position.z = 0;
    predicted_pose.erase(predicted_pose.begin());

         viz_pub_predicted_.publish(marker);
          predicted_pose_pub_.publish(pose);
        sendRobotContourPredicted( pose);
    }

}

void viz::pubOdomPose() {
      //the pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";



    //todo mark current goal and plath path
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(speed_);
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
    marker.color.a = 0.5;
    marker.color.b = 1.0;
    marker.color.g = 0.0;


    if(odom_pose.size()>=0){

   //the pose
    pose.pose.position.x = odom_pose[0].x;
    pose.pose.position.y = odom_pose[0].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[0].theta);

    marker.pose.position.x = odom_pose[0].x;
    marker.pose.position.y = odom_pose[0].y;
    marker.pose.position.z = 0;
    odom_pose.erase(odom_pose.begin());

         viz_odom_pub_.publish(marker);
          pose_odom_pub_.publish(pose);
        sendRobotContourOdom( pose);
    }

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "viz_data");
    viz data;
    data.init();
    ros::Rate r(100);
    //int counter = 0;
        ros::Duration(3).sleep();
    while (ros::ok()) {
        ros::spinOnce();
       data.pubTruePose();

     data.pubPredictPose();
     data.pubOdomPose();

       r.sleep();
    }

}


