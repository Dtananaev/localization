#include "recording_tools/get_data.h"


save_data::save_data(){
}

save_data::~save_data(){

    std::cout<<"pose size "<<pose.size()<<"\n";
    std::cout<<"odom size "<<odom.size()<<"\n";
    std::cout<<"laser size "<<laser.size()<<"\n";
    if(save_to_file( folder_path_ , name_  )){  
     std::cout<<"DATA SAVED"<<"\n";    
    }
}

void save_data::init(){
      laser_sub_= nh_.subscribe <sensor_msgs::LaserScan> ("scan_front", 1, &save_data::laserCallback, this); // laser scans
     laser_sub_p= nh_.subscribe <sensor_msgs::LaserScan> ("scan_front_perfect", 1, &save_data::PlaserCallback, this); // laser scans
    back_laser_sub_= nh_.subscribe <sensor_msgs::LaserScan> ("scan_rear", 1, &save_data::back_laserCallback, this); // laser scans
    back_laser_sub_p= nh_.subscribe <sensor_msgs::LaserScan> ("scan_rear_perfect", 1, &save_data::Pback_laserCallback, this); // laser scans
      odometry_= nh_.subscribe <nav_msgs::Odometry>("odom", 1, &save_data::odomCallback, this);
      true_pose_=nh_.subscribe <geometry_msgs::PoseStamped>("true_pose", 1, &save_data::poseCallback, this); // real position from simulator


     name_ = "data"; 
    folder_path_ = ros::package::getPath("deep_localization"); 
    ros::Duration(2).sleep();
}

void save_data::odomCallback(const nav_msgs::Odometry::ConstPtr& cmd){
    odomCB_=*cmd;
/*
    odometry temp;

    temp.v=cmd->twist.twist.linear.x;    
    temp.w=cmd->twist.twist.angular.z;

    odomMap[cmd->header.stamp]=temp;
*/
}


void save_data::laserCallback(const sensor_msgs::LaserScan::ConstPtr& cmd){
    laserCB_=*cmd;
   /*      
        std::ostringstream stream;
        std::copy(cmd->ranges.begin(), cmd->ranges.end(), std::ostream_iterator<float>(stream, ","));
        std::string s=stream.str();
        s.erase(s.length()-1);
        laserMap[cmd->header.stamp]=s;
         
*/
}

void save_data::PlaserCallback(const sensor_msgs::LaserScan::ConstPtr& cmd){
    PlaserCB_=*cmd;
   /*      
        std::ostringstream stream;
        std::copy(cmd->ranges.begin(), cmd->ranges.end(), std::ostream_iterator<float>(stream, ","));
        std::string s=stream.str();
        s.erase(s.length()-1);
        laserMap[cmd->header.stamp]=s;
         
*/
}

void save_data::back_laserCallback(const sensor_msgs::LaserScan::ConstPtr& cmd){
    back_laserCB_=*cmd;
   /*      
        std::ostringstream stream;
        std::copy(cmd->ranges.begin(), cmd->ranges.end(), std::ostream_iterator<float>(stream, ","));
        std::string s=stream.str();
        s.erase(s.length()-1);
        laserMap[cmd->header.stamp]=s;
         
*/
}
void save_data::Pback_laserCallback(const sensor_msgs::LaserScan::ConstPtr& cmd){
    Pback_laserCB_=*cmd;
   /*      
        std::ostringstream stream;
        std::copy(cmd->ranges.begin(), cmd->ranges.end(), std::ostream_iterator<float>(stream, ","));
        std::string s=stream.str();
        s.erase(s.length()-1);
        laserMap[cmd->header.stamp]=s;
         
*/
}

void save_data::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& cmd){
    poseCB_=*cmd;     
/*   
    truepose temp;
              
        temp.x=cmd->pose.position.x;
        temp.y=cmd->pose.position.y;

      double roll, pitch, yaw;
      tools::quat2rpy(cmd->pose.orientation.x, cmd->pose.orientation.y, cmd->pose.orientation.z, cmd->pose.orientation.w, roll, pitch, yaw);
       temp.theta=yaw;
    poseMap[cmd->header.stamp]=temp;
*/

}

void save_data::get_data(){
/*
std::cout<<"---------------------------------------------------------------- "<<"\n";
    std::cout<<"POSE "<<poseCB_.header.stamp<<"\n";
     std::cout<<"ODOM "<<odomCB_.header.stamp<<"\n";
    std::cout<<"laser "<<laserCB_.header.stamp<<"\n";
    if(poseCB_.header.stamp==odomCB_.header.stamp && odomCB_.header.stamp==laserCB_.header.stamp){

    std::cout<<"SAME!!!!! "<<"\n";
    }else{
      std::cout<<"DIFFERENT!!!!! "<<"\n";
    }
std::cout<<"---------------------------------------------------------------- "<<"\n";
   */ 
    //POSE
    truepose pose_temp;


    pose_temp.x=poseCB_.pose.position.x;
    pose_temp.y=poseCB_.pose.position.y;

      double roll, pitch, yaw;
      tools::quat2rpy(poseCB_.pose.orientation.x, poseCB_.pose.orientation.y, poseCB_.pose.orientation.z, poseCB_.pose.orientation.w, roll, pitch, yaw);
       pose_temp.theta=yaw;

   pose.push_back( pose_temp);

    //ODOM
    odometry temp;

    temp.v=odomCB_.twist.twist.linear.x;    
    temp.w=odomCB_.twist.twist.angular.z;
    odom.push_back(temp);
    //delta
    truepose delta;
    delta.x=odomCB_.pose.pose.position.x;
    delta.y=odomCB_.pose.pose.position.y;
   double roll1, pitch1, yaw1;
     tools::quat2rpy(odomCB_.pose.pose.orientation.x, odomCB_.pose.pose.orientation.y, odomCB_.pose.pose.orientation.z, odomCB_.pose.pose.orientation.w, roll1, pitch1, yaw1);
    delta.theta=yaw1;
    delta_odom.push_back(delta);

    //laser
        std::ostringstream stream;

        std::copy(laserCB_.ranges.begin(), laserCB_.ranges.end(), std::ostream_iterator<float>(stream, ","));

        std::string s=stream.str();

         s.erase(s.length()-1);
         laser.push_back(s);

           std::ostringstream pstream;

        std::copy(PlaserCB_.ranges.begin(), PlaserCB_.ranges.end(), std::ostream_iterator<float>(pstream, ","));

        std::string ps=pstream.str();
        ps.erase(ps.length()-1);
        plaser.push_back(ps);
  
    //back lasers
        std::ostringstream stream1;

        std::copy(back_laserCB_.ranges.begin(), back_laserCB_.ranges.end(), std::ostream_iterator<float>(stream1, ","));

        std::string s1=stream1.str();

        s1.erase(s1.length()-1);

       back_laser.push_back(s1);



    std::ostringstream stream2;

        std::copy(Pback_laserCB_.ranges.begin(), Pback_laserCB_.ranges.end(), std::ostream_iterator<float>(stream2, ","));

        std::string s2=stream2.str();

        s2.erase(s2.length()-1);

       pback_laser.push_back(s2);

}


bool save_data::save_to_file(std::string path, std::string name ){
    
    std::string odom_file_name = path + "/"+ name + "_odometry.txt";
    std::string laser_file_name = path + "/" +name + "_laser.txt";
    std::string pose_file_name = path + "/" +name + "_pose.txt";
    
    std::string delta_file_name = path + "/"+ name + "_delta.txt";
    std::string back_file_name = path + "/" +name + "_back_laser.txt";


     std::string perfect_laser_file_name = path + "/" +name + "_perfect_laser.txt";
        std::string perfect_back_file_name = path + "/" +name + "_perfect_back_laser.txt";
    //start writing vertices
    std::ofstream dataFile;
    dataFile.open (odom_file_name, std::ios::out | std::ios::trunc );
    if (dataFile.is_open()){ 
         for(int i =0 ; i<odom.size(); i++){
             // dataFile<<save_odom[i].time<<" "<<save_odom[i].v<<" "<<save_odom[i].w<<"\n";
          dataFile<<odom[i].v<<" "<<odom[i].w<<"\n";
        } 
    }   
    else{
        return false;
    }
  dataFile.close();

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

    std::ofstream poseFile;
    poseFile.open (pose_file_name, std::ios::out | std::ios::trunc );
    if (poseFile.is_open()){ 
         for(int i =0 ; i<pose.size(); i++){
            //  poseFile<<save_pose[i].time<<" "<<save_pose[i].x<<" "<<save_pose[i].y<<" "<<save_pose[i].theta<<"\n";
           poseFile<<pose[i].x<<" "<<pose[i].y<<" "<<pose[i].theta<<"\n";
        } 
    }   
    else{
        return false;
    }
  poseFile.close();


  std::ofstream deltaFile;
    deltaFile.open (delta_file_name, std::ios::out | std::ios::trunc );
    if (deltaFile.is_open()){ 
         for(int i =0 ; i<delta_odom.size(); i++){
            //  poseFile<<save_pose[i].time<<" "<<save_pose[i].x<<" "<<save_pose[i].y<<" "<<save_pose[i].theta<<"\n";
           deltaFile<<delta_odom[i].x<<" "<<delta_odom[i].y<<" "<<delta_odom[i].theta<<"\n";
        } 
    }   
    else{
        return false;
    }
  deltaFile.close();

        std::ofstream backFile;
    backFile.open (back_file_name, std::ios::out | std::ios::trunc );
    if (backFile.is_open()){ 
         for(int i =0 ; i<back_laser.size(); i++){
             // laserFile<<save_laser[i].time<<" "<<save_laser[i].result<<"\n";
               backFile<<back_laser[i]<<"\n";
        } 
    }   
    else{
        return false;
    }
  backFile.close();





 std::ofstream plaserFile;
    plaserFile.open (perfect_laser_file_name, std::ios::out | std::ios::trunc );
    if (plaserFile.is_open()){ 
         for(int i =0 ; i<plaser.size(); i++){
             // laserFile<<save_laser[i].time<<" "<<save_laser[i].result<<"\n";
               plaserFile<<plaser[i]<<"\n";
        } 
    }   
    else{
        return false;
    }
  plaserFile.close();


  std::ofstream pbackFile;
    pbackFile.open (perfect_back_file_name, std::ios::out | std::ios::trunc );
    if (pbackFile.is_open()){ 
         for(int i =0 ; i<pback_laser.size(); i++){
             // laserFile<<save_laser[i].time<<" "<<save_laser[i].result<<"\n";
               pbackFile<<pback_laser[i]<<"\n";
        } 
    }   
    else{
        return false;
    }
  pbackFile.close();

 return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_data");


    save_data data;

    data.init();
    ros::spinOnce();
    ros::Rate r(10);
    int counter = 0;
    while (ros::ok()) {
       
     ros::spinOnce();
      data.get_data();
    
        r.sleep();
    }

}

