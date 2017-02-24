/*
 * Author: Denis Tananaev
 * File: laser_cast.h
 *
 */


#ifndef LASER_CAST_H
#define	LASER_CAST_H





#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <ros/package.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include "map_tools.h"

#include "simulator/laser.h"
#include "simulator/collidechecker.h"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

typedef struct pose{
    ros::Time time;
    double x; 
    double y;
    double theta;
}pose;

class caster{

public:
     caster();
    ~caster();
    
    void init();
    void sendTF();
    void loadParams();
    bool load_data(std::string path);
    bool spinOnes();
    void   sendLaserScan();
    void  pubOdomPose();
    void sendRobotContour(geometry_msgs::PoseStamped pose);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
private:

    ros::ServiceClient get_map_srv_client_;
    bool init_map_;
    //robot pose
    float robot_x_;
    float robot_y_;
    float robot_theta_;
    int     counter;
    //parameters
    float min_occupancy_;
    double robot_dim_length_;
    double robot_dim_width_;

    double sensor_max_range_;
    double sensor_min_angle_;
    double sensor_angle_increment_;
    unsigned int sensor_no_of_rays_;
    double sensor_noise_variance_;


      double  laser_scanner_x_ ;
   double laser_scanner_y_ ;
   double laser_scanner_theta_;
bool save_to_file(std::string path, std::string name );
    bool simulate_front_scan_;
    Laser laser_scanner_;
    std::vector<float> laser_scan_ranges_;


    std::shared_ptr<tools::Map2D<float> > map_ptr_;
    std::shared_ptr<tools::Map2D<float> > distance_map_ptr_;
    collidechecker collision_check_;
    ros::NodeHandle nh_;
    ros::Publisher laser_scan_front_pub_,robot_contour_pub_,true_pose_pub_, viz_pub_;
    ros::Time sim_time_;
    tf::TransformBroadcaster br_;

     std::string path_;
    std::vector<pose> true_pose;

    std::vector<std::string> laser;
};

#endif /*LASER_CAST_H*/
