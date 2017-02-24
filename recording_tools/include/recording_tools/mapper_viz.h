/*
 * File: mapper_viz.h
 * Author: Denis Tananaev
 *
 */

#ifndef MAPPER_VIZ_H
#define MAPPER_VIZ_H

#include <iostream>
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



typedef struct pose{
    ros::Time time;
    double x; 
    double y;
    double theta;
}pose;

class mapper_viz{
    public:
    mapper_viz();
    ~mapper_viz();


    void init();
    void pubTruePose();
    bool load_data(std::string path);
    void sendRobotContour( geometry_msgs::PoseStamped pose);
    void sendLaserScan(int i);
    void sendTF();
    private:

   // int speed_;
    ros::Publisher viz_pub_, viz_pub_predicted_,true_pose_pub_,predicted_pose_pub_ ,  robot_contour_pub_,robot_contour_pub_predicted_, laser_scan_front_pub_;
    ros::NodeHandle nh_;
    std::string path_;
        int counter;
    std::vector<pose> true_pose;
    std::vector<std::vector<float>> laser_scans;

    double sensor_max_range_;
    double sensor_min_angle_;
    double sensor_angle_increment_;
    unsigned int sensor_no_of_rays_;
    double sensor_noise_variance_;

   double  laser_scanner_x_ ;
   double laser_scanner_y_ ;
   double laser_scanner_theta_;

    tf::TransformBroadcaster br_;

    ros::Time sim_time_;
    ros::Time las_loc_update_;
    tf::Transform transform_noise_;
    
    float robot_x_;
    float robot_y_;
    float robot_theta_;
};


#endif /*MAPPER_VIZ_H*/
