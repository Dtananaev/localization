/*
 * File: get_data.h
 * Author: Denis Tananaev
 *
 */

#ifndef GET_DATA_H
#define GET_DATA_H

#include "recording_tools/conversions.h"
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <ros/package.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>


#include <iterator>  // ostream_iterator

#include <algorithm> // copy



typedef struct odom{
   
    float v;
    float w;
}odometry;


typedef struct pose{
    double x; 
    double y;
    double theta;
}truepose;

typedef struct data{
    truepose p;          
    odometry o;
    std::string l;
}dat;

class save_data{
public:
    save_data();
    ~save_data();

    void init();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& cmd);
    void back_laserCallback(const sensor_msgs::LaserScan::ConstPtr& cmd);
    void PlaserCallback(const sensor_msgs::LaserScan::ConstPtr& cmd);
    void Pback_laserCallback(const sensor_msgs::LaserScan::ConstPtr& cmd);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& cmd);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& cmd);
   
    void get_data();
 

    bool save_to_file(std::string path, std::string name );

    //std::map<ros::Time,odometry> odomMap;
   // std::map<ros::Time, std::string> laserMap; 
   // std::map<ros::Time, truepose> poseMap;
   
    std::vector<truepose> pose;
    std::vector<odometry> odom;
     std::vector<truepose> delta_odom;
    std::vector<std::string> laser;
    std::vector<std::string> back_laser;


    std::vector<std::string> plaser;
    std::vector<std::string> pback_laser;
private:
    
    std::string  name_;
    std::string folder_path_;

    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_,back_laser_sub_, laser_sub_p,  back_laser_sub_p; // laser scans
    ros::Subscriber odometry_; // odom velocity
    ros::Subscriber true_pose_; // real position from simulator


    geometry_msgs::PoseStamped poseCB_;
    sensor_msgs::LaserScan   laserCB_, back_laserCB_, PlaserCB_,Pback_laserCB_;
    nav_msgs::Odometry odomCB_;
    
};


#endif /*GET_DATA_H*/


