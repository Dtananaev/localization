/*
 * File: viz_data.h
 * Author: Denis Tananaev
 *
 */

#ifndef VIZ_DATA_H
#define VIZ_DATA_H

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

class viz{
    public:
    viz();
    ~viz();


    void init();

    bool load_data(std::string path);
    std::vector<pose> true_pose;
    std::vector<pose> predicted_pose;
    std::vector<pose> odom_pose;

   void sendPose();
    void pubTruePose();
    void pubPredictPose();
    void pubOdomPose();
    void sendRobotContour( geometry_msgs::PoseStamped pose);
    void sendRobotContourPredicted( geometry_msgs::PoseStamped pose);
    void sendRobotContourOdom( geometry_msgs::PoseStamped pose);
    private:

    int speed_;
    ros::Publisher viz_pub_, viz_pub_predicted_,true_pose_pub_,predicted_pose_pub_ ,  robot_contour_pub_,robot_contour_pub_predicted_;
    ros::Publisher pose_odom_pub_, viz_odom_pub_, robot_contour_odom_pub_;
    ros::NodeHandle nh_;
    std::string path_;



};


#endif /*VIZ_DATA_H*/
