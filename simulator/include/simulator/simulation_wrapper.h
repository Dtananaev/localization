#ifndef SIMULATION_WRAPPER_H
#define	SIMULATION_WRAPPER_H
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include "simulator/simulation.h"
#include <visualization_msgs/Marker.h>
#include <ros/package.h>
class SimulationWrapper : public Simulation {
public:
    SimulationWrapper();
    virtual ~SimulationWrapper();
    bool init();
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void velKeyboardCallback(const geometry_msgs::TwistStamped::ConstPtr& twi);
    void velCallback(const geometry_msgs::Twist::ConstPtr& twi);
    void placeRobotCallback(const geometry_msgs::Pose2D::ConstPtr& pos);
    void placeRobotRvizCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos);
    bool spinOnes();
    void sendTF();
    void sendPose();
    void sendLaserScan();
    void sendRobotContour();

protected:
    ros::NodeHandle nh_;
    ros::Publisher laser_scan_front_pub_;
    ros::Publisher laser_scan_front_pub_perfect;
    ros::Publisher laser_scan_rear_pub_;
    ros::Publisher laser_scan_rear_pub_perfect;
    ros::Publisher odom_pose_pub_;
    ros::Publisher true_pose_pub_;
    ros::Subscriber joy_sub_, vel_sub_, place_sub_, place_rviz_sub_, vel_keyboard_sub;
    ros::Publisher robot_contour_pub_;
    ros::Publisher robot_odom_contour_pub_;
    ros::ServiceClient get_map_srv_client_;
    bool init_map_;
    std::string robot_link_;
    bool pub_real_pose_;
    bool pub_fake_localizer_;
    double joy_gain_x_, joy_gain_y_, joy_gain_theta_;
    double robot_v_x_control_, robot_v_y_control_, robot_v_theta_control_;
    unsigned int odom_seq_;
    tf::TransformBroadcaster br_;
    ros::Time sim_time_;
    ros::Time las_loc_update_;
    tf::Transform transform_noise_;
};

#endif	/* SIMULATION_WRAPPER_H */

