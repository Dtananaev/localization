#ifndef SIMULATION_H
#define	SIMULATION_H

#include "map_2d.hpp"
#include "map_tools.h"

#include "collidechecker.h"

#include <stdlib.h>     /* srand, rand */
#include <random>


#include <memory>

#include "laser.h"

class Simulation {
public:
    Simulation();
    virtual ~Simulation();
    virtual bool init();
    virtual void loadParams(std::string param_file);
    virtual bool spinOnes();
    void updatePose();
    virtual void updateVelocity(double v_x, double v_y, double v_theta);


protected:

    virtual void computeContour();

    std::shared_ptr<tools::Map2D<float> > map_ptr_;
    std::shared_ptr<tools::Map2D<float> > distance_map_ptr_;

    float min_occupancy_;

    double robot_dim_length_;
    double robot_dim_width_;

    double robot_v_x_, robot_v_y_, robot_v_theta_forw, robot_v_theta_back;
    double robot_x_, robot_y_, robot_theta_;
    double odom_x_, odom_y_, odom_theta_;

    double frequency_;

    double alpha_acceleration_;
    double alpha_deceleration_;


    double sensor_max_range_;
    double sensor_min_angle_;
    double sensor_angle_increment_;
    unsigned int sensor_no_of_rays_;
    double sensor_noise_variance_;

    bool simulate_front_scan_, simulate_rear_scan_,simulate_front_scan_perfect,simulate_rear_scan_perfect;
    Laser laser_scanner_;

    Laser laser_scanner_rear_;

    Laser laser_scanner_perfect;
    Laser laser_scanner_rear_perfect;
    std::vector<float> laser_scan_ranges_perfect;
    std::vector<float> laser_scan_rear_ranges_perfect;

    std::vector<float> laser_scan_ranges_;
    std::vector<float> laser_scan_rear_ranges_;
    double laser_scanner_x_, laser_scanner_y_, laser_scanner_theta_;
    double laser_scanner_rear_x_, laser_scanner_rear_y_, laser_scanner_rear_theta_;

    collidechecker collision_check_;
    bool last_pose_collides_;

    std::vector< std::vector<double> > robot_contour_wrt_base_link_;


    //odom noise params
    double noise_ff_, noise_fs_, noise_fr_, noise_rf_, noise_rs_, noise_rr_;


    std::default_random_engine generator_;
};

#endif	/* SIMULATION_H */

