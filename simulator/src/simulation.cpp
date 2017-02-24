#include "simulator/simulation.h"
#include "simulator/collidechecker.h"
Simulation::Simulation() {
    min_occupancy_ = 0.55;
    robot_dim_length_ = 1.0;
    robot_dim_width_ = 0.5;
    robot_x_ = 6.0;
    robot_y_ = 5.3;
    robot_theta_ = 0;
    frequency_ = 50;
    odom_x_ = odom_y_ = odom_theta_ = 0;
    robot_v_x_ = 0;
    robot_v_theta_forw = 0.3;
    simulate_front_scan_ = true;
    simulate_rear_scan_ = true;
    simulate_front_scan_perfect=true;
    simulate_rear_scan_perfect=true;
    last_pose_collides_ = false;
}

Simulation::~Simulation() {
}

bool Simulation::init() {
    distance_map_ptr_.reset(new tools::Map2D<float>());
    tools::map_tools::computeDistanceMap<float>(map_ptr_.get(), distance_map_ptr_.get(), min_occupancy_);
    collision_check_.init(distance_map_ptr_);
    laser_scanner_.init(sensor_max_range_, sensor_min_angle_, sensor_angle_increment_,
            sensor_no_of_rays_, sensor_noise_variance_, map_ptr_);
    laser_scanner_perfect.init(sensor_max_range_, sensor_min_angle_, sensor_angle_increment_,
            sensor_no_of_rays_, 0, map_ptr_);
    laser_scanner_rear_.init(sensor_max_range_, sensor_min_angle_, sensor_angle_increment_,
            sensor_no_of_rays_, sensor_noise_variance_, map_ptr_);
    laser_scanner_rear_perfect.init(sensor_max_range_, sensor_min_angle_, sensor_angle_increment_,
            sensor_no_of_rays_, 0, map_ptr_);

    computeContour();

    return true;
}

void Simulation::loadParams(std::string param_file) {
    min_occupancy_ = 0.55;
    robot_dim_length_ = 0.4;
    robot_dim_width_ = 0.4;
    robot_x_ = 6;
    robot_y_ = 5;
    robot_theta_ = 0;
    frequency_ = 50;
    alpha_acceleration_ = 0.0;
    sensor_max_range_ = 50;
    sensor_min_angle_ = -M_PI / 2.0;
    sensor_no_of_rays_ = 181;
    sensor_angle_increment_ = M_PI / (sensor_no_of_rays_ - 1);
    sensor_noise_variance_ = 0.0002 / 2.0;
   
    //odom noise
    noise_ff_ = 0.1;
    noise_fs_ = 0.0;
    noise_fr_ = 0.1;
    noise_rf_ = 0.1;
    noise_rs_ = 0.1;
    noise_rr_ = 0.1;

}

bool Simulation::spinOnes() {
    updatePose();
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

    if(simulate_front_scan_perfect){
    x = robot_x_ + laser_scanner_x_ * cos_theta - laser_scanner_y_ * sin_theta;
    y = robot_y_ + laser_scanner_x_ * sin_theta + laser_scanner_y_ * cos_theta;
    theta = robot_theta_ + laser_scanner_theta_;
    if(theta > M_PI){
      theta -= 2 * M_PI;
    }
    if(theta < -M_PI){
      theta += 2 * M_PI;
    }
    laser_scan_ranges_perfect = laser_scanner_perfect.scan(x,y,theta);
  }

  if(simulate_rear_scan_){
    x = robot_x_ + laser_scanner_rear_x_ * cos_theta - laser_scanner_rear_y_ * sin_theta;
    y = robot_y_ + laser_scanner_rear_x_ * sin_theta + laser_scanner_rear_y_ * cos_theta;
    theta = robot_theta_ + laser_scanner_rear_theta_;
    if(theta > M_PI){
      theta -= 2 * M_PI;
    }
    if(theta < -M_PI){
      theta += 2 * M_PI;
    }
    laser_scan_rear_ranges_ = laser_scanner_rear_.scan(x,y,theta);
  }


    if(simulate_rear_scan_perfect){
    x = robot_x_ + laser_scanner_rear_x_ * cos_theta - laser_scanner_rear_y_ * sin_theta;
    y = robot_y_ + laser_scanner_rear_x_ * sin_theta + laser_scanner_rear_y_ * cos_theta;
    theta = robot_theta_ + laser_scanner_rear_theta_;
    if(theta > M_PI){
      theta -= 2 * M_PI;
    }
    if(theta < -M_PI){
      theta += 2 * M_PI;
    }
    laser_scan_rear_ranges_perfect = laser_scanner_rear_perfect.scan(x,y,theta);
  }
    return true;
}

void Simulation::updateVelocity(double v_x, double v_y, double v_theta) {
    robot_v_x_ = alpha_acceleration_ * robot_v_x_ + (1 - alpha_acceleration_) * v_x;
    robot_v_y_ = alpha_acceleration_ * robot_v_y_ + (1 - alpha_acceleration_) * v_y;
    robot_v_theta_forw = alpha_acceleration_ * robot_v_theta_forw + (1 - alpha_acceleration_) * v_theta;

    if (fabs(robot_v_x_) < 0.0001) {
        robot_v_x_ = 0;
    }
    if (fabs(robot_v_y_) < 0.0001) {
        robot_v_y_ = 0;
    }
    if (fabs(robot_v_theta_forw) < 0.0001) {
        robot_v_theta_forw = 0;
    }
}

void Simulation::updatePose() {

    robot_x_ += (robot_v_x_ * cos(robot_theta_) - robot_v_y_ * sin(robot_theta_)) / frequency_;
    robot_y_ += (robot_v_x_ * sin(robot_theta_) + robot_v_y_ * cos(robot_theta_)) / frequency_;
    robot_theta_ += robot_v_theta_forw / frequency_;

    std::normal_distribution<double> distribution(0.0, 1.0);

    double robot_v_x_noise_ = robot_v_x_
            + robot_v_x_ * distribution(generator_) * noise_ff_
            + robot_v_theta_forw * distribution(generator_) * noise_rf_;
    double robot_v_y_noise_ = robot_v_y_
            + robot_v_x_ * distribution(generator_) * noise_fs_
            + robot_v_theta_forw * distribution(generator_) * noise_rs_;
    double robot_v_theta_forw_noise_ = robot_v_theta_forw
            + robot_v_x_ * distribution(generator_) * noise_fr_
            + robot_v_theta_forw * distribution(generator_) * noise_rr_;

    odom_x_ += (robot_v_x_noise_ * cos(odom_theta_) - robot_v_y_noise_ * sin(odom_theta_)) / frequency_;
    odom_y_ += (robot_v_x_noise_ * sin(odom_theta_) + robot_v_y_noise_ * cos(odom_theta_)) / frequency_;
    odom_theta_ += robot_v_theta_forw_noise_ / frequency_;

    double clearance = 0;

    if (collision_check_.collCheck(robot_x_, robot_y_, robot_theta_, robot_dim_length_, robot_dim_width_, clearance)) {
        robot_theta_ -= robot_v_theta_forw / frequency_;
        robot_x_ -= (robot_v_x_ * cos(robot_theta_) - robot_v_y_ * sin(robot_theta_)) / frequency_;
        robot_y_ -= (robot_v_x_ * sin(robot_theta_) + robot_v_y_ * cos(robot_theta_)) / frequency_;
        robot_v_x_ = 0;
        robot_v_y_ = 0;
        robot_v_theta_forw = 0;
        //std::cerr << "Robot collides!\n";
        last_pose_collides_ = true;
    } else {
        last_pose_collides_ = false;
    }


}

void Simulation::computeContour() {

    robot_contour_wrt_base_link_.clear();
    std::vector<double> point;
    point.push_back(0);
    point.push_back(0);

    point[0] = robot_dim_length_ / 2.0;
    point[1] = robot_dim_width_ / 2.0;
    robot_contour_wrt_base_link_.push_back(point);

    point[0] = -robot_dim_length_ / 2.0;
    point[1] = robot_dim_width_ / 2.0;
    robot_contour_wrt_base_link_.push_back(point);

    point[0] = -robot_dim_length_ / 2.0;
    point[1] = -robot_dim_width_ / 2.0;
    robot_contour_wrt_base_link_.push_back(point);

    point[0] = robot_dim_length_ / 2.0;
    point[1] = -robot_dim_width_ / 2.0;
    robot_contour_wrt_base_link_.push_back(point);
}
