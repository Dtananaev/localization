#ifndef LASER_H
#define	LASER_H

#include <memory>
#include <vector>
#include "map_tools.h"
#include <cmath>
#include <limits>
#include <random>
#include <chrono>

class Laser {
public:
    Laser();
    virtual ~Laser();

    void init(double max_range, double min_angle, double resolution_angle, unsigned int no_of_beams,
            double noise_variance, std::shared_ptr<tools::Map2D<float> > map_ptr);
    std::vector<float> scan(double x, double y, double theta);
    void updateAngle(double &angle, const double increment);
    float rayCast(double x, double y, double theta);
    double maxRange();
    double minAngle();
    double increment();
private:
    double max_range_;
    double min_angle_;
    double resolution_angle_;
    unsigned int no_of_beams_;
    double noise_variance_;
    std::shared_ptr<tools::Map2D<float> > map_ptr_;
    std::vector<float> ranges_;
    std::shared_ptr<std::default_random_engine> generator_;
    std::shared_ptr<std::normal_distribution<double> > normal_distribution_ptr_;
};

#endif	/* LASER_H */

