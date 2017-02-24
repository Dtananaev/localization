
#ifndef COLLIDECHECKER_H_
#define COLLIDECHECKER_H_
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include "map_2d.hpp"

class collidechecker {
public:
    collidechecker();
    virtual ~collidechecker();
    bool init(std::shared_ptr<tools::Map2D<float> > map_ptr);
    bool collCheck(double x, double y, double theta, const double & width, const double & height,
            double & guaranteed_clearance);
    bool collCheck(int x, int y, double theta, const double & width, const double & height,
            double & guaranteed_clearance);
    bool collCheck(Eigen::Affine2d center, const double &width, const double &height,
            double& guaranteed_clearance);
    bool collCheck(Eigen::Affine2d center, const double &radius, double& guaranteed_clearance);    
private:
    bool check(Eigen::Affine2d center, const double &width, const double &height,
            double& guaranteed_clearance, unsigned int calldepth);
    std::shared_ptr<tools::Map2D<float> > map_ptr_;
    double step_;

};
#endif /* COLLIDECHECKER_H_ */
