
#include "simulator/laser.h"

Laser::Laser() {
}

Laser::~Laser() {
}

void Laser::init(double max_range, double min_angle, double resolution_angle, unsigned int no_of_beams,
        double noise_variance, std::shared_ptr<tools::Map2D<float> > map_ptr) {
    max_range_ = max_range;
    min_angle_ = min_angle;
    resolution_angle_ = resolution_angle;
    no_of_beams_ = no_of_beams;
    noise_variance_ = noise_variance;
    map_ptr_ = map_ptr;
    ranges_.resize(no_of_beams_);

    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.reset(new std::default_random_engine(seed));
    normal_distribution_ptr_.reset(new std::normal_distribution<double>(0.0, sqrt(noise_variance_)));
}

std::vector<float> Laser::scan(double x, double y, double theta) {
    double current_angle = theta;
    updateAngle(current_angle, min_angle_);

    for (unsigned int r = 0; r < ranges_.size(); r++) {
        ranges_[r] = rayCast(x, y, current_angle);
        updateAngle(current_angle, resolution_angle_);
    }

    return ranges_;
}

void Laser::updateAngle(double &angle, const double increment) {
    angle += increment;

    if (angle > M_PI * 2) {
        angle -= M_PI * 2;
    }

    //    if (angle < -M_PI) {
    //        angle += M_2_PI;
    //    }
}

float Laser::rayCast(double x, double y, double theta) {
    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // basically: DDA in 3D --> 2D again
    // stolen from octomap and changed

    int start_x, start_y;

    if (map_ptr_->world2map(x, y, start_x, start_y)) {

        if (map_ptr_->getCell(start_x, start_y) == 1) {
        }
    } else {
        return 0;
    }


    // Initialization phase -------------------------------------------------------

    double direction[2];
    direction[0] = cos(theta);
    direction[1] = sin(theta);

    double origin[2];
    origin[0] = x;
    origin[1] = y;

    int step[2];
    double tMax[2];
    double tDelta[2];

    int current[2];
    current[0] = start_x;
    current[1] = start_y;


    double voxelBorder[2];
    map_ptr_->map2world(current[0], current[1], voxelBorder[0], voxelBorder[1]);
    voxelBorder[0] -= 0.5 * map_ptr_->resolution_; //this is the lower left corner
    voxelBorder[1] -= 0.5 * map_ptr_->resolution_; //this is the lower left corner

    for (unsigned int i = 0; i < 2; ++i) {
        // compute step direction
        if (direction[i] > 0.0) step[i] = 1;
        else if (direction[i] < 0.0) step[i] = -1;
        else step[i] = 0;

        // compute tMax, tDelta
        if (step[i] != 0) {
            // corner point of voxel (in direction of ray)
            if (step[i] == 1) {
                voxelBorder[i] += (float) (step[i] * map_ptr_->resolution_ * 1.0);
            }

            tMax[i] = (voxelBorder[i] - origin[i]) / direction[i];
            tDelta[i] = map_ptr_->resolution_ / fabs(direction[i]);
        } else {
            tMax[i] = std::numeric_limits<double>::max();
            tDelta[i] = std::numeric_limits<double>::max();
        }
    }


    // Incremental phase ---------------------------------------------------------



    while (true) {

        unsigned int dim;

        // find minimum tMax:
        if (tMax[0] < tMax[1]) {
            dim = 0;
        } else {
            dim = 1;
        }

        // advance in direction "dim"
        current[dim] += step[dim];
        tMax[dim] += tDelta[dim];


        if (sqrt((current[0] - start_x) * (current[0] - start_x)
                + (current[1] - start_y) * (current[1] - start_y)) * map_ptr_->resolution_ > max_range_) {
            return max_range_;
        } else {
            float value;
            if (map_ptr_->getCell(current[0], current[1], value)) {
                if (value < 0.55) {
                    return sqrt((current[0] - start_x) * (current[0] - start_x)
                            + (current[1] - start_y) * (current[1] - start_y)) * map_ptr_->resolution_
                            + (*normal_distribution_ptr_)(*generator_);
                }
            } else {
                return max_range_;
            }

        }
    }// end while

}

double Laser::maxRange() {
    return max_range_;
}

double Laser::minAngle() {
    return min_angle_;
}

double Laser::increment() {
    return resolution_angle_;
}
