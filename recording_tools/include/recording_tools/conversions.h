

#ifndef TOOLS_CONVERSIONS_H
#define	TOOLS_CONVERSIONS_H


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace tools {

    template<class T>
    void quat2rpy(const T &x, const T &y, const T &z, const T &w, T &roll, T &pitch, T &yaw) {

        roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        pitch = asin(2.0 * (w * y - z * x));
        yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    }

    template<class T>
    T quat2yaw(const T &x, const T &y, const T &z, const T &w) {

        return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    }


    template<class T>
    void xytheta2eigen2d(const T &x, const T &y, const T &theta, Eigen::Isometry2d &tfe_odom_front) {
        tfe_odom_front.setIdentity();
        tfe_odom_front.rotate(Eigen::Rotation2Dd(theta));
        tfe_odom_front.translation().x() = x;
        tfe_odom_front.translation().y() = y;
    }

    template<class T>
    void tf2eigen2d(const T &x, const T &y, const T &qx,
            const T &qy, const T &qz, const T &qw, Eigen::Isometry2d &tfe_odom_front) {

        T theta = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        tfe_odom_front.setIdentity();
        tfe_odom_front.rotate(Eigen::Rotation2Dd(theta));
        tfe_odom_front.translation().x() = x;
        tfe_odom_front.translation().y() = y;
    }

    //    double ft2rot(Eigen::Isometry2d &tf) {
    //        return atan2(tf.matrix()(1, 0), tf.matrix()(0, 0));
    //    }



}

#endif	/* TOOLS_CONVERSIONS_H */

