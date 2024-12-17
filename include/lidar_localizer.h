#ifndef LIDAR_LOCALIZER_H_
#define LIDAR_LOCALIZER_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "common_lib.h"
#include "use-ikfom.hpp"

#define foreach BOOST_FOREACH
#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

typedef std::vector<Eigen::Matrix<double, 5, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 5, 1>>> TlsPositionVector; // [x, y, z, projectid, scanid]

struct Pose3d {
    SO3 R;
    V3D p;
    Pose3d() : R(Eye3d), p(Zero3d) {}
    Pose3d(const SO3 &R_, const V3D &p_) : R(R_), p(p_) {}
};


enum ODOM_MODE {
    Odom = 0,
    LocToMap,
    LocWithOdom
};

std::string OdomModeToString(int mode) {
    switch (mode) {
        case Odom:
            return "Odom";
        case LocToMap:
            return "LocToMap";
        case LocWithOdom:
            return "LocWithOdom";
        default:
            return "Unknown " + std::to_string(mode);
    }
}


class LidarLocalizer {
public:
    // Localize lidar scans to a prior map, using odometry poses, covariance, and unskewed (accumulated) scans.
    LidarLocalizer() {};
    void initialize() {}
    void setMode(int mode) {}
    void push() {}
};

#endif