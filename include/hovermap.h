#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <vector>


namespace hovermap_velodyne {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;  // This macro adds the x, y, z, and padding to align to 16-byte boundaries.
        double timestamp;  // Offset: 16, datatype: double (8 bytes)
        float intensity;   // Offset: 24, datatype: float (4 bytes)
        uint8_t ring;     // Offset: 28, datatype: uint8_t (1 byte)
        uint8_t returnNum; // Offset: 29, datatype: uint8_t (1 byte)

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace hovermap_velodyne

POINT_CLOUD_REGISTER_POINT_STRUCT(hovermap_velodyne::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (double, timestamp, timestamp)
    (float, intensity, intensity)
    (std::uint8_t, ring, ring)
    (std::uint8_t, returnNum, returnNum)
)

typedef std::vector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> PoseVector;
// px py pz qx qy qz qw

typedef std::vector<Eigen::Matrix<double, 10, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 10, 1>>> ImuDataVector;
// gx gy gz ax ay az qx qy qz qw, first 3 for angular rate, second 3 for linear acceleration, last 4 for orientation
