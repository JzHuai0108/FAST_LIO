#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>

typedef std::vector<Eigen::Matrix<double, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 4, 1>>> Trajectory;

class DistCheckup {
// Check if a point is close to the trajectory for generating the TLS map
private:
    Trajectory ref_traj;
    double dist_threshold2;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

public:
    DistCheckup();

    DistCheckup(const Trajectory &ref_traj, double dist_threshold);

    void initialize(const Trajectory &ref_traj, double dist_threshold);
    // whether the point is close to the trajectory, true if close, false otherwise
    bool check(const Eigen::Matrix<double, 4, 1> &timed_position);
};

size_t load_states(const std::string &path_to_traj, Trajectory &traj);

void load_ref_traj(const std::vector<std::string> &tls_ref_traj_files, Trajectory &ref_traj);

std::vector<bool> check_dist_to_ref_traj(const Trajectory &traj, const Trajectory &ref_traj, double dist_threshold);