#pragma once

#include "common_lib.h"
#include "use-ikfom.hpp"

#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>


typedef std::vector<Eigen::Matrix<double, 5, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 5, 1>>> 
        TlsPositionVector; // [x, y, z, projectid, scanid]


void dump_tls_traj_to_log(double lidar_end_time, const Eigen::Matrix4d &tls_T_lidar, const state_ikfom &state_point, 
        std::ofstream &stream);

template <typename PointSource, typename PointTarget>
class GeneralizedIterativeClosestPointExposed : public pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget> {
public:
    size_t nr_;

    typedef pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget> BaseClass;
    typedef typename pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::PointCloudSource PointCloudSource;
    using BaseClass::input_;
    using BaseClass::final_transformation_;
    using BaseClass::tree_;
    double getFitnessScore(double max_range=std::numeric_limits< double >::max())
    {
        nr_ = 0;
        double fitness_score = 0.0;
        
        // Transform the input dataset using the final transformation
        PointCloudSource input_transformed;
        transformPointCloud(*input_, input_transformed, final_transformation_);
        
        pcl::Indices nn_indices(1);
        std::vector<float> nn_dists(1);
        
        // For each point in the source dataset
        int nr = 0;
        for (const auto& point : input_transformed) {
            if (!input_->is_dense && !pcl::isXYZFinite(point))
            continue;
            // Find its nearest neighbor in the target
            tree_->nearestKSearch(point, 1, nn_indices, nn_dists);
        
            // Deal with occlusions (incomplete targets)
            if (nn_dists[0] <= max_range) {
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
            }
        }
        nr_ = nr;
        if (nr > 0)
            return (fitness_score / nr);
        return 1e9; // (std::numeric_limits<double>::max());
    }
};


void add_pose_noise(V3D &pos, M3D &rot, double pos_noise, double rot_noise);

size_t load_tls_project_poses(const std::string &tls_project_dir, TlsPositionVector &TLS_positions);

pcl::PointCloud<pcl::PointXYZ>::Ptr loadTlsScans(const TlsPositionVector &tls_position_ids,
    const std::vector<int> &nearest_scan_ids);

int load_close_tls_scans(const TlsPositionVector &TLS_positions, 
        const std::string &tls_dir,
        const Eigen::Vector3d &lidar_pos,
        int prev_nearest_scan_idx, int maxscans,
        pcl::PointCloud<pcl::PointXYZ>::Ptr tls_scans);