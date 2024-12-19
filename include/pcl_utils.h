#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>


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

void locToTlsByGicp(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidar_frame,
                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr tls_submap,
                    Eigen::Matrix4f &tls_T_lidar,  int num_gicp_iter, double &meansquaredist,
                    size_t &nummatches, bool &converged);

void pointXyziToPointXyz(pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out);

#endif // PCL_UTILS_H
