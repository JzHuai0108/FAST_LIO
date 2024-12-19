#include "pcl_utils.h"

void locToTlsByGicp(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidar_frame,
                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr tls_submap,
                    Eigen::Matrix4f &tls_T_lidar,  int num_gicp_iter, double &meansquaredist,
                    size_t &nummatches, bool &converged) {
    GeneralizedIterativeClosestPointExposed<pcl::PointXYZ, pcl::PointXYZ> reg;
    reg.setInputSource(lidar_frame);
    reg.setInputTarget(tls_submap);
    // use default parameters or set them yourself, for example:
    reg.setMaximumIterations(num_gicp_iter);
    // reg.setTransformationEpsilon(...);
    // reg.setRotationEpsilon(...);
    // reg.setCorrespondenceRandomness(...);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    reg.align(*output, tls_T_lidar);
    meansquaredist = reg.getFitnessScore(0.15);
    nummatches = reg.nr_;
    converged = reg.hasConverged();
    tls_T_lidar = reg.getFinalTransformation();
}

void pointXyziToPointXyz(pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out) {
    out->resize(in->size());
    for (int i = 0; i < in->size(); i++) {
        out->points[i].x = in->points[i].x;
        out->points[i].y = in->points[i].y;
        out->points[i].z = in->points[i].z;
    }
}
