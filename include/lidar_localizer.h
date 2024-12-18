#ifndef LIDAR_LOCALIZER_H_
#define LIDAR_LOCALIZER_H_

#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "common_lib.h"
#include "dist_checkup.h"
#include "use-ikfom.hpp"
#include <ikd-Tree/ikd_Tree.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>

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

std::string OdomModeToString(int mode);

std::string joinPath(const std::string& dir, const std::string& file);

bool load_initial_lidar_pose(const std::string &init_lidar_pose_file, V3D &world_t_lidar,
                             M3D &world_R_lidar, V3D &world_v_lidar, std::string &time);

void add_pose_noise(V3D &pos, M3D &rot, double pos_noise, double rot_noise);

size_t load_tls_project_poses(const std::string &tls_project_dir, TlsPositionVector &TLS_positions);

ros::Time parseTimeStr(const std::string &time_str);

class LidarLocalizer {
public:
    // Localize lidar scans to a prior map, using odometry poses, covariance, and unskewed (accumulated) scans.
    LidarLocalizer();

    virtual ~LidarLocalizer();

    void initialize(const std::string &init_lidar_pose_file,
        const std::string &tls_dir, const std::string &tls_ref_traj_files,
        const M3D &Lidar_R_wrt_IMU, const V3D &Lidar_T_wrt_IMU,
        double filter_size_surf, double filter_size_map, double G_m_s2,
        const std::string &logdir);

    void push(PointCloudXYZI::ConstPtr unskewed_scan, const double stamp, const state_ikfom &state, 
        const esekfom::esekf<state_ikfom, 12, input_ikfom>::cov &cov);

private:
    bool propagate(const ros::Time &stamp, const state_ikfom &state,
        const esekfom::esekf<state_ikfom, 12, input_ikfom>::cov &cov);

    void assembleScan(PointCloudXYZI::Ptr accum_scan);

    void loadCloseTlsScans(const state_ikfom &state);

    void updateState(PointCloudXYZI::Ptr accum_scan);

    void saveState() const;
private:
    bool initialized_;
    std::string logdir_;
    std::string statefile_;
    mutable std::ofstream statestream_;

    std::deque<pcl::PointCloud<PointType>::ConstPtr> scans_;
    std::deque<state_ikfom> odom_states_; // states from the odometry, states and scans have the same length
    std::deque<ros::Time> odom_stamps_;
    size_t accum_window_ = 10;

    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    ros::Time statestamp_;
    KD_TREE<PointType> ikdtree_;

    pcl::UniformSampling<PointType> downSizeFilterMap_;
    pcl::UniformSampling<PointType> downSizeFilterSurf_;

    TlsPositionVector tls_position_ids_;
    double tls_dist_thresh = 8; // the max distance to the tls trajectory to abort the odometer.
    DistCheckup dist_checkup_;
    int nearest_scan_idx = -1; // scan idx within the tls_position_ids.
};

#endif