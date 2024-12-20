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
#include "IMU_Processing.hpp"
#include <ikd-Tree/ikd_Tree.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>

#include <nav_msgs/Path.h>

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

int load_close_tls_scans(const TlsPositionVector &TLS_positions, const Eigen::Vector3d &cur_position,
                         const std::string &tls_dir,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr TLS_submap, int prev_nearest_scan_idx);


class LidarLocalizer {
public:
    // Localize lidar scans to a prior map, using odometry poses, covariance, and unskewed (accumulated) scans.
    LidarLocalizer();

    virtual ~LidarLocalizer();

    void initializeImu(
        const M3D &Lidar_R_wrt_IMU, const V3D &Lidar_T_wrt_IMU,
        double gyr_cov, double acc_cov, double b_gyr_cov, double b_acc_cov, double G_m_s2);

    void initialize(const std::string &init_lidar_pose_file,
        const std::string &tls_dir, const std::string &tls_ref_traj_files,
        const M3D &Lidar_R_wrt_IMU, const V3D &Lidar_T_wrt_IMU, double G_m_s2,
        double filter_size_surf, double filter_size_map, double tls_dist_thresh,
        const std::string &statefilename);

    void setPublishers(ros::Publisher *path, ros::Publisher *frame_map, ros::Publisher *pose, ros::Publisher *map) {
        path_publisher = path;
        frame_map_publisher = frame_map;
        pose_publisher = pose;
        map_publisher = map;
    }

    void propagateCov(const MeasureGroup &measurements);

    void push(PointCloudXYZI::ConstPtr unskewed_scan, const double stamp, const state_ikfom &state);

    bool shouldAbort() const { return !inbound_; }

    void publish_path(double lidar_end_time);
    void publish_map_frame(double lidar_end_time);
    void publish_pose(double lidar_end_time);
    void publish_map(double lidar_end_time);
    void publish(double lidar_end_time);

private:
    LidarLocalizer(const LidarLocalizer&); // prevent copy as ofstream is a class member.
    LidarLocalizer& operator=(const LidarLocalizer&);

    bool propagate(const ros::Time &stamp,
                   const state_ikfom &state,
                   PointCloudXYZI::ConstPtr unskewed_scan);

    void assembleScan(PointCloudXYZI::Ptr accum_scan);

    void loadCloseTlsScans(const state_ikfom &state);

    void refinePoseByGICP(PointCloudXYZI::Ptr accum_scan, const double stamp);

    void updateState(PointCloudXYZI::Ptr accum_scan, const double stamp);

    void saveState() const;

    void updateKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr TLS_submap, const TlsPositionVector &TLS_positions,
                      int prev_nearest_scan_idx, int nearest_index);

    void lasermap_fov_segment();

private:
    bool initialized_;
    std::string statefile_;
    mutable std::ofstream statestream_;

    std::deque<PointCloudXYZI::ConstPtr> scans_; // downsampled unskewed lidar points in lidar frame at end frame time
    std::deque<state_ikfom> odom_states_; // states from the odometry, states and scans have the same length
    std::deque<ros::Time> odom_stamps_; // lidar frame end times.
    size_t accum_window_;

    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    ros::Time statestamp_;

    pcl::UniformSampling<pcl::PointXYZ> downSizeFilterMap_;
    pcl::UniformSampling<PointType> downSizeFilterSurf_;

    TlsPositionVector tls_position_ids_;
    std::string tls_dir_;
    double tls_dist_thresh_ = 8; // the max distance to the tls trajectory to abort the odometer.
    DistCheckup dist_checkup_;
    int nearest_scan_idx = -1; // scan idx within the tls_position_ids.
    double filter_size_map_ = 0;
    bool inbound_ = true;

    ros::Publisher *path_publisher;
    ros::Publisher *frame_map_publisher;
    ros::Publisher *pose_publisher;
    ros::Publisher *map_publisher;

    Eigen::Affine3d M_T_O_;
    Eigen::Affine3d O_T_I_kf_;
    Eigen::Affine3d M_T_I_kf_;
    bool should_swap_kf_ = false;

    std::shared_ptr<ImuProcess> p_imu; // for covariance propagation. The off diagonal terms in the covariance matrix is critical for full state observability.
    pcl::PointCloud<pcl::PointXYZ>::Ptr tls_submap_;
    int num_gicp_iter_;

public:
    BoxPointType LocalMap_Points;
    bool Localmap_Initialized = false;
    std::vector<BoxPointType> cub_needrm;
    std::vector<PointVector>  Nearest_Points;
    int kdtree_delete_counter = 0;
    double kdtree_delete_time = 0.0;
    V3F XAxisPoint_body{LIDAR_SP_LEN, 0.0, 0.0};
    V3F XAxisPoint_world{LIDAR_SP_LEN, 0.0, 0.0};
    vect3 pos_lid;
    double cube_len = 0;

    PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI()};  // points in the lidar frame.
    PointCloudXYZI::Ptr feats_down_world{new PointCloudXYZI()};
    PointCloudXYZI::Ptr normvec{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr laserCloudOri{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr corr_normvect{new PointCloudXYZI(100000, 1)};
    int feats_down_size = 0;
    bool   point_selected_surf[100000] = {0};
    float res_last[100000] = {0.0};
    double res_mean_last = 0.05;

    double icp_dist_thresh = 0.9; // The larger, the less points will be selected for ICP matching.
    double est_plane_thresh = 0.1; // The larger, the more surfels will be accepted as planes.
    int    effct_feat_num = 0;
    double match_time = 0, solve_time = 0, solve_const_H_time = 0;
    bool extrinsic_est_en = false;
    nav_msgs::Path path_;
};

#endif
