#include "lidar_localizer.h"

#include <sstream>
#include <boost/algorithm/string.hpp>

// a kdtree has to be a global variable because of multithreading, see
// https://github.com/hku-mars/ikd-Tree/issues/8
KD_TREE<PointType> prior_map;
LidarLocalizer *localizer_ptr = nullptr;
const float MOV_THRESHOLD = 1.5f;
extern float DET_RANGE;
extern int NUM_MAX_ITERATIONS;

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

std::string joinPath(const std::string& dir, const std::string& file) {
    if (!dir.empty() && (dir.back() == '/' || dir.back() == '\\')) {
        return dir + file;
    }
    return dir + "/" + file;
}


bool load_initial_lidar_pose(const std::string &init_lidar_pose_file, V3D &world_t_lidar,
                             M3D &world_R_lidar, V3D &world_v_lidar, std::string &timestr) {
    if (!init_lidar_pose_file.empty()) {
        std::ifstream in(init_lidar_pose_file);
        if (in.is_open()) {
            double val[12];
            for (int i = 0; i < 12; ++i) {
                in >> val[i];
            }
            double vel[3] = {0};
            try {
                for (int i = 0; i < 3; ++i) {
                    in >> vel[i];
                }
            } catch (...) {
                std::cerr << "Warn: No velocity information in " << init_lidar_pose_file << std::endl;
            }
            in >> timestr;
            in.close();
            world_t_lidar = V3D(val[3], val[7], val[11]);
            world_R_lidar << val[0], val[1], val[2],
                             val[4], val[5], val[6],
                             val[8], val[9], val[10];
            world_v_lidar = V3D(vel[0], vel[1], vel[2]);
            std::cout << "Start point in TLS: " << ", world_t_lidar:"
                      << world_t_lidar.transpose() << "\nworld_R_lidar\n"
                      << world_R_lidar << "\nworld_v_lidar:"
                      << world_v_lidar.transpose() << "\ttime: " << timestr << std::endl;
            return true;
        } else {
            std::cerr << "Cannot open file: " << init_lidar_pose_file << std::endl;
            return false;
        }
    }
    return false;
}

void add_pose_noise(V3D &pos, M3D &rot, double pos_noise, double rot_noise) {
    V3D pos_noise_vec = pos_noise * V3D::Random();
    pos += pos_noise_vec;
    V3D rot_noise_vec = rot_noise * V3D::Random();
    rot = rot * SO3::exp(rot_noise_vec).matrix();
    std::cout << "Pose noise " << pos_noise << ", rot noise " << rot_noise << "\n";
    std::cout << "Position noise " << pos_noise_vec.transpose()
              << ", rot noise vec " << rot_noise_vec.transpose() << std::endl;
}

size_t load_tls_project_poses(const std::string &tls_project_dir, TlsPositionVector &TLS_positions) {
    Eigen::Matrix<double, 5, 1> position_id;
    std::string centerfile = tls_project_dir + "/centers.txt";
    int projectid = 0;
    if (tls_project_dir.find("project1") != std::string::npos) {
        projectid = 1;
    } else if (tls_project_dir.find("project2") != std::string::npos) {
        projectid = 2;
    } else {
        std::cout << "TLS project id not found in the directory name" << std::endl;
        return 0;
    }
    std::ifstream stream(centerfile.c_str());
    if (!stream.is_open()) {
        std::cout << centerfile << " doesn't exist" << std::endl;
        return 0;
    }
    std::string line;
    TLS_positions.reserve(TLS_positions.size() + 65);
    while (getline(stream, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        int scanid;
        ss >> scanid >> position_id[0] >> position_id[1] >> position_id[2];
        position_id[3] = projectid;
        position_id[4] = scanid;
        TLS_positions.push_back(position_id);
    }
    std::cout << "TLS_positions, 0: " << TLS_positions.front().transpose()
              << ", " << TLS_positions.size() - 1 << ": " << TLS_positions.back().transpose() << std::endl;
    return TLS_positions.size();
}

ros::Time parseTimeStr(const std::string &time_str) {
    if (time_str.empty()) {
        return ros::Time(0);
    }
    if (time_str == "0") {
        return ros::Time(0);
    }
    size_t pos = time_str.find('.');
    int secs = std::stoi(time_str.substr(0, pos));
    size_t nseclen = time_str.size() - pos - 1;
    if (nseclen < 9) {
        return ros::Time(secs, std::stoi(time_str.substr(pos + 1)) * std::pow(10, 9 - nseclen));
    } else {
        return ros::Time(secs, std::stoi(time_str.substr(pos + 1, 9)));
    }
}


int load_close_tls_scans(const TlsPositionVector &TLS_positions, const Eigen::Vector3d &cur_position,
                         const std::string &tls_dir,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr TLS_submap, int prev_nearest_scan_idx) {
    // find the nearest position in the TLS map
    int nearest_index = -1;
    double min_distance = std::numeric_limits<double>::max();
    for (int i = 0; i < TLS_positions.size(); i++)
    {
        V3D dist_vec = cur_position - TLS_positions[i].head(3);
        dist_vec.z() = 0;
        double distance = dist_vec.norm();
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_index = i;
        }
    }

    if (nearest_index == prev_nearest_scan_idx) { // already loaded, no need to load again.
        return nearest_index;
    }

    std::string map_filename;
    int target_project_id = int(TLS_positions[nearest_index][3]);
    std::vector<int> loadedscans;
    loadedscans.reserve(3);
    for (int i = nearest_index - 1; i <= nearest_index + 1; i++){
        if (i < 0 || i >= TLS_positions.size()){
            continue;
        }
        int projectid = int(TLS_positions[i][3]);
        if (projectid != target_project_id)
            continue;
        int scanid = int(TLS_positions[i][4]);
        if (projectid == 1) {
            map_filename = tls_dir + "/project1/regis/" + std::to_string(scanid) + ".pcd";
        } else if (projectid == 2) {
            map_filename = tls_dir + "/project2/regis/" + std::to_string(scanid) + "_uniform.pcd";
        }
        loadedscans.push_back(scanid);
        pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPCDFile<pcl::PointXYZ>(map_filename, *map);
        *TLS_submap += *map;
    }
    std::cout << "Loaded " << loadedscans.size() << " new scans: ";
    for (size_t j = 0; j < loadedscans.size(); ++j) {
        std::cout << j << ":" << loadedscans[j] << " ";
    }
    std::cout << std::endl;
    return nearest_index;
}


inline void points_cache_collect()
{
    PointVector points_history;
    prior_map.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}


void h_meas_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    localizer_ptr->laserCloudOri->clear();
    localizer_ptr->corr_normvect->clear();
    double total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < localizer_ptr->feats_down_size; i++)
    {
        PointType &point_body  = localizer_ptr->feats_down_body->points[i];
        PointType &point_world = localizer_ptr->feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = localizer_ptr->Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            prior_map.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            localizer_ptr->point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!localizer_ptr->point_selected_surf[i]) continue;

        VF(4) pabcd;
        localizer_ptr->point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, float(localizer_ptr->est_plane_thresh)))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > localizer_ptr->icp_dist_thresh)
            {
                localizer_ptr->point_selected_surf[i] = true;
                localizer_ptr->normvec->points[i].x = pabcd(0);
                localizer_ptr->normvec->points[i].y = pabcd(1);
                localizer_ptr->normvec->points[i].z = pabcd(2);
                localizer_ptr->normvec->points[i].intensity = pd2;
                localizer_ptr->res_last[i] = abs(pd2);
            }
        }
    }

    localizer_ptr->effct_feat_num = 0;

    for (int i = 0; i < localizer_ptr->feats_down_size; i++)
    {
        if (localizer_ptr->point_selected_surf[i])
        {
            localizer_ptr->laserCloudOri->points[localizer_ptr->effct_feat_num] = localizer_ptr->feats_down_body->points[i];
            localizer_ptr->corr_normvect->points[localizer_ptr->effct_feat_num] = localizer_ptr->normvec->points[i];
            total_residual += localizer_ptr->res_last[i];
            localizer_ptr->effct_feat_num ++;
        }
    }

    if (localizer_ptr->effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("Localizer: No Effective Points! \n");
        return;
    }

    localizer_ptr->res_mean_last = total_residual / localizer_ptr->effct_feat_num;
    localizer_ptr->match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = Eigen::MatrixXd::Zero(localizer_ptr->effct_feat_num, 12); //23
    ekfom_data.h.resize(localizer_ptr->effct_feat_num);

    for (int i = 0; i < localizer_ptr->effct_feat_num; i++)
    {
        const PointType &laser_p  = localizer_ptr->laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = localizer_ptr->corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (localizer_ptr->extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    localizer_ptr->solve_time += omp_get_wtime() - solve_start_;
}

LidarLocalizer::LidarLocalizer():
    initialized_(false), path_publisher(nullptr),
    frame_map_publisher(nullptr),
    pose_publisher(nullptr) {
    p_imu.reset(new ImuProcess());
    localizer_ptr = this;
}

LidarLocalizer::~LidarLocalizer() {
    statestream_.close();
    std::cout << "Loc to prior map results saved to " << statefile_ << std::endl;
    localizer_ptr = nullptr;
    path_publisher = nullptr;
    frame_map_publisher = nullptr;
    pose_publisher = nullptr;
}

void LidarLocalizer::initializeImu(
    const M3D &Lidar_R_wrt_IMU, const V3D &Lidar_T_wrt_IMU,
    double gyr_cov, double acc_cov, double b_gyr_cov, double b_acc_cov, double G_m_s2) {
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->G_m_s2 = G_m_s2;
}

void LidarLocalizer::initialize(const std::string &init_lidar_pose_file,
        const std::string &tls_dir, const std::string &tls_ref_traj_files,
        const M3D &Lidar_R_wrt_IMU, const V3D &Lidar_T_wrt_IMU, const double G_m_s2,
        double filter_size_surf, double filter_size_map, double tls_dist_thresh,
        const std::string &logdir) {
    logdir_ = logdir;
    statefile_ = joinPath(logdir_, "loc_states.txt");
    statestream_.open(statefile_, std::fstream::out);
    std::string header = "#time(sec),M_p_L_x,M_p_L_y,M_p_L_z,M_q_L_x,M_q_L_y,M_q_L_z,M_q_L_w";
    header += ",M_v_L_x,M_v_L_y,M_v_L_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,M_G_x,M_G_y,M_G_z";
    header += ",sigma_M_p_I_x,sigma_M_p_I_y,sigma_M_p_I_z,sigma_M_q_I_x,sigma_M_q_I_y,sigma_M_q_I_z";
    header += ",sigma_M_v_I_x,sigma_M_v_I_y,sigma_M_v_I_z,sigma_M_G_x,sigma_M_G_y\n";
    statestream_ << header;

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    filter_size_map_ = filter_size_map;
    downSizeFilterSurf_.setRadiusSearch(filter_size_surf);
    downSizeFilterMap_.setRadiusSearch(filter_size_map);

    std::cout << "init_lidar_pose_file: " << init_lidar_pose_file << std::endl;
    V3D init_w_t_lidar;
    M3D init_w_R_lidar;
    V3D init_w_v_lidar;
    std::string timestr;
    load_initial_lidar_pose(init_lidar_pose_file, init_w_t_lidar, init_w_R_lidar, init_w_v_lidar, timestr);

    V3D init_world_t_imu_vec(Zero3d);
    M3D init_world_R_imu(Eye3d);
    V3D init_world_v_imu_vec(Zero3d);
    init_world_R_imu = init_w_R_lidar * Lidar_R_wrt_IMU.transpose();
    init_world_t_imu_vec = init_w_t_lidar - init_world_R_imu * Lidar_T_wrt_IMU;
    init_world_v_imu_vec = init_w_v_lidar;

    tls_dir_ = tls_dir;
    std::cout << "TLS dir: " << tls_dir << std::endl;
    std::string tls_project_dir = tls_dir + "/project1/regis";
    load_tls_project_poses(tls_project_dir, tls_position_ids_);
    tls_project_dir = tls_dir + "/project2/regis";
    load_tls_project_poses(tls_project_dir, tls_position_ids_);

    prior_map.set_downsample_param(filter_size_map);
    tls_dist_thresh_ = tls_dist_thresh;
    if (!tls_ref_traj_files.empty()) {
        std::vector<std::string> filenames;
        boost::split(filenames, tls_ref_traj_files, boost::is_any_of(";"));
        ROS_INFO("TLS reference trajectory files: %s", tls_ref_traj_files.c_str());
        ROS_INFO("TLS distance threshold: %.2f", tls_dist_thresh_);
        Trajectory ref_traj;
        load_ref_traj(filenames, ref_traj);
        dist_checkup_.initialize(ref_traj, tls_dist_thresh_);
    } else {
        ROS_WARN("No TLS reference trajectory files provided, TLS checkup disabled.");
    }

    double epsi[23] = {0.001};
    std::fill(epsi, epsi+23, 0.001);
    kf_.init_dyn_share(get_f, df_dx, df_dw, h_meas_model, NUM_MAX_ITERATIONS, epsi);

    // initialize state
    state_ikfom init_state = kf_.get_x();
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    init_state.pos = init_world_t_imu_vec;
    init_state.rot = SO3(init_world_R_imu);
    init_state.vel = init_world_v_imu_vec;
    init_state.grav = S2(0, 0, -G_m_s2);
    init_state.bg  = V3D(0, 0, 0);
    init_state.ba  = V3D(0, 0, 0);
    ROS_INFO_STREAM("Localizer init pose with external translation " << init_world_t_imu_vec.transpose()
                     << " and rotation\n" << init_world_R_imu << "\nand gravity "
                     << init_state.grav.get_vect().transpose() << " at " << timestr);
    kf_.change_x(init_state);
    statestamp_ = parseTimeStr(timestr);
    p_imu->first_lidar_time = statestamp_.toSec();

    initialized_ = true;
}

void LidarLocalizer::push(PointCloudXYZI::ConstPtr unskewed_scan, const double stamp,
    const state_ikfom &state) {
    if (!initialized_)
        return;
    ros::Time tstamp;
    tstamp.fromSec(stamp);
    bool ready = propagate(tstamp, state, unskewed_scan);

    loadCloseTlsScans(kf_.get_x());

    // update the pose and covariance by matching to the prior map
    if (ready && inbound_) {
        PointCloudXYZI::Ptr accum_scan(new PointCloudXYZI());
        assembleScan(accum_scan);
        updateState(accum_scan, stamp);
    }

    publish(stamp);

    saveState();
}

void LidarLocalizer::propagateCov(const MeasureGroup &measurements) {
    PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
    bool init_status_bef = p_imu->needInit();
    auto state = kf_.get_x();
    p_imu->Process(measurements, kf_, feats_undistort);
    bool init_status_aft = p_imu->needInit();
    if (init_status_bef) { // restore our initial kf state when the IMU is to be initialized, and we will keep the initial cov from the IMU.
        kf_.change_x(state);
    }
}

bool LidarLocalizer::propagate(const ros::Time &stamp,
    const state_ikfom &odom_state,
    PointCloudXYZI::ConstPtr unskewed_scan) {
    auto map_state = kf_.get_x();

    ros::Duration half_interval(0.05);
    if (odom_states_.size() == 0) {
        if (stamp + half_interval < statestamp_) {
            ROS_INFO_STREAM("The map state at " << statestamp_ << " is later than the odometry state at " << stamp);
            return false;
        }

        ros::Duration d = stamp - statestamp_;
        if (d > ros::Duration(0.2)) {
            ROS_WARN_STREAM("The map state at " << statestamp_ << " is much earlier than odometry state at " << stamp << ", but we have to init anyway!");
        } else {
            ROS_INFO_STREAM("The map state at " << statestamp_ << " is aligned to the odometry state at " << stamp << " with diff " << d);
        }

        Eigen::Affine3d O_T_I(Eigen::Translation3d(odom_state.pos) * odom_state.rot);
        Eigen::Affine3d M_T_I(Eigen::Translation3d(map_state.pos) * map_state.rot);
        O_T_I_kf_ = O_T_I;
        M_T_I_kf_ = M_T_I;
        M_T_O_ = M_T_I_kf_ * O_T_I_kf_.inverse();
    } else {
        Eigen::Affine3d O_T_I(Eigen::Translation3d(odom_state.pos) * odom_state.rot);
        Eigen::Affine3d Ip_T_I = O_T_I_kf_.inverse() * O_T_I;
        Eigen::Affine3d M_T_I = M_T_I_kf_ * Ip_T_I;
        map_state.pos = M_T_I.translation();
        map_state.rot = M_T_I.rotation();

        double dd =Ip_T_I.translation().norm();
        double da = std::fabs(Eigen::AngleAxisd(Ip_T_I.rotation()).angle());
        if ( dd > 0.5 || da > M_PI / 18.0) {
            should_swap_kf_ = true;
        }
    }
    map_state.vel = M_T_O_.rotation() * odom_state.vel;
    map_state.bg = odom_state.bg;
    map_state.ba = odom_state.ba;
    map_state.grav = M_T_O_.rotation() * odom_state.grav;

    kf_.change_x(map_state);
    statestamp_ = stamp;

    const auto &state_point = kf_.get_x();
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

    // update the sliding window of frames
    scans_.push_back(unskewed_scan);
    odom_states_.push_back(odom_state);
    odom_stamps_.push_back(stamp);
    if (scans_.size() > accum_window_) {
        scans_.pop_front();
        odom_states_.pop_front();
        odom_stamps_.pop_front();
    }
    return true;
}

void LidarLocalizer::assembleScan(PointCloudXYZI::Ptr accum_scan) {
    // given a seq of scans and their odom poses
    // convert all scan points to the end of the last scan
    // and create a new scan
    // new scan's points are in the lidar frame at the last scan timestamp
    // also filter the points by voxel downsampling
    size_t total_pts = 0;
    // Note scans_, odom_states_, and odom_stamps_ are deques.
    for (const auto& scan : scans_) {
        total_pts += scan->size();
    }
    accum_scan->resize(total_pts);

    Eigen::Affine3d I_T_L = Eigen::Translation3d(odom_states_.back().offset_T_L_I) * odom_states_.back().offset_R_L_I;
    Eigen::Affine3d O_T_Ir = Eigen::Translation3d(odom_states_.back().pos) * odom_states_.back().rot;
    Eigen::Affine3d O_T_Lr = O_T_Ir * I_T_L;
    Eigen::Affine3d Lr_T_O = O_T_Lr.inverse();

    size_t pts = 0;
    for (size_t i = 0; i < scans_.size(); ++i) {
        Eigen::Affine3d O_T_Ii = Eigen::Translation3d(odom_states_[i].pos) * odom_states_[i].rot;
        Eigen::Affine3d Lr_T_Li = Lr_T_O * O_T_Ii * I_T_L;

        for (const auto& p : scans_[i]->points) {
            Eigen::Vector3d transformed_point = Lr_T_Li.rotation() * Eigen::Vector3d(p.x, p.y, p.z) + Lr_T_Li.translation();
            accum_scan->points[pts].x = transformed_point.x();
            accum_scan->points[pts].y = transformed_point.y();
            accum_scan->points[pts].z = transformed_point.z();
            ++pts;
        }
    }
    // Create a temporary point cloud for filtering
    PointCloudXYZI::Ptr temp_scan(new PointCloudXYZI);
    downSizeFilterSurf_.setInputCloud(accum_scan);
    downSizeFilterSurf_.filter(*temp_scan);
    *accum_scan = *temp_scan;
    // ROS_INFO_STREAM("Accum scan points reduced from " << pts << " to " << accum_scan->size());
}

void LidarLocalizer::loadCloseTlsScans(const state_ikfom &state) {
    // given the current pose, find close TLS scans and load them to the current kdtree
    // also filter the points
    prior_map.set_downsample_param(filter_size_map_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr TLS_submap(new pcl::PointCloud<pcl::PointXYZ>());
    int prev_nearest_idx = nearest_scan_idx;
    nearest_scan_idx = load_close_tls_scans(tls_position_ids_, state.pos, tls_dir_, TLS_submap, nearest_scan_idx);
    if (TLS_submap->size()) {
        updateKdTree(TLS_submap, tls_position_ids_, prev_nearest_idx, nearest_scan_idx);
    }
}

void LidarLocalizer::updateState(PointCloudXYZI::Ptr feats_undistort, double lidar_end_time) {
    // use the IEKF scheme and the new scan to update the kf
    match_time = 0;
    solve_time = 0;
    solve_const_H_time = 0;

    if (feats_undistort->empty() || (feats_undistort == NULL))
    {
        ROS_WARN("Localizer: No point, skip this unskewed scan!");
        return;
    }

    /*** Segment the map in lidar FOV ***/
    lasermap_fov_segment();

    /*** downsample the feature points in a scan ***/
    downSizeFilterSurf_.setInputCloud(feats_undistort);
    downSizeFilterSurf_.filter(*feats_down_body);
    feats_down_size = feats_down_body->points.size();

//    int featsFromMapNum = prior_map.validnum();
//    int kdtree_size_st = prior_map.size();

    // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

    /*** ICP and iterated Kalman filter update ***/
    if (feats_down_size < 5)
    {
        ROS_WARN("Localizer: No point, skip this downsampled scan!");
        return;
    }

    normvec->resize(feats_down_size);
    feats_down_world->resize(feats_down_size);

//    V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
//    fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
//             <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

//    if (show_submap) {// If you need to see map point
//        // But this can drastically slow down the program.
//        PointVector ().swap(ikdtree.PCL_Storage);
//        ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
//        featsFromMap->clear();
//        featsFromMap->points = ikdtree.PCL_Storage;
//        publish_map(pubLaserCloudMap);
//    }
    std::vector<std::vector<int>>  pointSearchInd_surf;
    pointSearchInd_surf.resize(feats_down_size);
    Nearest_Points.resize(feats_down_size);
    int  rematch_num = 0;
    bool nearest_search_en = true; //

    /*** iterated state estimation ***/
//    double t_update_start = omp_get_wtime();
    double solve_H_time = 0;
    kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
    const auto &state_point = kf_.get_x();
    V3D euler_cur = SO3ToEuler(state_point.rot);
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    geometry_msgs::Quaternion geoQuat;
    geoQuat.x = state_point.rot.coeffs()[0];
    geoQuat.y = state_point.rot.coeffs()[1];
    geoQuat.z = state_point.rot.coeffs()[2];
    geoQuat.w = state_point.rot.coeffs()[3];

    Eigen::Vector4d timed_position;
    timed_position[0] = lidar_end_time;
    timed_position.segment(1, 3) = pos_lid;
    inbound_ = dist_checkup_.check(timed_position);
    if (!inbound_) {
        ROS_WARN("The lidar pose is out of the TLS trajectory, abort the localizer.");
    }

    if (should_swap_kf_) {
        O_T_I_kf_ = Eigen::Affine3d(Eigen::Translation3d(odom_states_.back().pos) * odom_states_.back().rot);
        const auto& state = kf_.get_x();
        M_T_I_kf_ = Eigen::Affine3d(Eigen::Translation3d(state.pos) * state.rot);
        M_T_O_ = M_T_I_kf_ * O_T_I_kf_.inverse();
        should_swap_kf_ = false;
    }
}

void LidarLocalizer::saveState() const {
    // save the state and covariance to the output log file
    // the pose velocity, gravity, etc
    // the covariance for pose and velocity

    const auto &state_point = kf_.get_x();
    Pose3d B_T_S;
    B_T_S.R = state_point.offset_R_L_I;
    B_T_S.p = state_point.offset_T_L_I;
    V3D w_t_s = state_point.rot * B_T_S.p + state_point.pos;
    Eigen::Quaterniond w_r_s = state_point.rot * B_T_S.R;
    const std::string delim = ",";
    statestream_ << statestamp_ << delim;
    statestream_ << std::fixed << std::setprecision(6)
                 << w_t_s(0) << delim << w_t_s(1) << delim << w_t_s(2) << delim;
    statestream_ << std::fixed << std::setprecision(9)
                 << w_r_s.coeffs()[0] << delim << w_r_s.coeffs()[1] << delim
                 << w_r_s.coeffs()[2] << delim << w_r_s.coeffs()[3] << delim;
    statestream_ << std::fixed << std::setprecision(6)
                 << state_point.vel(0) << delim << state_point.vel(1) << delim << state_point.vel(2) << delim  // Velocity
                 << state_point.bg(0) << delim << state_point.bg(1) << delim << state_point.bg(2) << delim    // Bias_g
                 << state_point.ba(0) << delim << state_point.ba(1) << delim << state_point.ba(2) << delim    // Bias_a
                 << state_point.grav[0] << delim << state_point.grav[1] << delim << state_point.grav[2];    // Gravity in world
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P = kf_.get_P();

    Eigen::Matrix3d pcov = P.topLeftCorner<3, 3>();
    Eigen::Vector3d psigma = pcov.diagonal().cwiseSqrt();  // Standard deviations for position
    Eigen::Matrix3d qcov = P.block<3, 3>(3, 3);
    Eigen::Vector3d qsigma = qcov.diagonal().cwiseSqrt();  // Standard deviations for orientation
    Eigen::Matrix3d vcov = P.block<3, 3>(12, 12);
    Eigen::Vector3d vsigma = vcov.diagonal().cwiseSqrt();  // Standard deviations for velocity
    Eigen::Matrix2d gcov = P.block<2, 2>(21, 21);
    Eigen::Vector2d gsigma = gcov.diagonal().cwiseSqrt();  // Standard deviations for gravity

    statestream_ << delim << std::fixed << std::setprecision(9)
                 << psigma[0] << delim << psigma[1] << delim << psigma[2] << delim  // Position sigma
                 << qsigma[0] << delim << qsigma[1] << delim << qsigma[2] << delim  // Orientation sigma
                 << vsigma[0] << delim << vsigma[1] << delim << vsigma[2] << delim  // Velocity sigma
                 << gsigma[0] << delim << gsigma[1] << "\n";                    // Gravity sigma
}


void LidarLocalizer::updateKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr TLS_submap, const TlsPositionVector &TLS_positions,
                  int prev_nearest_scan_idx, int nearest_index) {
    downSizeFilterMap_.setInputCloud(TLS_submap);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ds(new pcl::PointCloud<pcl::PointXYZ>());
    downSizeFilterMap_.filter(*map_ds);
    PointVector PointToAdd;
    PointToAdd.reserve(map_ds->points.size());
    for (int i = 0; i < map_ds->points.size(); i++) {
        pcl::PointXYZINormal point;
        point.x = map_ds->points[i].x;
        point.y = map_ds->points[i].y;
        point.z = map_ds->points[i].z;
        PointToAdd.push_back(point);
    }
    prior_map.Build(PointToAdd);
    if (prev_nearest_scan_idx >= 0) {
        ROS_INFO(("New nearest id %d, project id %d, old nearest id %d, project id %d, "
                  "%zu points downsampled from %zu."),
                 nearest_index, (int)TLS_positions[nearest_index][3], prev_nearest_scan_idx,
                 (int)TLS_positions[prev_nearest_scan_idx][3], PointToAdd.size(), TLS_submap->points.size());
    } else {
        ROS_INFO(("New nearest id %d, project id %d, "
                  "%zu points downsampled from %zu."),
                 nearest_index, (int)TLS_positions[nearest_index][3], PointToAdd.size(), TLS_submap->points.size());
    }
}


void pointBodyToWorld(
    const state_ikfom &state_point,
    const Eigen::Matrix<float, 3, 1> &pi,
    Eigen::Matrix<float, 3, 1> &po)
{
    V3D p_lidar(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_lidar + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void LidarLocalizer::lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(kf_.get_x(), XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = std::max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;
    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = prior_map.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}


/// @{ ros stuff

template<typename T>
void set_posestamp(const state_ikfom &state_point, T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);

    out.pose.orientation.x = state_point.rot.coeffs()[0];
    out.pose.orientation.y = state_point.rot.coeffs()[1];
    out.pose.orientation.z = state_point.rot.coeffs()[2];
    out.pose.orientation.w = state_point.rot.coeffs()[3];
}

void LidarLocalizer::publish(double lidar_end_time) {
    publish_map_frame(lidar_end_time);
    publish_path(lidar_end_time);
    publish_pose(lidar_end_time);
    publish_map(lidar_end_time);
}

void LidarLocalizer::publish_path(double lidar_end_time)
{
    geometry_msgs::PoseStamped msg_body_pose;
    set_posestamp(kf_.get_x(), msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "map";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path_.header.stamp    = ros::Time().fromSec(lidar_end_time);
        path_.header.frame_id ="map";
        path_.poses.push_back(msg_body_pose);
        path_publisher->publish(path_);
    }
}

void RGBpointBodyToWorld(const state_ikfom &state_point,
    PointType const * const pi, PointType * const po)
{
    V3D p_lidar(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_lidar + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LidarLocalizer::publish_map_frame(double lidar_end_time)
{
    if(frame_map_publisher)
    {
        PointCloudXYZI::Ptr laserCloudWorld;
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();
        laserCloudWorld.reset(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(kf_.get_x(), &laserCloudFullRes->points[i],
                                &laserCloudWorld->points[i]);
        }
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "map";
        frame_map_publisher->publish(laserCloudmsg);
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    //    if (pcd_save_en)
    //    {
    //        int size = feats_undistort->points.size();
    //        PointCloudXYZI::Ptr laserCloudWorld( \
    //            new PointCloudXYZI(size, 1));
    //        if (pcd_save_interval == 1) {
    //            for (int i = 0; i < size; i++) {
    //                // save the pointcloud in the local lidar frame.
    //                Matrix<double, 3, 1> temp;
    //                temp(0) = feats_undistort->points[i].x;
    //                temp(1) = feats_undistort->points[i].y;
    //                temp(2) = feats_undistort->points[i].z;
    //                // temp = state_point.offset_R_L_I*temp + state_point.offset_T_L_I;
    //                laserCloudWorld->points[i].x = temp(0);
    //                laserCloudWorld->points[i].y = temp(1);
    //                laserCloudWorld->points[i].z = temp(2);
    //                laserCloudWorld->points[i].intensity = feats_undistort->points[i].intensity;
    //            }
    //            *pcl_wait_save = *laserCloudWorld;
    //        } else {
    //            for (int i = 0; i < size; i++) {
    //                RGBpointBodyToWorld(&feats_undistort->points[i], \
    //                                                                     &laserCloudWorld->points[i]);
    //            }
    //            *pcl_wait_save += *laserCloudWorld;
    //        }

    //        static int scan_wait_num = 0;
    //        scan_wait_num ++;
    //        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
    //        {
    //            pcd_index ++;

    //            std::stringstream ss;
    //            std::string all_points_dir = state_log_dir + "/PCD/";
    //            ss << std::fixed << std::setprecision(9) << lidar_end_time << ".pcd";
    //            std::string fn = all_points_dir + ss.str();

    //            pcl::PCDWriter pcd_writer;
    //            pcd_writer.writeBinary(fn, *pcl_wait_save);
    //            pcl_wait_save->clear();
    //            scan_wait_num = 0;
    //        }
    //    }
}

void LidarLocalizer::publish_map(double lidar_end_time) {
    if (map_publisher) {
        // If you need to see map point
        // But this can drastically slow down the program.
        PointVector().swap(prior_map.PCL_Storage);
        prior_map.flatten(prior_map.Root_Node, prior_map.PCL_Storage, NOT_RECORD);
        PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
        featsFromMap->points = prior_map.PCL_Storage;
        sensor_msgs::PointCloud2 laserCloudMap;
        pcl::toROSMsg(*featsFromMap, laserCloudMap);
        laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudMap.header.frame_id = "map";
        map_publisher->publish(laserCloudMap);
    }
}

void LidarLocalizer::publish_pose(double lidar_end_time) {
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "map";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(kf_.get_x(), odomAftMapped.pose);
    auto P = kf_.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }
    pose_publisher->publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;

    // Set translation
    const Eigen::Vector3d& translation = M_T_O_.translation();
    transform.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));

    // Set rotation
    Eigen::Quaterniond eigen_quat(M_T_O_.rotation());
    q.setW(eigen_quat.w());
    q.setX(eigen_quat.x());
    q.setY(eigen_quat.y());
    q.setZ(eigen_quat.z());
    transform.setRotation(q);
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "map", "camera_init"));
}

/// @}
