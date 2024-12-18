#include "lidar_localizer.h"

#include <sstream>
#include <boost/algorithm/string.hpp>


KD_TREE<PointType> prior_map;

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


LidarLocalizer::LidarLocalizer(): initialized_(false) { }

LidarLocalizer::~LidarLocalizer() {
    statestream_.close();
}

void LidarLocalizer::initialize(const std::string &init_lidar_pose_file,
        const std::string &tls_dir, const std::string &tls_ref_traj_files,
        const M3D &Lidar_R_wrt_IMU, const V3D &Lidar_T_wrt_IMU,
        double filter_size_surf, double filter_size_map, double G_m_s2,
        const std::string &logdir) {
    logdir_ = logdir;
    statefile_ = joinPath(logdir_, "loc_states.txt");
    statestream_.open(statefile_, std::fstream::out);
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

    std::cout << "TLS dir: " << tls_dir << std::endl;
    std::string tls_project_dir = tls_dir + "/project1/regis";
    load_tls_project_poses(tls_project_dir, tls_position_ids_);
    tls_project_dir = tls_dir + "/project2/regis";
    load_tls_project_poses(tls_project_dir, tls_position_ids_);

    prior_map.set_downsample_param(filter_size_map);
    if (!tls_ref_traj_files.empty()) {
        std::vector<std::string> filenames;
        boost::split(filenames, tls_ref_traj_files, boost::is_any_of(";"));
        ROS_INFO("TLS reference trajectory files: %s", tls_ref_traj_files.c_str());
        ROS_INFO("TLS distance threshold: %.2f", tls_dist_thresh);
        Trajectory ref_traj;
        load_ref_traj(filenames, ref_traj);
        dist_checkup_.initialize(ref_traj, tls_dist_thresh);
    } else {
        ROS_WARN("No TLS reference trajectory files provided, TLS checkup disabled.");
    }

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
    ROS_INFO_STREAM("Init pose with external translation " << init_world_t_imu_vec.transpose() << " and rotation\n"
            << init_world_R_imu << "\nand gravity " << init_state.grav.get_vect().transpose());
    kf_.change_x(init_state);
    statestamp_ = parseTimeStr(timestr);

    // initialize cov
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_.get_P();
    init_P.setIdentity();
    init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
    init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
    init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
    init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
    init_P(21,21) = init_P(22,22) = 0.00001; 
    kf_.change_P(init_P);

    initialized_ = true;
}

void LidarLocalizer::push(PointCloudXYZI::ConstPtr unskewed_scan, const double stamp,
    const state_ikfom &state, 
    const esekfom::esekf<state_ikfom, 12, input_ikfom>::cov &cov) {
    if (!initialized_)
        return;
    // update the sliding window of frames
    scans_.push_back(unskewed_scan);
    odom_states_.push_back(state);
    ros::Time tstamp;
    tstamp.fromSec(stamp);
    odom_stamps_.push_back(tstamp);
    if (scans_.size() > accum_window_) {
        scans_.pop_front();
        odom_states_.pop_front();
        odom_stamps_.pop_front();
    }

    PointCloudXYZI::Ptr accum_scan;
    assembleScan(accum_scan);

    bool ready = propagate(tstamp, state, cov);

    loadCloseTlsScans(kf_.get_x());

    // update the pose and covariance by matching to the prior map
    if (ready)
        updateState(accum_scan);

    // save the pose and covariance 
    saveState();
}

bool LidarLocalizer::propagate(const ros::Time &stamp, const state_ikfom &odom_state, 
    const esekfom::esekf<state_ikfom, 12, input_ikfom>::cov &cov) {
    const state_ikfom &last_odom_state = odom_states_.back();
    // if the time diff between odom time and state time less than a threshold {
    //     warn("lidar localizer has not alive")
    //     return false;
    // }
    // compute the relative pose change between the last and current state, apply it to the kf state
    // compute the relative transform between the last odom pose and the last kf pose, apply it to the current velocity, to reset the kf velocity
    // update the biases of the kf state
    // update the gravity of the kf state by using the relative transform

    // update the kf cov to that of the odom
    return true;
}

void LidarLocalizer::assembleScan(PointCloudXYZI::Ptr accum_scan) {
    // given a seq of scans and their odom poses
    // convert all scan points to the end of the last scan
    // and create a new scan
    // new scan's points are in the lidar frame at the last scan timestamp
    // also filter the points by voxel downsampling
}

void LidarLocalizer::loadCloseTlsScans(const state_ikfom &state) {
    // given the current pose, find close TLS scans and load them to the current kdtree
    // also filter the points
}

void LidarLocalizer::updateState(PointCloudXYZI::Ptr accum_scan) {
    // use the IEKF scheme and the new scan to update the kf
}

void LidarLocalizer::saveState() const {
    // save the state and covariance to the output log file
    // the pose velocity, gravity, etc
    // the covariance for pose and velocity
    statestream_ << "New state\n";
}

