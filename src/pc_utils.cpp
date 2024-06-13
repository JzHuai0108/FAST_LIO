#include "pc_utils.h"

#include <pcl/io/pcd_io.h>

#include <iomanip>
#include <sstream>

void dump_tls_traj_to_log(double lidar_end_time, const Eigen::Matrix4d &tls_T_lidar, const state_ikfom &state_point, 
        std::ofstream &stream) {
    stream << std::fixed << std::setprecision(9) << lidar_end_time << " ";
    V3D tls_p_lidar = tls_T_lidar.block<3, 1>(0, 3);
    Eigen::Quaterniond tls_q_lidar(tls_T_lidar.block<3, 3>(0, 0));
    Eigen::Quaterniond tls_q_world = tls_q_lidar * state_point.offset_R_L_I.inverse() * state_point.rot.inverse();
    V3D tls_v_body = tls_q_world * state_point.vel;
    V3D tls_gravity = tls_q_world * state_point.grav;
    stream << std::fixed << std::setprecision(6) << tls_p_lidar(0) << " " << tls_p_lidar(1) << " " << tls_p_lidar(2) << " ";
    stream << std::fixed << std::setprecision(9) << tls_q_world.coeffs()[0] << " " << tls_q_world.coeffs()[1] << " " << tls_q_world.coeffs()[2] << " " << tls_q_world.coeffs()[3] << " ";
    stream << std::fixed << std::setprecision(6) << tls_v_body(0) << " " << tls_v_body(1) << " " << tls_v_body(2) << " ";
    stream << std::fixed << std::setprecision(6) << state_point.bg(0) << " " << state_point.bg(1) << " " << state_point.bg(2) << " ";
    stream << std::fixed << std::setprecision(6) << state_point.ba(0) << " " << state_point.ba(1) << " " << state_point.ba(2) << " ";
    stream << std::fixed << std::setprecision(6) << tls_gravity(0) << " " << tls_gravity(1) << " " << tls_gravity(2) << " ";
    stream << std::fixed << std::setprecision(6) << state_point.offset_T_L_I(0) << " " << state_point.offset_T_L_I(1) << " " << state_point.offset_T_L_I(2) << " ";
    stream << std::fixed << std::setprecision(9) << state_point.offset_R_L_I.coeffs()[0] << " " << state_point.offset_R_L_I.coeffs()[1] << " " << state_point.offset_R_L_I.coeffs()[2] << " " << state_point.offset_R_L_I.coeffs()[3] << " ";
    stream << "\n";
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
    string centerfile = tls_project_dir + "/centers.txt";
    int projectid = 0;
    if (tls_project_dir.find("project1") != string::npos) {
        projectid = 1;
    } else if (tls_project_dir.find("project2") != string::npos) {
        projectid = 2;
    } else {
        std::cout << "TLS project id not found in the directory name" << std::endl;
        return 0;
    }
    ifstream stream(centerfile.c_str());
    if (!stream.is_open()) {
        std::cout << centerfile << " doesn't exist" << std::endl;
        return 0;
    }
    string line;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr loadTlsScans(
    const std::string &tls_dir,
    const TlsPositionVector &tls_position_ids,
    const std::vector<int> &nearest_scan_ids) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tls_scans(new pcl::PointCloud<pcl::PointXYZ>());
    std::string map_filename;
    for (size_t i = 0; i < nearest_scan_ids.size(); i++) {
        int idx = nearest_scan_ids[i];
        int projectid = int(tls_position_ids[idx][3]);
        int scanid = int(tls_position_ids[idx][4]);
        if (projectid == 1) {
            map_filename = tls_dir + "/project1/regis/" + std::to_string(scanid) + ".pcd";
        } else if (projectid == 2) {
            map_filename = tls_dir + "/project2/regis/" + std::to_string(scanid) + "_uniform.pcd";
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPCDFile<pcl::PointXYZ>(map_filename, *map);
        *tls_scans += *map;
    }
    std::cout << "Loaded " << nearest_scan_ids.size() << " new scans: ";
    for (size_t j = 0; j < nearest_scan_ids.size(); ++j) {
        std::cout << j << ":" << nearest_scan_ids[j] << " ";
    }
    return tls_scans;
}

int load_close_tls_scans(const TlsPositionVector &TLS_positions, 
        const std::string &tls_dir,
        const Eigen::Vector3d &lidar_position,
        int prev_nearest_scan_idx, int maxscans,
        pcl::PointCloud<pcl::PointXYZ>::Ptr tls_scans) {
    // find the nearest position in the TLS map
    int nearest_index = -1;
    double min_distance = std::numeric_limits<double>::max();
    for (int i = 0; i < TLS_positions.size(); i++)
    {
        V3D dist_vec = lidar_position - TLS_positions[i].head(3);
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

    size_t count = 0;
    int target_project_id = int(TLS_positions[nearest_index][3]);
    std::vector<int> loadedscans;
    loadedscans.reserve(3);

    int halfscans = maxscans / 2;
    for (int i = nearest_index - halfscans; i <= nearest_index + halfscans; i++) {
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
        *tls_scans += *map;
        count++;
    }
    std::cout << "Loaded " << loadedscans.size() << " new scans: ";
    for (size_t j = 0; j < loadedscans.size(); ++j) {
        std::cout << j << ":" << loadedscans[j] << " ";
    }
    return nearest_index;
}