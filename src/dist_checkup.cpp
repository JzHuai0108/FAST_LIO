#include "dist_checkup.h"

size_t load_states(const std::string &path_to_traj, Trajectory &traj) {
    std::ifstream stream(path_to_traj);
    if (!stream.is_open()) {
        std::cerr << "Failed to open file: " << path_to_traj << std::endl;
        return 0;
    }
    traj.reserve(10000);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        double timestamp;
        iss >> timestamp;
        Eigen::Matrix<double, 4, 1> timed_position;
        timed_position(0) = timestamp;
        for (size_t i = 1; i < 4; ++i) {
            iss >> timed_position(i);
        }
        traj.push_back(timed_position);
    }
    stream.close();
    return traj.size();
}

void load_ref_traj(const std::vector<std::string> &tls_ref_traj_files, Trajectory &ref_traj) {
    for (const auto &tls_ref_traj_file : tls_ref_traj_files) {
        Trajectory traj;
        size_t numposes = load_states(tls_ref_traj_file, traj);
        std::cout << "Loaded " << numposes << " poses from " << tls_ref_traj_file << std::endl;
        ref_traj.reserve(ref_traj.size() + numposes);
        ref_traj.insert(ref_traj.end(), traj.begin(), traj.end());
    }
}

DistCheckup::DistCheckup() : dist_threshold2(0.0) {}

DistCheckup::DistCheckup(const Trajectory &_ref_traj, double _dist_threshold) {
    initialize(_ref_traj, _dist_threshold);
}

void DistCheckup::initialize(const Trajectory &_ref_traj, double _dist_threshold) {
    ref_traj = _ref_traj;
    dist_threshold2 = _dist_threshold * _dist_threshold;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = ref_traj.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < ref_traj.size(); ++i) {
        cloud->points[i].x = ref_traj[i](1);
        cloud->points[i].y = ref_traj[i](2);
        cloud->points[i].z = ref_traj[i](3);
    }
    kdtree.setInputCloud(cloud);
}

bool DistCheckup::check(const Eigen::Matrix<double, 4, 1> &timed_position) {
    if (ref_traj.empty()) {
        return true;
    }
    pcl::PointXYZ searchPoint;
    searchPoint.x = timed_position(1);
    searchPoint.y = timed_position(2);
    searchPoint.z = timed_position(3);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        if (pointNKNSquaredDistance[0] > dist_threshold2) {
            std::cout << "Distance2 to reference trajectory is too large: " 
                      << pointNKNSquaredDistance[0] << " > " << dist_threshold2 << std::endl;
            return false;
        } else {
            return true;
        }
    } else {
        std::cerr << "Failed to find nearest point in reference trajectory" << std::endl;
        return false;
    }
}

std::vector<bool> check_dist_to_ref_traj(
        const Trajectory &traj, 
        const Trajectory &ref_traj,
        double dist_threshold) {
    DistCheckup dist_checkup(ref_traj, dist_threshold);
    std::vector<bool> status;
    status.reserve(traj.size());
    for (const auto &timed_position : traj) {
        status.push_back(dist_checkup.check(timed_position));
    }
    return status;
}
