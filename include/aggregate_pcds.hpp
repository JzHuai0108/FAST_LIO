#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <filesystem>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct Pose {
    double time;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;

    Pose() : time(0), translation(Eigen::Vector3d::Zero()), rotation(Eigen::Quaterniond::Identity()) {}
};

std::map<double, Pose> loadTumPoses(const std::string &poseFile);

void aggregatePointCloudsWithPose(const std::vector<std::pair<std::string, std::string>> &pcdFolderAndPoseFileList,
        const std::string outputdir, double trim_start_secs, double trim_end_secs);

static bool hasSuffix(const std::string& str, const std::string& suffix) {
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool extractAndCompensateImu(const std::string& ros1_bag, const std::string& lio_states_txt, 
                             const std::string& imu_topic, const std::string& csv_path,
                             double gyro_scale, double acc_scale);

bool extractAndConvertImu(const std::string& ros1_bag, const std::string& lio_states_txt, 
                          const std::string& imu_topic, const Eigen::Isometry3d& B_T_L,
                          const std::string& csv_path, double gyro_scale, double acc_scale);

