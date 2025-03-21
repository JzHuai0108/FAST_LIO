#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fs = std::filesystem;

struct Pose {
    double time;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <pcd_folder> <pose_file>" << std::endl;
        return -1;
    }
    
    std::string pcdFolder = argv[1];
    std::string poseFile = argv[2];

    // --- Read poses from file ---
    std::map<double, Pose> poses;
    std::ifstream ifs(poseFile);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open pose file: " << poseFile << std::endl;
        return -1;
    }
    
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        double time, x, y, z, qx, qy, qz, qw;
        if (!(iss >> time >> x >> y >> z >> qx >> qy >> qz >> qw)) {
            std::cerr << "Error reading line: " << line << std::endl;
            continue;
        }
        Pose pose;
        pose.time = time;
        pose.translation = Eigen::Vector3d(x, y, z);
        // Reorder quaternion components: Eigen expects (w, x, y, z)
        pose.rotation = Eigen::Quaterniond(qw, qx, qy, qz);
        poses[time] = pose;
    }
    ifs.close();

    // --- Create an aggregated point cloud ---
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr aggregatedCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    
    // --- Process each PCD file in the given folder ---
    for (const auto& entry : fs::directory_iterator(pcdFolder)) {
        if (entry.path().extension() == ".pcd") {
            std::string filePath = entry.path().string();
            std::string filename = entry.path().filename().string();
            
            // Assume the file name (without extension) is the timestamp
            std::string time_str = entry.path().stem().string();
            double timestamp;
            try {
                timestamp = std::stod(time_str);
            } catch (const std::exception& e) {
                std::cerr << "Failed to parse timestamp from filename: " << filename << std::endl;
                continue;
            }
            
            // Find the corresponding pose (exact match is assumed)
            if (poses.find(timestamp) == poses.end()) {
                std::cerr << "No pose found for timestamp: " << timestamp << std::endl;
                continue;
            }
            Pose pose = poses[timestamp];
            
            // Load the point cloud
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
            if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(filePath, *cloud) < 0) {
                std::cerr << "Failed to load " << filePath << std::endl;
                continue;
            }
            
            // Create a transformation matrix from the pose
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3,3>(0,0) = pose.rotation.toRotationMatrix().cast<float>();
            transform.block<3,1>(0,3) = pose.translation.cast<float>();
            
            // Transform the point cloud from lidar frame to world frame
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::transformPointCloudWithNormals(*cloud, *transformedCloud, transform);
            
            // Append the transformed cloud to the aggregated cloud
            *aggregatedCloud += *transformedCloud;
        }
    }
    
    // --- Save the aggregated point cloud in binary format ---
    std::string outputFilename = "aggregated_cloud.pcd";
    if (pcl::io::savePCDFileBinary(outputFilename, *aggregatedCloud) < 0) {
        std::cerr << "Failed to save aggregated point cloud." << std::endl;
        return -1;
    }
    
    std::cout << "Aggregated point cloud saved to " << outputFilename << std::endl;
    return 0;
}
