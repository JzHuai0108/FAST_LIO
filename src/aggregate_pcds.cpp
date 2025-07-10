#include "aggregate_pcds.hpp"

namespace fs = std::filesystem;

std::map<double, Pose> loadTumPoses(const std::string &poseFile) {
    std::map<double, Pose> poses;
    std::ifstream ifs(poseFile);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open pose file: " << poseFile << std::endl;
        return poses;
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
    return poses;
}

void aggregatePointCloudsWithPose(const std::vector<std::pair<std::string, std::string>> &pcdFolderAndPoseFileList,
        const std::string outputdir, double trim_start_secs, double trim_end_secs) {
    Pose ref_W_T_L;
    std::map<double, Pose> agg_poses;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr aggregatedCloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    for (size_t i = 0; i < pcdFolderAndPoseFileList.size(); ++i) {
        std::string pcdFolder = pcdFolderAndPoseFileList[i].first;
        std::string poseFile = pcdFolderAndPoseFileList[i].second;
        std::map<double, Pose> poses = loadTumPoses(poseFile);
        if (poses.empty()) {
            std::cerr << "Failed to load poses from: " << poseFile << std::endl;
            continue;
        }
        std::cout << "Loaded " << poses.size() << " poses from " << poseFile << std::endl;

        Pose W0_T_W1;
        if (i > 0) {
            if (poses.find(ref_W_T_L.time) != poses.end()) {
                Pose match_W_T_L = poses[ref_W_T_L.time];
                W0_T_W1.rotation = ref_W_T_L.rotation * match_W_T_L.rotation.inverse();
                W0_T_W1.translation = ref_W_T_L.translation - W0_T_W1.rotation * match_W_T_L.translation;
            } else {
                std::cerr << "No ref pose found for timestamp: " << std::fixed << std::setprecision(9) <<
                     ref_W_T_L.time << " in " << i << " pair " << std::endl;
            }
        }

        // transform the poses and put into agg_poses, but skip those in the last trim part
        for (const auto& [timestamp, pose] : poses) {
            if (poses.begin()->first + trim_start_secs > timestamp || timestamp + trim_end_secs > poses.rbegin()->first) {
                std::cout << "trimming " << std::fixed << std::setprecision(9) << timestamp << " of pair " << i << std::endl;
                continue;
            }
            if (timestamp > ref_W_T_L.time) {
                // ref_W_T_L = W0_T_W1 * pose
                ref_W_T_L.time = timestamp;
                ref_W_T_L.rotation = W0_T_W1.rotation * pose.rotation;
                ref_W_T_L.translation = W0_T_W1.rotation * pose.translation + W0_T_W1.translation;
            }
            Pose pose2 = pose;
            if (i > 0) {
                pose2.rotation = W0_T_W1.rotation * pose.rotation;
                pose2.translation = W0_T_W1.rotation * pose.translation + W0_T_W1.translation;
            }
            agg_poses[timestamp] = pose2;
        }

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
                    std::cerr << "No pose found for timestamp: " << std::fixed << std::setprecision(9) << timestamp << std::endl;
                    continue;
                }
                if (poses.begin()->first + trim_start_secs > timestamp || timestamp + trim_end_secs > poses.rbegin()->first) {
                    continue;
                }

                Pose pose = poses[timestamp];
                
                // Load the point cloud
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
                if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(filePath, *cloud) < 0) {
                    std::cerr << "Failed to load " << filePath << std::endl;
                    continue;
                }
                
                // transform the W0 frame, W0_T_W1 * pose
                if (i > 0) {
                    pose.rotation = W0_T_W1.rotation * pose.rotation;
                    pose.translation = W0_T_W1.rotation * pose.translation + W0_T_W1.translation;
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
    }


    // save all poses
    std::string outputPoseFile = outputdir + "/aggregated_poses.txt";
    std::ofstream ofs(outputPoseFile);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open " << outputPoseFile << " for writing." << std::endl;
        return;
    }
    for (const auto& [timestamp, pose] : agg_poses) {
        ofs << std::fixed << std::setprecision(9) << timestamp << " ";
        ofs << pose.translation.x() << " " << pose.translation.y() << " " << pose.translation.z() << " ";
        ofs << pose.rotation.x() << " " << pose.rotation.y() << " " << pose.rotation.z() << " " << pose.rotation.w() << std::endl;
    }
    ofs.close();

    std::string outputFilename = outputdir + "/aggregated_cloud.pcd";
    if (pcl::io::savePCDFileBinary(outputFilename, *aggregatedCloud) < 0) {
        std::cerr << "Failed to save aggregated point cloud." << std::endl;
        return;
    }

    std::cout << "Aggregated point cloud saved to " << outputFilename << std::endl;
}
