#include "aggregate_pcds.hpp"

#include "fast_lio/common.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

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


static std::pair<int,int> bracketStates(const cba::StampedStateVector& states, const ros::Time& t) {
  if (states.empty()) return {-1,-1};
  if (t <= states.front().time) return {0,0};
  if (t >= states.back().time)  return {int(states.size()-1), int(states.size()-1)};
  int lo = 0, hi = int(states.size()) - 1;
  while (hi - lo > 1) {
    int mid = (lo + hi) / 2;
    if (states[mid].time <= t) lo = mid; else hi = mid;
  }
  return {lo, lo+1};
}

static Eigen::Vector3d interpGyroBiasB(const cba::StampedStateVector& states, const ros::Time& t) {
  auto [i0, i1] = bracketStates(states, t);
  if (i0 < 0) return Eigen::Vector3d::Zero();
  if (i0 == i1) return states[i0].bg;
  const double t0 = states[i0].time.toSec(), t1 = states[i1].time.toSec();
  const double tau = (t1 > t0) ? ((t.toSec() - t0) / (t1 - t0)) : 0.0;
  return (1.0 - tau) * states[i0].bg + tau * states[i1].bg;
}

static Eigen::Vector3d interpAccBiasB(const cba::StampedStateVector& states, const ros::Time& t) {
  auto [i0, i1] = bracketStates(states, t);
  if (i0 < 0) return Eigen::Vector3d::Zero();
  if (i0 == i1) return states[i0].ba;
  const double t0 = states[i0].time.toSec(), t1 = states[i1].time.toSec();
  const double tau = (t1 > t0) ? ((t.toSec() - t0) / (t1 - t0)) : 0.0;
  return (1.0 - tau) * states[i0].ba + tau * states[i1].ba;
}

static inline Eigen::Vector3d tangential(const Eigen::Vector3d& alpha, const Eigen::Vector3d& r) {
  return alpha.cross(r);
}
static inline Eigen::Vector3d centripetal(const Eigen::Vector3d& omega, const Eigen::Vector3d& r) {
  return omega.cross(omega.cross(r));
}

struct AlphaEstimator {
  bool has_prev = false;
  ros::Time t_prev;
  Eigen::Vector3d w_prev = Eigen::Vector3d::Zero();
  Eigen::Vector3d update(const ros::Time& t, const Eigen::Vector3d& w_L) {
    if (!has_prev) { has_prev = true; t_prev = t; w_prev = w_L; return Eigen::Vector3d::Zero(); }
    const double dt = (t.toSec() - t_prev.toSec());
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
    if (dt > 0) a = (w_L - w_prev) / dt;
    t_prev = t; w_prev = w_L;
    return a;
  }
};

bool extractAndConvertImu(const std::string& ros1_bag, const std::string& lio_states_txt, 
                          const std::string& imu_topic, const Eigen::Isometry3d& B_T_L,
                          const std::string& csv_path, double gyro_scale, double acc_scale) {
  // first extract all imu messages from rosbag, these msgs are omega_{WB,s}^B and a_{WB,s}^B
  // second compute the omega_{WL,s}^L = L_R_B * (omega_{WB,s}^B - bg), the suberscript s means sensed.
  // third compute the specific force acceleration in the L frame by
  // a_{WL,s}^L = L_R_B * a_{WB,s}^B - (\Omega_{WL}^{L, 2} L_p_B + \dot{\Omega}_{WL}^L L_p_B)
  // In the below interpolation for \Omega_{WL}^{L}, we will omega_{WB}^{B} = omega_{WB,s}^{B} - b_g, then 
  // omega_{WL}^{L} = R_LB * omega_{WB}^{B}
  // Load states (for bias interpolation)
  cba::StampedStateVector states;
  if (!loadStates(lio_states_txt, &states)) {
    std::cerr << "No states loaded from " << lio_states_txt << "\n";
    return false;
  }
  std::sort(states.begin(), states.end(),
            [](const cba::StampedState& a, const cba::StampedState& b){ return a.time < b.time; });

  // Extrinsics: x_B = R_BL x_L + t_BL
  const Eigen::Matrix3d R_BL = B_T_L.rotation();
  const Eigen::Matrix3d R_LB = R_BL.transpose();
  const Eigen::Vector3d t_BL = B_T_L.translation();
  const Eigen::Vector3d r_L  = - R_LB * t_BL;   // ^L p_B

  rosbag::Bag bag;
  try { bag.open(ros1_bag, rosbag::bagmode::Read); }
  catch (const rosbag::BagException& e) {
    std::cerr << "Failed to open bag: " << e.what() << "\n";
    return false;
  }
  rosbag::View view(bag, rosbag::TopicQuery({imu_topic}));

  std::ofstream ofs(csv_path);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open csv for write: " << csv_path << "\n";
    bag.close();
    return false;
  }
  ofs << std::fixed << std::setprecision(9);
  ofs << "# sec.nsec,gx,gy,gz,ax,ay,az (L-frame, bg/ba-comp, lever-arm corrected)\n";

  AlphaEstimator alpha_est;
  size_t n_written = 0;
  size_t n_large_accel = 0;

  for (const rosbag::MessageInstance& m : view) {
    auto imu = m.instantiate<sensor_msgs::Imu>();
    if (!imu) continue;

    const ros::Time t = imu->header.stamp;

    // Raw in B (sensor outputs)
    Eigen::Vector3d w_B_meas(imu->angular_velocity.x,
                             imu->angular_velocity.y,
                             imu->angular_velocity.z);
    Eigen::Vector3d a_B_meas(imu->linear_acceleration.x,
                             imu->linear_acceleration.y,
                             imu->linear_acceleration.z);

    // Apply optional global scales to measurements ONLY (biases are in SI already)
    w_B_meas *= gyro_scale;
    a_B_meas *= acc_scale;

    // Interpolate biases in B and subtract
    const Eigen::Vector3d bg_B = interpGyroBiasB(states, t);
    const Eigen::Vector3d ba_B = interpAccBiasB(states, t);
    const Eigen::Vector3d w_B  = w_B_meas - bg_B;
    const Eigen::Vector3d f_B  = a_B_meas - ba_B;   // specific force in B after bias removal

    // Rotate to L
    const Eigen::Vector3d w_L     = R_LB * w_B;
    const Eigen::Vector3d f_L_rot = R_LB * f_B;

    // Angular acceleration in L by finite differencing
    const Eigen::Vector3d alpha_L = alpha_est.update(t, w_L);

    // Lever-arm correction: f_L = R_LB f_B - α×r - ω×(ω×r)
    const Eigen::Vector3d f_L = f_L_rot
                              - tangential(alpha_L, r_L)
                              - centripetal(w_L, r_L);

    if (f_L.lpNorm<Eigen::Infinity>() > 40) {
      std::cerr << "Warn: large acceleration " << t.sec << "." << std::setw(9) << std::setfill('0') << t.nsec
        << ", w_L " << std::setprecision(9) << w_L.x() << "," << w_L.y() << "," << w_L.z() << ", f_L "
        << f_L.x() << "," << f_L.y() << "," << f_L.z() << ", alpha_L " << alpha_L.transpose()
        << "\n\tw_L " << w_L.transpose() << ", f_L_rot " << f_L_rot.transpose()
        << ", tangential " << tangential(alpha_L, r_L).transpose()
        << ", centripetal " << centripetal(w_L, r_L).transpose() << "\n\tbg_B " << bg_B.transpose() 
        << ", ba_B " << ba_B.transpose() << ", w_B "<< w_B.transpose() << ", f_B " << f_B.transpose() << std::endl;
      n_large_accel++;
    }
    // CSV line
    ofs << t.sec << "." << std::setw(9) << std::setfill('0') << t.nsec
        << "," << std::setprecision(9)
        << w_L.x() << "," << w_L.y() << "," << w_L.z() << ","
        << f_L.x() << "," << f_L.y() << "," << f_L.z() << "\n";
    ++n_written;
  }

  ofs.close();
  bag.close();
  std::cout << "Wrote " << n_written << " rows to " << csv_path << "\n";
  if (n_large_accel) {
    std::cout << "Warn: detected " << n_large_accel
          << (n_large_accel == 1 ? " large-acceleration event. " : " large-acceleration events. ")
          << "Possible cause: clustered IMU timestamps (tiny/irregular dt)."
          << std::endl;
  }
  return (n_written > 0);
}


// --- main function: bias removal only, output in B frame ---
bool extractAndCompensateImu(const std::string& ros1_bagfile,
                             const std::string& lio_states_txt,
                             const std::string& imu_topic,
                             const std::string& csv_path,
                             double gyro_scale,
                             double acc_scale)
{
  // 1) Load states for bias interpolation
  cba::StampedStateVector states;
  if (!loadStates(lio_states_txt, &states)) {
    ROS_ERROR_STREAM("Failed to load states from " << lio_states_txt
                     << ". Will proceed with zero biases.");
  } else {
    std::sort(states.begin(), states.end(),
              [](const cba::StampedState& a, const cba::StampedState& b){ return a.time < b.time; });
  }

  // 2) Open bag/view
  rosbag::Bag bag;
  try { bag.open(ros1_bagfile, rosbag::bagmode::Read); }
  catch (const rosbag::BagException& e) {
    ROS_ERROR_STREAM("Open bag failed: " << e.what());
    return false;
  }
  rosbag::View view(bag, rosbag::TopicQuery({imu_topic}));

  // 3) CSV
  std::ofstream ofs(csv_path);
  if (!ofs.is_open()) {
    ROS_ERROR_STREAM("Open CSV failed: " << csv_path);
    bag.close();
    return false;
  }
  ofs << std::fixed;
  ofs << "# time(gps sec.nsec), gx, gy, gz, ax, ay, az  (B-frame, bias-compensated)\n";

  // 4) Iterate IMU
  size_t n = 0;
  for (const rosbag::MessageInstance& mi : view) {
    auto imu = mi.instantiate<sensor_msgs::Imu>();
    if (!imu) continue;
    const ros::Time t = imu->header.stamp;

    // Measured (sensor outputs) in B
    Eigen::Vector3d w_meas(imu->angular_velocity.x,
                           imu->angular_velocity.y,
                           imu->angular_velocity.z);
    Eigen::Vector3d a_meas(imu->linear_acceleration.x,
                           imu->linear_acceleration.y,
                           imu->linear_acceleration.z);

    // Optional scalar calibration factors (apply to measurements, not biases)
    w_meas *= gyro_scale;
    a_meas *= acc_scale;

    // Interpolate biases (in B) and subtract
    const Eigen::Vector3d bg_B = interpGyroBiasB(states, t);
    const Eigen::Vector3d ba_B = interpAccBiasB(states, t);

    const Eigen::Vector3d w_B = w_meas - bg_B;   // rad/s
    const Eigen::Vector3d f_B = a_meas - ba_B;   // m/s^2 (specific force if driver publishes that)

    // Write time as sec.nsec with 9 digits
    ofs << t.sec << "." << std::setw(9) << std::setfill('0') << t.nsec
        << std::setfill(' ') << std::setprecision(9)
        << "," << w_B.x() << "," << w_B.y() << "," << w_B.z()
        << "," << f_B.x() << "," << f_B.y() << "," << f_B.z() << "\n";
    ++n;
  }

  ofs.close();
  bag.close();
  ROS_INFO_STREAM("extractAndCompensateImu: wrote " << n << " rows to " << csv_path);
  return n > 0;
}
