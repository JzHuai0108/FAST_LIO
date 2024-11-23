/**
 * rectify the points in ROS1 PointCloud2 messages of hovermap rosbags
 * by transforming these points to the stationary encoder frame.
 */

#include "hovermap.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/TimeReference.h>
#include <tf2_msgs/TFMessage.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp> // sudo apt-get install libboost-filesystem-dev libboost-system-dev

#include <algorithm>  // For std::lower_bound
#include <iostream>
#include <string>

bool is_directory(const std::string& path) {
    boost::filesystem::path p(path);
    return boost::filesystem::is_directory(p);
}

void create_directories(const std::string& path) {
    boost::filesystem::path dir(path);
    if (!boost::filesystem::exists(dir)) {
        if (!boost::filesystem::create_directories(dir)) {
            throw std::runtime_error("Error creating directory: " + path);
        }
    }
}

std::vector<std::string> search_bag_files(const std::string& dir) {
    std::vector<std::string> bagnames;
    boost::filesystem::path p(dir);

    if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p)) {
        for (const auto& entry : boost::filesystem::directory_iterator(p)) {
            if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ".bag") {
                bagnames.push_back(entry.path().string());
            }
        }
    }

    return bagnames;
}

void read_pps_times(const std::string& bagname, std::vector<ros::Time> *host_times,
        std::vector<ros::Time> *sensor_times, const std::string& topic="/encoder/pps_timestamp") {
    // get the encoder remote sensor times and local host time from /encoder/pps_timestamp topic
    // a message looks like
// header: 
//   seq: 55579
//   stamp: 
//     secs: 1724900044
//     nsecs:  57923836
//   frame_id: "system"
// time_ref: 
//   secs: 595
//   nsecs: 830108000
// source: "micro"
    rosbag::Bag bag;
    bag.open(bagname, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(topic));

    host_times->reserve(1000);
    sensor_times->reserve(1000);
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::TimeReference::ConstPtr msg = m.instantiate<sensor_msgs::TimeReference>();
        if (msg != nullptr) {
            host_times->push_back(m.getTime());
            sensor_times->push_back(msg->time_ref);
        }
    }

    bag.close();
}

void read_lidar_times(const std::string& bagname, std::vector<ros::Time> *host_times,
        std::vector<ros::Time> *sensor_times, const std::string& topic="/velodyne_points") {
    // get the header timestamp as host time and the first lidar point's timestamp (as sensor time)
    //  from lidar messages on topic /velodyne_points.
    rosbag::Bag bag;
    bag.open(bagname, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(topic));
    host_times->reserve(1000);
    sensor_times->reserve(1000);
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != nullptr) {
            pcl::PointCloud<hovermap_velodyne::Point> pl_orig;
            pcl::fromROSMsg(*msg, pl_orig);
            int plsize = pl_orig.size();

            host_times->push_back(msg->header.stamp);
            ros::Time sensor_time;
            sensor_time.fromSec(pl_orig[0].timestamp);
            sensor_times->push_back(sensor_time);
        }
    }

    bag.close();
}

void read_encoder_tf_msgs(const std::string fn, std::vector<ros::Time> *host_times, 
                          PoseVector *poses, std::string topic) {
// each tf2_msgs/TFMessage message has the following fields:
//     transforms: 
//   - 
//     header: 
//       seq: 0
//       stamp: 
//         secs: 1724900040
//         nsecs: 917898170
//       frame_id: "encoder"
//     child_frame_id: "encoder_rotating"
//     transform: 
//       translation: 
//         x: 0.0
//         y: 0.0
//         z: 0.0
//       rotation: 
//         x: 0.0
//         y: -0.0
//         z: -0.8104803702913226
//         w: 0.5857657973733535
    rosbag::Bag bag;
    bag.open(fn, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        tf2_msgs::TFMessage::ConstPtr msg = m.instantiate<tf2_msgs::TFMessage>();
        if (msg != nullptr) {
            for (const auto& transform : msg->transforms) {
                host_times->push_back(transform.header.stamp);

                Eigen::Matrix<double, 7, 1> pose;
                pose(0) = transform.transform.translation.x;
                pose(1) = transform.transform.translation.y;
                pose(2) = transform.transform.translation.z;
                pose(3) = transform.transform.rotation.x;
                pose(4) = transform.transform.rotation.y;
                pose(5) = transform.transform.rotation.z;
                pose(6) = transform.transform.rotation.w;
                poses->push_back(pose);
            }
        }
    }
    bag.close();
}


void read_imu_data(const std::string& fn, 
                   std::vector<ros::Time>& imu_host_times, 
                   ImuDataVector& imu_datalist, 
                   const std::string& topic) {
    rosbag::Bag bag;
    bag.open(fn, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(topic));
    imu_host_times.reserve(1000);
    imu_datalist.reserve(1000);
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg != nullptr) {
            // Store the timestamp of the IMU message
            imu_host_times.push_back(imu_msg->header.stamp);

            // Create an Eigen vector to store the IMU data
            Eigen::Matrix<double, 10, 1> imu_data;

            // Fill in the angular velocity (gx, gy, gz)
            imu_data(0) = imu_msg->angular_velocity.x;
            imu_data(1) = imu_msg->angular_velocity.y;
            imu_data(2) = imu_msg->angular_velocity.z;

            // Fill in the linear acceleration (ax, ay, az)
            imu_data(3) = imu_msg->linear_acceleration.x;
            imu_data(4) = imu_msg->linear_acceleration.y;
            imu_data(5) = imu_msg->linear_acceleration.z;

            // Fill in the orientation quaternion (qx, qy, qz, qw)
            imu_data(6) = imu_msg->orientation.x;
            imu_data(7) = imu_msg->orientation.y;
            imu_data(8) = imu_msg->orientation.z;
            imu_data(9) = imu_msg->orientation.w;

            // Store the data in the vector
            imu_datalist.push_back(imu_data);
        }
    }

    bag.close();
    std::cout << imu_datalist.size() <<" IMU data read from topic " << topic << " and stored in vector." << std::endl;
}

void save_times(const std::vector<ros::Time>& host_times, 
                const std::vector<ros::Time>& sensor_times, 
                const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Check if we need to write both host and sensor times or just host times
    if (!sensor_times.empty()) {
        if (host_times.size() != sensor_times.size()) {
            std::cerr << "Error: host_times and sensor_times vectors have different sizes." << std::endl;
            file.close();
            return;
        }

        // Write both host and sensor times with formatted nanoseconds
        for (size_t i = 0; i < host_times.size(); ++i) {
            file << host_times[i].sec << "." << std::setw(9) << std::setfill('0') << host_times[i].nsec << " "
                 << sensor_times[i].sec << "." << std::setw(9) << std::setfill('0') << sensor_times[i].nsec << std::endl;
        }
    } else {
        // Write only host times with formatted nanoseconds
        for (const auto& time : host_times) {
            file << time.sec << "." << std::setw(9) << std::setfill('0') << time.nsec << std::endl;
        }
    }

    file.close();
    std::cout << "Saved times to " << filename << std::endl;
}

template <typename VectorT>
void associate_data_to_pps(const std::vector<ros::Time>& data_host_times, 
                         const VectorT& datalist, 
                         const std::vector<ros::Time>& pps_host_times, 
                         const std::vector<ros::Time>& pps_sensor_times, 
                         std::vector<ros::Time>* assoc_sensor_times, 
                         VectorT* assoc_datalist,
                         const double tol_ms = 5.0) {
    // for each data host time, find its matching pps host time, with tol less than tol_ms
    // so we have a mapping from pps sensor time to data.
    // data_host_times.size() == datalist.size()
    // pps_host_times.size() == pps_sensor_times.size()
    // assoc_sensor_times.size() == assoc_datalist.size()

    size_t data_idx = 0;
    size_t start_idx = 0;
    const double tol_sec = tol_ms / 1000.0;  // Convert tolerance from ms to seconds
    for (const auto& data_time : data_host_times) {
        double closest_time_diff = tol_sec * 2;
        size_t closest_idx = std::numeric_limits<size_t>::max();

        // Search for a matching encoder host time within the given tolerance
        for (size_t i = start_idx; i < pps_host_times.size(); ++i) {
            double time_diff = (pps_host_times[i] - data_time).toSec();

            // Stop searching if the time difference exceeds 10 times the tolerance
            if (time_diff > 10 * tol_sec) {
                break;
            }

            // Check if the current time difference is within the tolerance
            double abs_diff = std::fabs(time_diff);
            if (abs_diff <= tol_sec) {
                if (abs_diff < closest_time_diff) {
                    closest_time_diff = abs_diff;
                    closest_idx = i;
                }
            }
        }

        // If a valid closest index was found, store the corresponding sensor time and pose
        if (closest_idx != std::numeric_limits<size_t>::max()) {
            assoc_sensor_times->push_back(pps_sensor_times[closest_idx]);
            assoc_datalist->push_back(datalist[data_idx]);

            // Update start index to avoid rechecking previous entries
            start_idx = closest_idx + 1;
        } else {
            std::cout << "Warn: fail to find pps host time for data at " << data_time << std::endl;
        }
        ++data_idx;
    }
}


void save_time_and_poses(const std::vector<ros::Time>& sensor_times, 
                         const PoseVector& poses, 
                         const std::string& filename) {
    // save time and poses, each line, time(sec) px py pz qx qy qz qw

    if (sensor_times.size() != poses.size()) {
        std::cerr << "Error: sensor_times and poses vectors must have the same size." << std::endl;
        return;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    for (size_t i = 0; i < sensor_times.size(); ++i) {
        const auto& time = sensor_times[i];
        const auto& pose = poses[i];

        file << time.sec << "." << std::setw(9) << std::setfill('0') << time.nsec << " "
             << std::fixed << std::setprecision(6)  // Set precision for translation
             << pose(0) << " " << pose(1) << " " << pose(2) << " "
             << std::setprecision(9)  // Set precision for rotation
             << pose(3) << " " << pose(4) << " " << pose(5) << " " << pose(6) << std::endl;
    }

    file.close();
    std::cout << "Saved times and poses to " << filename << std::endl;
}


void save_encoder_to_bag(const std::vector<ros::Time>& assoc_sensor_times, 
                         const PoseVector& assoc_encoder_poses, 
                         const std::string& outbagname, 
                         const std::string& topic) {
    // save encoder poses as TFMessage to outbagname
    // the frame_id: "encoder"
    //   child_frame_id: "encoder_rotating"
    if (assoc_sensor_times.size() != assoc_encoder_poses.size()) {
        std::cerr << "Error: assoc_sensor_times and assoc_encoder_poses vectors must have the same size." << std::endl;
        return;
    }

    rosbag::Bag bag;
    try {
        bag.open(outbagname, rosbag::bagmode::Write);

        for (size_t i = 0; i < assoc_sensor_times.size(); ++i) {
            // Create a TransformStamped message
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped.header.stamp = assoc_sensor_times[i];
            transform_stamped.header.frame_id = "encoder";
            transform_stamped.child_frame_id = "encoder_rotating";

            // Fill in the translation part (px, py, pz)
            transform_stamped.transform.translation.x = assoc_encoder_poses[i][0];
            transform_stamped.transform.translation.y = assoc_encoder_poses[i][1];
            transform_stamped.transform.translation.z = assoc_encoder_poses[i][2];

            // Fill in the rotation part (qx, qy, qz, qw)
            transform_stamped.transform.rotation.x = assoc_encoder_poses[i][3];
            transform_stamped.transform.rotation.y = assoc_encoder_poses[i][4];
            transform_stamped.transform.rotation.z = assoc_encoder_poses[i][5];
            transform_stamped.transform.rotation.w = assoc_encoder_poses[i][6];

            // Create a TFMessage and add the TransformStamped message to it
            tf2_msgs::TFMessage tf_message;
            tf_message.transforms.push_back(transform_stamped);

            // Write the message to the bag
            bag.write(topic, assoc_sensor_times[i], tf_message);
        }

        bag.close();
        std::cout << "Saved " << assoc_sensor_times.size() << " encoder poses to " << outbagname << std::endl;
    } catch (const rosbag::BagIOException& e) {
        std::cerr << "Error: Could not open or write to bag file " << outbagname << ": " << e.what() << std::endl;
    }
}


Eigen::Affine3d interpolate_pose(const std::vector<ros::Time>& times,
                                 const PoseVector& poses,
                                 const ros::Time& query_time) {
    // Use binary search to find the closest time index
    auto it = std::lower_bound(times.begin(), times.end(), query_time);

    // Handle edge cases where query_time is before the first time or after the last time
    if (it == times.begin()) {
        std::cout << "Warn: query_time " << query_time << " earlier than the first time " << times.front() << std::endl;
        return Eigen::Translation3d(poses.front().head<3>()) *
               Eigen::Quaterniond(poses.front()[6], poses.front()[3], poses.front()[4], poses.front()[5]);
    }
    if (it == times.end()) {
        std::cout << "Warn: query_time " << query_time << " later than the last time " << times.back() << std::endl;
        return Eigen::Translation3d(poses.back().head<3>()) *
               Eigen::Quaterniond(poses.back()[6], poses.back()[3], poses.back()[4], poses.back()[5]);
    }

    // Get the indices for interpolation
    size_t i = std::distance(times.begin(), it) - 1;  // Index of the lower bound
    size_t ip1 = i + 1;  // Next index

    // Perform linear interpolation for translation
    double alpha = (query_time - times[i]).toSec() / (times[ip1] - times[i]).toSec();
    Eigen::Vector3d translation_i = poses[i].head<3>();
    Eigen::Vector3d translation_ip1 = poses[ip1].head<3>();
    Eigen::Vector3d interpolated_translation = (1 - alpha) * translation_i + alpha * translation_ip1;

    // Perform spherical linear interpolation (slerp) for rotation
    Eigen::Quaterniond rotation_i(poses[i][6], poses[i][3], poses[i][4], poses[i][5]);
    Eigen::Quaterniond rotation_ip1(poses[ip1][6], poses[ip1][3], poses[ip1][4], poses[ip1][5]);
    Eigen::Quaterniond interpolated_rotation = rotation_i.slerp(alpha, rotation_ip1);

    // Combine the interpolated translation and rotation into a single transform
    Eigen::Affine3d interpolated_pose = Eigen::Translation3d(interpolated_translation) * interpolated_rotation;
    return interpolated_pose;
}

void get_lidar_extrinsics() {
    Eigen::Affine3d E_T_I = Eigen::Translation3d(-0.03587546555175705, 0.005730217182290136, 0.032125318431006494) *
                            Eigen::Quaterniond(0.005488590158788708, 0.5002131474472541, -0.0020782735504224405, 0.8658824188526003);
    std::cout << "E_T_I:\n";
    std::cout << E_T_I.matrix() << std::endl;
    Eigen::Matrix4d I_T_E = E_T_I.inverse().matrix();
    std::cout << "I_R_E:\n";
    std::cout << std::fixed << std::setprecision(9);
    std::cout << I_T_E(0, 0) << "," << I_T_E(0, 1) << "," << I_T_E(0, 2) << "," << std::endl;
    std::cout << I_T_E(1, 0) << "," << I_T_E(1, 1) << "," << I_T_E(1, 2) << "," << std::endl;
    std::cout << I_T_E(2, 0) << "," << I_T_E(2, 1) << "," << I_T_E(2, 2) << std::endl;
    std::cout << "I_p_E:\n";
    std::cout << I_T_E(0, 3) << "," << I_T_E(1, 3) << "," << I_T_E(2, 3) << std::endl;
}

void rotate_lidar_points(const std::string& fn,
                         const std::vector<ros::Time>& assoc_enc_sensor_times,
                         const PoseVector& assoc_encoder_poses,
                         const std::string& outbagname,
                         const std::string& topic) {
    // rotate the lidar points in each frame to the IMU frame, using the equation
    // p_I = I_T_E E_T_Er Er_T_L p_L
    // and then save the points to a new lidar point cloud2 message.
    // Note p_W = W_T_E E_T_Er Er_T_L p_L
    // W is a stationary frame on Earth.
    // E is the baselink or encoder frame.
    // Er is the encoder rotating frame.
    // E_T_Er is computed from the encoder poses given a sensor time by linear interpolation
    // Er_T_L is from calib file as given below
    // Er_T_L = [0.007907239801392001, 0.0004581341025153923, -0.1116369382182321, 
    // 0.009711627769598887, 0.7102844361843709, 0.01055670043315378, 0.703768612596697] // px py pz qx qy qz qw
    // E_T_I = [-0.03587546555175705, 0.005730217182290136, 0.032125318431006494,
    //  0.5002131474472541, -0.0020782735504224405, 0.8658824188526003, 0.005488590158788708] // px py pz qx qy qz qw

    Eigen::Affine3d Er_T_L = Eigen::Translation3d(0.007907239801392001, 0.0004581341025153923, -0.1116369382182321) *
                             Eigen::Quaterniond(0.703768612596697, 0.009711627769598887, 0.7102844361843709, 0.01055670043315378);
    Eigen::Affine3d E_T_I = Eigen::Translation3d(-0.03587546555175705, 0.005730217182290136, 0.032125318431006494) *
                            Eigen::Quaterniond(0.005488590158788708, 0.5002131474472541, -0.0020782735504224405, 0.8658824188526003);
    Eigen::Affine3d I_T_E = E_T_I.inverse();
    rosbag::Bag bag;
    rosbag::Bag outbag;
    bag.open(fn, rosbag::bagmode::Read);
    outbag.open(outbagname, rosbag::bagmode::Write);

    rosbag::View view(bag, rosbag::TopicQuery(topic));
    int bad_frames = 0;
    int bad_points = 0;
    int64_t all_points = 0;
    int all_frames = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_msg != nullptr) {
            pcl::PointCloud<hovermap_velodyne::Point> pcl_cloud;
            pcl::fromROSMsg(*cloud_msg, pcl_cloud);

            pcl::PointCloud<hovermap_velodyne::Point> transformed_cloud;
            transformed_cloud.reserve(pcl_cloud.size());
            int new_bad = 0;
            int pts = 0;
            for (const auto& point : pcl_cloud) {
                // Use the timestamp in the point as its absolute sensor time
                ros::Time point_time(point.timestamp);
                if (point_time < assoc_enc_sensor_times.front() || point_time > assoc_enc_sensor_times.back()) {
                    new_bad++;
                    continue;
                }
                Eigen::Affine3d E_T_Er = interpolate_pose(assoc_enc_sensor_times, assoc_encoder_poses, point_time);

                // Apply the transformation: p_I = I_T_E * E_T_Er * Er_T_L * p_L
                Eigen::Vector3d p_L(point.x, point.y, point.z);
                Eigen::Vector3d p_I = I_T_E * E_T_Er * Er_T_L * p_L;
                // Create a new point with transformed coordinates
                hovermap_velodyne::Point transformed_point = point;  // Copy the original point's metadata
                transformed_point.x = p_I.x();
                transformed_point.y = p_I.y();
                transformed_point.z = p_I.z();
                transformed_cloud.push_back(transformed_point);
            }
            if (new_bad > 0) {
                bad_points += new_bad;
                bad_frames++;
            }

            if (transformed_cloud.size()) {
                // Convert the PCL cloud back to a ROS message
                sensor_msgs::PointCloud2 transformed_msg;
                pcl::toROSMsg(transformed_cloud, transformed_msg);
                transformed_msg.header = cloud_msg->header;  // Preserve the header
                transformed_msg.header.stamp = ros::Time(transformed_cloud.points.front().timestamp);  // Set to the first point's sensor time
                transformed_msg.header.frame_id = "imu";

                // Write the new PointCloud2 message to the output bag
                outbag.write(topic, transformed_msg.header.stamp, transformed_msg);
                all_points += transformed_cloud.size();
                all_frames++;
            }
        }
    }

    bag.close();
    outbag.close();
    std::cout << "Discarded " << bad_points << " lidar points among " << bad_frames
            << " frames. Mean points per frame " << all_points / all_frames << "." << std::endl;
    std::cout << "Transformed lidar points from " << all_frames << " msgs have been saved to " << outbagname << std::endl;
}


void save_imu_to_bag(const std::vector<ros::Time>& assoc_imu_sensor_times, 
                     const ImuDataVector& assoc_imu_data, 
                     const std::string& outbagname, 
                     const std::string& topic) {
    if (assoc_imu_sensor_times.size() != assoc_imu_data.size()) {
        std::cerr << "Error: assoc_imu_sensor_times and assoc_imu_data vectors must have the same size." << std::endl;
        return;
    }

    rosbag::Bag outbag;
    try {
        outbag.open(outbagname, rosbag::bagmode::Write);

        for (size_t i = 0; i < assoc_imu_sensor_times.size(); ++i) {
            // Create a sensor_msgs::Imu message
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = assoc_imu_sensor_times[i];
            imu_msg.header.frame_id = "imu";  // Set the frame ID as appropriate

            // Assign angular velocity (gx, gy, gz)
            imu_msg.angular_velocity.x = assoc_imu_data[i][0];
            imu_msg.angular_velocity.y = assoc_imu_data[i][1];
            imu_msg.angular_velocity.z = assoc_imu_data[i][2];

            // Assign linear acceleration (ax, ay, az)
            imu_msg.linear_acceleration.x = assoc_imu_data[i][3];
            imu_msg.linear_acceleration.y = assoc_imu_data[i][4];
            imu_msg.linear_acceleration.z = assoc_imu_data[i][5];

            // Assign orientation quaternion (qx, qy, qz, qw)
            imu_msg.orientation.x = assoc_imu_data[i][6];
            imu_msg.orientation.y = assoc_imu_data[i][7];
            imu_msg.orientation.z = assoc_imu_data[i][8];
            imu_msg.orientation.w = assoc_imu_data[i][9];

            // Write the IMU message to the bag file
            outbag.write(topic, imu_msg.header.stamp, imu_msg);
        }

        outbag.close();
        std::cout << "IMU data saved to " << outbagname << std::endl;
    } catch (const rosbag::BagIOException& e) {
        std::cerr << "Error: Could not open or write to bag file " << outbagname << ": " << e.what() << std::endl;
    }
}

void save_time_and_imu(const std::vector<ros::Time>& assoc_imu_sensor_times, 
                       const ImuDataVector& assoc_imu_data, 
                       const std::string& filename) {
    if (assoc_imu_sensor_times.size() != assoc_imu_data.size()) {
        std::cerr << "Error: assoc_imu_sensor_times and assoc_imu_data vectors must have the same size." << std::endl;
        return;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    for (size_t i = 0; i < assoc_imu_sensor_times.size(); ++i) {
        const auto& time = assoc_imu_sensor_times[i];
        const auto& imu_data = assoc_imu_data[i];

        file << std::fixed << std::setprecision(9);
        file << time.sec << "." << std::setw(9) << std::setfill('0') << time.nsec << " ";
        file << std::setprecision(9);
        file << imu_data[0] << " " << imu_data[1] << " " << imu_data[2] << " ";  // gx, gy, gz
        file << imu_data[3] << " " << imu_data[4] << " " << imu_data[5] << " ";  // ax, ay, az
        file << std::setprecision(9);
        file << imu_data[6] << " " << imu_data[7] << " " << imu_data[8] << " " << imu_data[9] << std::endl;  // qx, qy, qz, qw
    }

    file.close();
    std::cout << "IMU data saved to " << filename << std::endl;
}


void merge_rosbags(const std::vector<std::string>& input_bags, const std::string& output_bag) {
    rosbag::Bag outbag;
    outbag.open(output_bag, rosbag::bagmode::Write);

    for (const auto& bagfile : input_bags) {
        rosbag::Bag inbag;
        inbag.open(bagfile, rosbag::bagmode::Read);

        rosbag::View view(inbag);

        // Iterate over all messages in the current bag and write them to the output bag
        BOOST_FOREACH(rosbag::MessageInstance const m, view) {
            outbag.write(m.getTopic(), m.getTime(), m);
        }

        inbag.close();
        std::cout << "Merged " << bagfile << " into " << output_bag << std::endl;
    }

    outbag.close();
    std::cout << "Bags have been merged into " << output_bag << std::endl;
}


ros::Time parseGpsTime(const std::string& gps_time_str) {
    size_t dot_pos = gps_time_str.find('.');
    if (dot_pos == std::string::npos) {
        throw std::invalid_argument("Invalid GPS time format: no decimal point found.");
    }

    // Split the string into the seconds and fractional parts
    std::string sec_str = gps_time_str.substr(0, dot_pos);
    std::string nsec_str = gps_time_str.substr(dot_pos + 1);

    // Convert the seconds part to an integer
    int sec = std::stoi(sec_str);

    // Ensure the nanoseconds part is exactly 9 digits by padding with zeros if necessary
    if (nsec_str.length() < 9) {
        nsec_str.append(9 - nsec_str.length(), '0');
    } else if (nsec_str.length() > 9) {
        nsec_str = nsec_str.substr(0, 9);  // Truncate to 9 digits if longer
    }

    // Convert the nanoseconds part to an integer
    int nsec = std::stoi(nsec_str);

    // Create and return the ros::Time instance
    return ros::Time(sec, nsec);
}

void read_poses(const std::string& posefile, std::vector<ros::Time>& posetimes, PoseVector& W_T_E_list) {
    // each line, time gpstime x y z pitch roll yaw rot.w rot.x rot.y rot.z
    // we save extract gpstime, xyz and rot.xyzw
    std::ifstream infile(posefile);
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open file " << posefile << std::endl;
        return;
    }

    posetimes.reserve(1000);
    W_T_E_list.reserve(1000);
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream ss(line);
        std::string unused_time_str, gps_time_str;
        double x, y, z, roll, pitch, yaw, rot_x, rot_y, rot_z, rot_w;

        // Read the line and parse the values
        if (!(ss >> unused_time_str >> gps_time_str >> x >> y >> z >> roll >> pitch >> yaw >> rot_w >> rot_x >> rot_y >> rot_z)) {
            std::cerr << "Error: Malformed line in pose file: " << line << std::endl;
            continue;  // Skip to the next line
        }

        // Parse gps_time string to ros::Time using parseGpsTime
        try {
            ros::Time pose_time = parseGpsTime(gps_time_str);
            posetimes.push_back(pose_time);
        } catch (const std::exception& e) {
            std::cerr << "Error: Failed to parse gps_time for line: " << line << " (" << e.what() << ")" << std::endl;
            continue;  // Skip to the next line
        }

        // Create a 7D vector for the pose (x, y, z, rot_x, rot_y, rot_z, rot_w)
        Eigen::Matrix<double, 7, 1> pose;
        pose << x, y, z, rot_x, rot_y, rot_z, rot_w;
        W_T_E_list.push_back(pose);
    }

    infile.close();
    std::cout << "Finished reading poses from " << posefile << std::endl;
    std::cout << "First time " << posetimes[0] << ", pose";
    for (int i = 0; i < 7; ++i) {
        std::cout << " " << W_T_E_list[0][i];
    }
    std::cout << std::endl;
}

void transform_poses(const PoseVector& W_T_I_list, const Eigen::Affine3d& I_T_E, PoseVector& W_T_E_list) {
    W_T_E_list.clear();  // Ensure the output vector is empty before populating
    W_T_E_list.reserve(W_T_I_list.size());

    for (const auto& W_T_I : W_T_I_list) {
        // Extract the translation (x, y, z) from the 7D vector
        Eigen::Vector3d translation_I(W_T_I[0], W_T_I[1], W_T_I[2]);

        // Extract the quaternion (rot_x, rot_y, rot_z, rot_w) from the 7D vector
        Eigen::Quaterniond rotation_I(W_T_I[6], W_T_I[3], W_T_I[4], W_T_I[5]);  // Note: Eigen uses (w, x, y, z)

        // Construct the Affine3d transform for W_T_I
        Eigen::Affine3d W_T_I_transform = Eigen::Translation3d(translation_I) * rotation_I;

        // Apply the I_T_E transformation: W_T_E = W_T_I * I_T_E
        Eigen::Affine3d W_T_E_transform = W_T_I_transform * I_T_E;

        // Extract the transformed translation and rotation
        Eigen::Vector3d translation_E = W_T_E_transform.translation();
        Eigen::Quaterniond rotation_E(W_T_E_transform.rotation());

        // Construct a 7D vector for W_T_E (x, y, z, rot_x, rot_y, rot_z, rot_w)
        Eigen::Matrix<double, 7, 1> W_T_E;
        W_T_E << translation_E.x(), translation_E.y(), translation_E.z(),
                 rotation_E.x(), rotation_E.y(), rotation_E.z(), rotation_E.w();

        // Append to the output vector
        W_T_E_list.push_back(W_T_E);
    }

    std::cout << "Finished transforming poses from W_T_I to W_T_E." << std::endl;
}

typedef pcl::PointCloud<hovermap_velodyne::Point> PointCloudHovermap;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
void aggregate_scans(std::string bagname, std::string posefile, std::string outpcd, 
        std::string topic = "/velodyne_points", double start_time=0, double end_time=10) {
    // TODO(jhuai): first get correspondences of pps sensor times and host times, either encoder or IMU data is OK;
    // then use convex hull to smooth the host jitters, thus a mapping from sensor time to host times 
    // then use the mapping to map the gt host time to sensor time for pose interpolation.

    // bagname: point cloud bag file rectified by rotating the lidar points to the static encoder frame
    // posefile: the hovermap .xyz traj output text file
    // outpcd: aggregated point cloud file
    // topic: /velodyne_points
    // start_time: relative to the begin of the rosbag
    // end_time: relative to the begin of the rosbag
    std::vector<ros::Time> posetimes;
    PoseVector W_T_B_list;
    read_poses(posefile, posetimes, W_T_B_list);

    const ros::Duration host_time_from_sensor_time(1724899451, 232580000);

    rosbag::Bag bag;
    bag.open(bagname, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    ros::Time bag_start_time = view.getBeginTime();
    ros::Duration start(start_time);
    ros::Duration finish(end_time);
    std::cout << "Aggregating points from " << start << " to  " << finish << " of " << bagname << std::endl;
    PointCloudXYZI aggregated_cloud;
    int numframes = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_msg != nullptr && (cloud_msg->header.stamp > bag_start_time + start && 
                cloud_msg->header.stamp <= bag_start_time + finish)) {
            PointCloudHovermap scan_cloud;
            pcl::fromROSMsg(*cloud_msg, scan_cloud);

            PointCloudXYZI transformed_scan_cloud;
            for (const auto& point : scan_cloud) {
                double sensor_time_sec = point.timestamp;  // Using the timestamp field from hovermap_velodyne::Point
                ros::Time sensor_time(sensor_time_sec);
                ros::Time gps_time = sensor_time + host_time_from_sensor_time;

                Eigen::Affine3d W_T_B = interpolate_pose(posetimes, W_T_B_list, gps_time);

                Eigen::Vector3d p_B(point.x, point.y, point.z);
                Eigen::Vector3d p_W = W_T_B * p_B;

                pcl::PointXYZI transformed_point;
                transformed_point.x = p_W.x();
                transformed_point.y = p_W.y();
                transformed_point.z = p_W.z();
                transformed_point.intensity = point.intensity;  // Copy intensity from the original point

                transformed_scan_cloud.push_back(transformed_point);
            }
            ++numframes;
            aggregated_cloud += transformed_scan_cloud;  // Append the current scan to the aggregated point cloud
        }
    }

    bag.close();

    // Save the aggregated point cloud to a PCD file
    pcl::io::savePCDFileASCII(outpcd, aggregated_cloud);
    std::cout << "Saved aggregated point cloud from " << numframes << " frames to " << outpcd << std::endl;
}

std::string rectify_bag(const std::string &fn, const std::string & subfolder, double tol_ms) {
    std::vector<ros::Time> encoder_host_times, encoder_sensor_times;
    read_pps_times(fn, &encoder_host_times, &encoder_sensor_times, "/encoder/pps_timestamp");

    std::vector<ros::Time> lidar_host_times, lidar_sensor_times;
    read_lidar_times(fn, &lidar_host_times, &lidar_sensor_times, "/velodyne_points");

    std::vector<ros::Time> tf_host_times;
    PoseVector tf_encoder_poses;
    read_encoder_tf_msgs(fn, &tf_host_times, &tf_encoder_poses, "/tf");

    std::vector<ros::Time> imu_host_times;
    ImuDataVector imu_datalist;
    read_imu_data(fn, imu_host_times, imu_datalist, "/imu/data");

    std::vector<ros::Time> imu_pps_host_times, imu_pps_sensor_times;
    read_pps_times(fn, &imu_pps_host_times, &imu_pps_sensor_times, "/imu/pps_timestamp");

    std::cout << "Encoder Host Times: " << encoder_host_times.size() << " entries" << std::endl;
    if (!encoder_host_times.empty()) {
        std::cout << "  First: " << encoder_host_times.front() << ", Last-1: " << *(++encoder_host_times.rbegin()) << std::endl;
    }

    std::cout << "Encoder Sensor Times: " << encoder_sensor_times.size() << " entries" << std::endl;
    if (!encoder_sensor_times.empty()) {
        std::cout << "  First: " << encoder_sensor_times.front() << ", Last: " << encoder_sensor_times.back() << std::endl;
    }

    std::cout << "Lidar Host Times: " << lidar_host_times.size() << " entries" << std::endl;
    if (!lidar_host_times.empty()) {
        std::cout << "  First: " << lidar_host_times.front() << ", Last: " << lidar_host_times.back() << std::endl;
    }

    std::cout << "Lidar Sensor Times: " << lidar_sensor_times.size() << " entries" << std::endl;
    if (!lidar_sensor_times.empty()) {
        std::cout << "  First: " << lidar_sensor_times.front() << ", Last: " << lidar_sensor_times.back() << std::endl;
    }

    std::cout << "TF Host Times: " << tf_host_times.size() << " entries" << std::endl;
    if (!tf_encoder_poses.empty()) {
        std::cout << "  First: " << tf_host_times.front() << ", Last: " << tf_host_times.back() << std::endl;
        std::cout << "First encoder pose: " << tf_encoder_poses.front().transpose() << std::endl;
        std::cout << "Last encoder pose: " << tf_encoder_poses.back().transpose() << std::endl;
    } else {
        std::cout << "No encoder poses available." << std::endl;
    }

    std::string tffile = subfolder + "/tf_host_times.txt";
    std::string ppsfile = subfolder + "/encoder_pps_times.txt";
    save_times(encoder_host_times, encoder_sensor_times, ppsfile);
    save_times(tf_host_times, std::vector<ros::Time>(), tffile);

    std::string imufile = subfolder + "/imu_data.txt";
    std::string imuppsfile = subfolder + "/imu_pps_times.txt";
    save_time_and_imu(imu_host_times, imu_datalist, imufile);
    save_times(imu_pps_host_times, imu_pps_sensor_times, imuppsfile);

    std::vector<ros::Time> assoc_enc_sensor_times;
    PoseVector assoc_encoder_poses;
    std::cout << "Associating tf data to encoder pps..." << std::endl;
    associate_data_to_pps(tf_host_times, tf_encoder_poses, encoder_host_times,
            encoder_sensor_times, &assoc_enc_sensor_times, &assoc_encoder_poses, tol_ms);

    std::string assocfile = subfolder + "/assoc_encoder_timed_poses.txt";
    save_time_and_poses(assoc_enc_sensor_times, assoc_encoder_poses, assocfile);

    std::string outbagname1 = subfolder + "/encoder.bag";
    save_encoder_to_bag(assoc_enc_sensor_times, assoc_encoder_poses, outbagname1, "/tf");

    std::string outbagname2 = subfolder + "/velodyne.bag";
    rotate_lidar_points(fn, assoc_enc_sensor_times, assoc_encoder_poses, outbagname2, "/velodyne_points");

    std::vector<ros::Time> assoc_imu_sensor_times;
    ImuDataVector assoc_imu_data;
    std::cout << "Associating IMU data to IMU pps..." << std::endl;
    associate_data_to_pps(imu_host_times, imu_datalist, imu_pps_host_times,
            imu_pps_sensor_times, &assoc_imu_sensor_times, &assoc_imu_data, tol_ms);
    std::string assoc_imu_file = subfolder + "/assoc_imu_timed_data.txt";
    save_time_and_imu(assoc_imu_sensor_times, assoc_imu_data, assoc_imu_file);

    std::string outbagname3 = subfolder + "/imu.bag";
    save_imu_to_bag(assoc_imu_sensor_times, assoc_imu_data, outbagname3, "/imu/data");

    std::string outbagname = subfolder + "/aligned.bag";
    merge_rosbags({outbagname1, outbagname2, outbagname3}, outbagname);

    return outbagname;
}

int main(const int argc, char **argv) {
    if (0) {
        std::string bagname = "Log/aligned.bag";
        std::string posefile = "Output/0829445e8e93_01_Output_traj.xyz";
        std::string outpcd = "Log/merged.pcd";
        aggregate_scans(bagname, posefile, outpcd, "/velodyne_points", 0.1);
        return 0;
    }

    if (argc == 1) {
        std::cout << "Usage: " << argv[0] << " path/to/hovermap/data.bag [path/to/output/dir(Log)] [tol_ms(5.0)]" << std::endl;
        return 0;
    }

    std::string fn = argv[1];
    std::string outdir = "Log";
    if (argc > 2)
        outdir = argv[2];
    if (!outdir.empty() && outdir.back() == '/') {
        outdir.pop_back(); // Remove the last character if it's a '/'
    }

    std::cout << "Output for intermediate files: " << outdir << std::endl;
    double tol_ms = 8.0;
    if (argc > 3)
        tol_ms = std::stod(argv[3]);
    std::cout << "pps and data host time association max diff " << std::fixed
              << std::setprecision(2) << tol_ms << " ms" << std::endl;

    std::vector<std::string> bagnames;

    // Check if `fn` is a directory or a file
    if (fn.size() > 4 && fn.substr(fn.size() - 4) == ".bag") {
        bagnames.push_back(fn);
    } else if (is_directory(fn)) {
        bagnames = search_bag_files(fn);
    }

    // Sort the bag files (assuming names are of the format `xxx_0.bag`, `xxx_1.bag`, etc.)
    std::sort(bagnames.begin(), bagnames.end());
    std::cout << "Rectifying the below bags:" << std::endl;
    size_t i = 0;
    for (const auto& name : bagnames) {
        std::cout << i << ": " << name << std::endl;
        ++i;
    }
    std::vector<std::string> rectified_bagnames;
    rectified_bagnames.reserve(bagnames.size());
    for (const auto& bagname : bagnames) {
        // Extract the filename without extension
        std::string basename = bagname.substr(bagname.find_last_of("/\\") + 1);
        basename = basename.substr(0, basename.size() - 4);

        // Create a subfolder in the output directory
        std::string subfolder = outdir + "/" + basename;
        create_directories(subfolder);

        std::string newbagname = rectify_bag(bagname, subfolder, tol_ms);
        rectified_bagnames.push_back(newbagname);
    }

    if (rectified_bagnames.size() > 1) {
        std::string bagname0 = bagnames[0];
        std::string basename = bagname0.substr(bagname0.find_last_of("/\\") + 1);
        basename = basename.substr(0, basename.size() - 6);
        std::string outbagname = outdir + "/" + basename + "_aligned.bag";
        merge_rosbags(rectified_bagnames, outbagname);
        std::cout << "The overall rectified bag is saved at " << outbagname << std::endl;
    } else {
        std::cout << "The rectified bag is saved at " << rectified_bagnames[0] << std::endl;
    }
    return 0;
}
