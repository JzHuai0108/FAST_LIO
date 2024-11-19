/**
 * rectify the points in ROS1 PointCloud2 messages of hovermap rosbags
 * by transforming these points to the stationary encoder frame.
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/TimeReference.h>
#include <tf2_msgs/TFMessage.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/foreach.hpp>

#include <algorithm>  // For std::lower_bound
#include <iostream>
#include <string>
#include <vector>


namespace hovermap_velodyne {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;  // This macro adds the x, y, z, and padding to align to 16-byte boundaries.
        double timestamp;  // Offset: 16, datatype: double (8 bytes)
        float intensity;   // Offset: 24, datatype: float (4 bytes)
        uint8_t ring;     // Offset: 28, datatype: uint8_t (1 byte)
        uint8_t returnNum; // Offset: 29, datatype: uint8_t (1 byte)

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace hovermap_velodyne

POINT_CLOUD_REGISTER_POINT_STRUCT(hovermap_velodyne::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (double, timestamp, timestamp)
    (float, intensity, intensity)
    (std::uint8_t, ring, ring)
    (std::uint8_t, returnNum, returnNum)
)

typedef std::vector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1>>> PoseVector;
// px py pz qx qy qz qw

typedef std::vector<Eigen::Matrix<double, 10, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 10, 1>>> ImuDataVector;
// gx gy gz ax ay az qx qy qz qw, first 3 for angular rate, second 3 for linear acceleration, last 4 for orientation


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
            start_idx = closest_idx;
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

void rotate_lidar_points(const std::string& fn,
                         const std::vector<ros::Time>& assoc_enc_sensor_times,
                         const PoseVector& assoc_encoder_poses,
                         const std::string& outbagname,
                         const std::string& topic) {
    // rotate the lidar points in each frame to the encoder frame, using the equation
    // p_E = E_T_Er Er_T_L p_L
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
                            Eigen::Quaterniond(0.5002131474472541, -0.0020782735504224405, 0.8658824188526003, 0.005488590158788708);

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

                // Apply the transformation: p_E = E_T_Er * Er_T_L * p_L
                Eigen::Vector3d p_L(point.x, point.y, point.z);
                Eigen::Vector3d p_E = E_T_Er * Er_T_L * p_L;
                // Create a new point with transformed coordinates
                hovermap_velodyne::Point transformed_point = point;  // Copy the original point's metadata
                transformed_point.x = p_E.x();
                transformed_point.y = p_E.y();
                transformed_point.z = p_E.z();
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
    std::cout << "All bags have been merged into " << output_bag << std::endl;
}


int main(const int argc, char **argv) {
    if (argc == 1) {
        std::cout << "Usage: " << argv[0] << " path/to/hovermap/data.bag [path/to/output/dir(Log)] [tol_ms(6.0)]" << std::endl;
        return 0;
    }

    std::string fn = argv[1];
    std::string outdir = "Log";
    if (argc > 2)
        outdir = argv[2];
    std::cout << "Output for intermediate files: " << outdir << std::endl;
    double tol_ms = 6.0;
    if (argc > 3)
        tol_ms = std::stod(argv[3]);
    std::cout << "pps and data host time association max diff " << std::fixed
              << std::setprecision(2) << tol_ms << " ms" << std::endl;

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
    std::string tffile = outdir + "/tf_host_times.txt";
    std::string ppsfile = outdir + "/encoder_pps_times.txt";
    save_times(encoder_host_times, encoder_sensor_times, ppsfile);
    save_times(tf_host_times, std::vector<ros::Time>(), tffile);

    std::string imufile = outdir + "/imu_data.txt";
    std::string imuppsfile = outdir + "/imu_pps_times.txt";
    save_time_and_imu(imu_host_times, imu_datalist, imufile);
    save_times(imu_pps_host_times, imu_pps_sensor_times, imuppsfile);

    std::vector<ros::Time> assoc_enc_sensor_times;
    PoseVector assoc_encoder_poses;
    associate_data_to_pps(tf_host_times, tf_encoder_poses, encoder_host_times,
            encoder_sensor_times, &assoc_enc_sensor_times, &assoc_encoder_poses, tol_ms);

    std::string assocfile = outdir + "/assoc_encoder_timed_poses.txt";
    save_time_and_poses(assoc_enc_sensor_times, assoc_encoder_poses, assocfile);

    std::string basename = fn.substr(fn.find_last_of("/\\") + 1); // Extract the filename only (remove path)
    basename = basename.substr(0, basename.size() - 4); // Remove the last 4 characters (e.g., ".bag")
    std::string outbagname1 = outdir + "/" + basename + "_encoder.bag";
    save_encoder_to_bag(assoc_enc_sensor_times, assoc_encoder_poses, outbagname1, "/tf");

    std::string outbagname2 = outdir + "/" + basename + "_velodyne.bag";
    rotate_lidar_points(fn, assoc_enc_sensor_times, assoc_encoder_poses, outbagname2, "/velodyne_points");

    std::vector<ros::Time> assoc_imu_sensor_times;
    ImuDataVector assoc_imu_data;
    associate_data_to_pps(imu_host_times, imu_datalist, imu_pps_host_times,
            imu_pps_sensor_times, &assoc_imu_sensor_times, &assoc_imu_data, tol_ms);
    std::string assoc_imu_file = outdir + "/assoc_imu_timed_data.txt";
    save_time_and_imu(assoc_imu_sensor_times, assoc_imu_data, assoc_imu_file);

    std::string outbagname3 = outdir + "/" + basename + "_imu.bag";
    save_imu_to_bag(assoc_imu_sensor_times, assoc_imu_data, outbagname3, "/imu/data");

    std::string outbagname = outdir + "/" + basename + "_aligned.bag";
    merge_rosbags({outbagname1, outbagname2, outbagname3}, outbagname);

    return 0;
}
