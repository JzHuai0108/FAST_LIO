"""
extract a pointcloud2 message from rosbag and save the xyz components as a pcd file
"""
import argparse
import os
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d

def extract_pcd_withimu(bagfile, pcd_topic, imu_topic, pcddir):
    """
    extract a pointcloud2 message from rosbag while have imu at the same time, and save as a pcd
    """
    bag = rosbag.Bag(bagfile)

    # Inserted code starts here
    pointcloud2_msg = None
    imu_msg = None
    first_scan_t = 0
    for topic, msg, t in bag.read_messages(topics=[pcd_topic, imu_topic]):
        if topic == pcd_topic and imu_msg is not None:
            pointcloud2_msg = msg
        elif topic == imu_topic:
            imu_msg = msg
        if pointcloud2_msg is not None and imu_msg is not None:
            first_scan_t =t
            break

    if pointcloud2_msg is not None and imu_msg is not None:
        # Save pointcloud2 message as pcd
        gen = pc2.read_points(pointcloud2_msg, skip_nans=True)
        int_data = list(gen)
        xyz = np.array([[0, 0, 0]])
        for x in int_data:
            xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(xyz)
        output_filename = os.path.join(pcddir, str(first_scan_t) + ".pcd")
        o3d.io.write_point_cloud(output_filename, out_pcd)
    else:
        print("Error: Pointcloud2 message or IMU message not found in the bag file.")



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bagfile', type=str, help='rosbag file')
    parser.add_argument('--topic', type=str, help='topic name', default='/velodyne_points')
    parser.add_argument('--imu_topic', type=str, help='topic name', default='/zed2i/zed_node/imu/data')
    parser.add_argument('--pcddir', type=str, help='pcd file directory')
    args = parser.parse_args()

    if not os.path.exists(args.pcddir):
        os.mkdir(args.pcddir)

    extract_pcd_withimu(args.bagfile, args.topic, args.imu_topic, args.pcddir)

