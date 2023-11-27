#include "fastlio/laserMapping.hpp"
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  fastlio::LaserMapping node(nh);
  tf::Transform map_T_odom;
  tf::TransformBroadcaster tf_broadcaster;
  ros::Rate rate(50);
  while (ros::ok()) {
      map_T_odom = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
      // since we use the same map_T_odom, we postdate it to avoid global plan to controller tf transform extrapolation.
      tf::StampedTransform map_T_odom_stamped(map_T_odom, ros::Time::now() + ros::Duration(0.5), "map", "odom");
      tf_broadcaster.sendTransform(map_T_odom_stamped);
      ros::spinOnce();
      node.spinOnce();
      rate.sleep();
  }
  // node.spin();
  node.saveAndClose();
  return 0;
}
