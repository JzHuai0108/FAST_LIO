#include "fastlio/laserMapping.hpp"
#include <fast_csm_icp/gsm_wrap.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  std::string pcdmap_path;
  nh.param<std::string>("pcdmap", pcdmap_path, "");
  std::shared_ptr<GSMWrap> gsm(new GSMWrap(nh));
  std::string fn_path = pcdmap_path.substr(0, pcdmap_path.find_last_of('/')) + "/";
  gsm->LoadMap(fn_path);
  ros::Rate rate(50);
  geometry_msgs::PoseStamped map_T_lidar;
  while (ros::ok()) {
    if (gsm->loc_status()) {
      map_T_lidar = gsm->init_pose();
      ROS_INFO("Global scan matcher returned initial position %f %f %f", map_T_lidar.pose.position.x, map_T_lidar.pose.position.y, map_T_lidar.pose.position.z);
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  gsm.reset(); // remove the gsm node.

  std::vector<double> position{map_T_lidar.pose.position.x, map_T_lidar.pose.position.y, map_T_lidar.pose.position.z};
  nh.setParam("/mapping/init_world_t_lidar", position);
  std::vector<double> qxyzw{map_T_lidar.pose.orientation.x, map_T_lidar.pose.orientation.y, map_T_lidar.pose.orientation.z, map_T_lidar.pose.orientation.w};
  nh.setParam("/mapping/init_world_qxyzw_lidar", qxyzw);

  fastlio::LaserMapping node(nh);
  node.spin();
  node.saveAndClose();
  return 0;
}
