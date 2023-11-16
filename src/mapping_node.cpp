#include "fastlio/laserMapping.hpp"
#include <ros/node_handle.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  fastlio::LaserMapping node(nh);
  node.spin();
  node.saveAndClose();
  return 0;
}
