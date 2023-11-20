#include "publish_tf.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace liopublisher {
void publish_tf(const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf, double lidar_end_time, 
        tf::TransformBroadcaster& tf_broadcaster) {
    state_ikfom my_state_point = kf.get_x();
    tf::Vector3 position(my_state_point.pos(0), my_state_point.pos(1), my_state_point.pos(2));
    Eigen::Quaterniond w_q_imu = my_state_point.rot;
    tf::Quaternion quat(w_q_imu.x(), w_q_imu.y(), w_q_imu.z(), w_q_imu.w());
    tf::Transform odom_T_baselink(quat, position); // note the baselink is the same as the lidar scan frame, FLU.
    tf::Transform baselink_T_basefootprint(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.1)); // per the burger turtlebot xacro file
    tf::Transform odom_T_basefootprint = odom_T_baselink * baselink_T_basefootprint;
    ros::Time msgtime = ros::Time::now();
    // ros::Time msgtime = ros::Time().fromSec(lidar_end_time); // lidar sensor time can be very different from current time which is used by the planners.
    tf::StampedTransform odom_T_basefootprint_stamped(odom_T_basefootprint, msgtime, "odom", "base_footprint");
    tf_broadcaster.sendTransform(odom_T_basefootprint_stamped);
}
}
