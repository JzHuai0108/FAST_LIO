#ifndef PUBLISH_TF_H_
#define PUBLISH_TF_H_
#include "use-ikfom.hpp"
#include <ros/publisher.h>
#include <tf/transform_broadcaster.h>

namespace liopublisher {
void publish_tf(const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf, double lidar_end_time, 
        tf::TransformBroadcaster& tf_broadcaster);
} // namespace liopublisher

#endif