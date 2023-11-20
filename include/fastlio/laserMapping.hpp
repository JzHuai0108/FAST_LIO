#include <cstdio>
#include <fstream>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf/transform_broadcaster.h>

namespace fastlio {
class LaserMapping {
public:
    LaserMapping(ros::NodeHandle& nh);
    ~LaserMapping();
    void spinOnce();
    int spin();
    void saveAndClose();

    int frame_num;
    double aver_time_consu, aver_time_icp, aver_time_match, aver_time_incre, aver_time_solve, aver_time_const_H_time;

    FILE *fp;
    std::ofstream fout_pre, fout_out, fout_dbg;

    ros::Subscriber sub_pcl;
    ros::Subscriber sub_imu;
    ros::Publisher pubLaserCloudFull;
    ros::Publisher pubLaserCloudFull_body;
    ros::Publisher pubLaserCloudEffect;
    ros::Publisher pubLaserCloudMap;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubPath;

    tf::TransformBroadcaster tf_broadcaster_;
};

} // namespace fastlio