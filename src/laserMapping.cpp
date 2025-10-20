// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "lidar_localizer.h"

#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>

#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <fast_lio/CustomMsg.h>

#include "aggregate_pcds.hpp"
#include "dist_checkup.h"
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
bool publish_cloud_in_imu_frame = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string bagfile;
string map_file_path, lid_topic, imu_topic;
string state_log_dir;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool point_to_plane_cost = false; // false means distribution to distrition cost other than point to plane cost.

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterMap;

// a kdtree has to be a global variable because of multithreading, see
// https://github.com/hku-mars/ikd-Tree/issues/8
KD_TREE<PointType> ikdtree;
int odom_mode = 0;
std::string tls_dir = "";
std::string init_lidar_pose_file = "";
vector<double> init_world_t_imu(3, 0.0);
vector<double> init_world_rpy_imu(3, 0.0);
V3D init_world_t_imu_vec(Zero3d);
M3D init_world_R_imu(Eye3d);
V3D init_world_v_imu_vec(Zero3d);
double msg_start_time;
double msg_end_time;
// start time in unix time to process the lidar data. 
// If empty or 0, the first lidar message will be used.
// This time corresponds to the init_world_t_imu_vec and init_world_R_imu.
ros::Time msg_start_time_ros;
ros::Time msg_end_time_ros;
bool stationary_start;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(double lidar_end_time, const state_ikfom& state_point, FILE *fp, Pose3d &B_T_S) {
    fprintf(fp, "%.9lf ", lidar_end_time);
    V3D w_t_s = state_point.rot * B_T_S.p + state_point.pos;
    Eigen::Quaterniond w_r_s = state_point.rot * B_T_S.R;
    fprintf(fp, "%.6lf %.6lf %.6lf ", w_t_s(0), w_t_s(1), w_t_s(2));
    fprintf(fp, "%.9lf %.9lf %.9lf %.9lf ", w_r_s.coeffs()[0], w_r_s.coeffs()[1], w_r_s.coeffs()[2], w_r_s.coeffs()[3]);
    fprintf(fp, "%.6lf %.6lf %.6lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel
    fprintf(fp, "%.6lf %.6lf %.6lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%.6lf %.6lf %.6lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%.6lf %.6lf %.6lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // gravity in world 
    fprintf(fp, "%.6lf %.6lf %.6lf ", state_point.offset_T_L_I(0), state_point.offset_T_L_I(1), state_point.offset_T_L_I(2));
    fprintf(fp, "%.9lf %.9lf %.9lf %.9lf ", state_point.offset_R_L_I.coeffs()[0], state_point.offset_R_L_I.coeffs()[1], state_point.offset_R_L_I.coeffs()[2], state_point.offset_R_L_I.coeffs()[3]);
    fprintf(fp, "\n");
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_lidar(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_lidar + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
    V3D p_lidar(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_lidar + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_lidar(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_lidar + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_lidar(pi->x, pi->y, pi->z);
    V3D p_imu(state_point.offset_R_L_I*p_lidar + state_point.offset_T_L_I);

    po->x = p_imu(0);
    po->y = p_imu(1);
    po->z = p_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;
    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const fast_lio::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double accelerometer_scale;
double gyro_scale;

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();
    msg->linear_acceleration.x *= accelerometer_scale;
    msg->linear_acceleration.y *= accelerometer_scale;
    msg->linear_acceleration.z *= accelerometer_scale;

    msg->angular_velocity.x *= gyro_scale;
    msg->angular_velocity.y *= gyro_scale;
    msg->angular_velocity.z *= gyro_scale;

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

void updateKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr TLS_submap, const TlsPositionVector &TLS_positions,
                  int prev_nearest_scan_idx, int nearest_index) {
    downSizeFilterMap.setInputCloud(TLS_submap);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ds(new pcl::PointCloud<pcl::PointXYZ>());
    downSizeFilterMap.filter(*map_ds);
    PointVector PointToAdd;
    PointToAdd.reserve(map_ds->points.size());
    for (int i = 0; i < map_ds->points.size(); i++) {
        pcl::PointXYZINormal point;
        point.x = map_ds->points[i].x;
        point.y = map_ds->points[i].y;
        point.z = map_ds->points[i].z;
        PointToAdd.push_back(point);
    }
    ikdtree.Build(PointToAdd);
    if (prev_nearest_scan_idx >= 0) {
        ROS_INFO(("New nearest id %d, project id %d, old nearest id %d, project id %d, "
             "%zu points downsampled from %zu."),
            nearest_index, (int)TLS_positions[nearest_index][3], prev_nearest_scan_idx, 
            (int)TLS_positions[prev_nearest_scan_idx][3], PointToAdd.size(), TLS_submap->points.size());
    } else {
        ROS_INFO(("New nearest id %d, project id %d, "
             "%zu points downsampled from %zu."),
            nearest_index, (int)TLS_positions[nearest_index][3], PointToAdd.size(), TLS_submap->points.size());
    }
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));
        if (pcd_save_interval == 1) {
            for (int i = 0; i < size; i++) {
                // save the pointcloud in the local lidar frame.
                Matrix<double, 3, 1> temp;
                temp(0) = feats_undistort->points[i].x;
                temp(1) = feats_undistort->points[i].y;
                temp(2) = feats_undistort->points[i].z;
                // temp = state_point.offset_R_L_I*temp + state_point.offset_T_L_I;
                laserCloudWorld->points[i].x = temp(0);
                laserCloudWorld->points[i].y = temp(1);
                laserCloudWorld->points[i].z = temp(2);
                laserCloudWorld->points[i].intensity = feats_undistort->points[i].intensity;
            }
            *pcl_wait_save = *laserCloudWorld;
        } else {
            for (int i = 0; i < size; i++) {
                RGBpointBodyToWorld(&feats_undistort->points[i], \
                                    &laserCloudWorld->points[i]);
            }
            *pcl_wait_save += *laserCloudWorld;
        }

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;

            std::stringstream ss;
            std::string all_points_dir = state_log_dir + "/pcd/";
            ss << std::fixed << std::setprecision(9) << lidar_end_time << ".pcd";
            std::string fn = all_points_dir + ss.str();

            pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinaryCompressed(fn, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    sensor_msgs::PointCloud2 laserCloudmsg;
    if (publish_cloud_in_imu_frame) {
        PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                                &laserCloudIMUBody->points[i]);
        }
        pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    } else {
        pcl::toROSMsg(*feats_undistort, laserCloudmsg);
    }

    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    if (publish_cloud_in_imu_frame)
        set_posestamp(odomAftMapped.pose);
    else {
        SO3 odom_R_lidar = state_point.rot * state_point.offset_R_L_I;
        V3D odom_t_lidar = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        odomAftMapped.pose.pose.position.x = odom_t_lidar[0];
        odomAftMapped.pose.pose.position.y = odom_t_lidar[1];
        odomAftMapped.pose.pose.position.z = odom_t_lidar[2];
        odomAftMapped.pose.pose.orientation.x = odom_R_lidar.coeffs()[0];
        odomAftMapped.pose.pose.orientation.y = odom_R_lidar.coeffs()[1];
        odomAftMapped.pose.pose.orientation.z = odom_R_lidar.coeffs()[2];
        odomAftMapped.pose.pose.orientation.w = odom_R_lidar.coeffs()[3];
    }
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

Eigen::Matrix3d compute_COV(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    Eigen::Vector4d pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3d covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3d eigenValuesPCA = eigen_solver.eigenvalues();

    Eigen::MatrixXd dz_cov_mat = Eigen::MatrixXd::Identity(3,3);
    dz_cov_mat(0,0) = 0.001;
    Eigen::Matrix3d cov_mat = eigenVectorsPCA*dz_cov_mat*eigenVectorsPCA.transpose();

    // // normal*normal^T
    // cov_mat = eigenVectorsPCA.col(0)*eigenVectorsPCA.col(0).transpose();

    return cov_mat;
}


PointCloudXYZI::Ptr NearPoint(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_NearPoint(new PointCloudXYZI(100000, 1));
std::vector<M3D> normMlist(100000);
std::vector<M3D> corr_normMlist(100000);
const int Ndim = 12;
int min_effct_feat_num = 10;
int dist_type = 2;
double range_th = 1.0;
double icp_dist_thresh = 0.9; // The larger, the less points will be selected for ICP matching.
double est_plane_thresh = 0.1; // The larger, the more surfels will be accepted as planes.

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear();
    corr_NearPoint->clear();
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    // #ifdef MP_EN
    //     omp_set_num_threads(MP_PROC_NUM);
    //     #pragma omp parallel for
    // #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;
        // point_world.normal_x = point_body.normal_x;  // RCS. jhuai: is this needed?

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, float(est_plane_thresh)))
        {
             if (!point_to_plane_cost) {
                float pd2 =sqrt((points_near[0].x-point_world.x)*(points_near[0].x-point_world.x)
                            + (points_near[0].y-point_world.y)*(points_near[0].y-point_world.y)
                            + (points_near[0].z-point_world.z)*(points_near[0].z-point_world.z));
                float ss = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
                if (ss > icp_dist_thresh){
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                    for(int ii=0;ii<points_near.size();ii++){
                        pcl::PointXYZ point = { points_near[ii].x, points_near[ii].y, points_near[ii].z};
                        cloud->insert(cloud->begin() + ii, point);
                    }
                    Eigen::Matrix3d cov_ref = Eigen::MatrixXd::Identity(3,3);
                    cov_ref = compute_COV(cloud);
                    // std::cout<<"cov_ref:\n"<< cov_ref <<std::endl;
                    // std::cout<<"normal:\n"<<compute_normal(cloud)<<std::endl;
                    // std::cout<<pabcd<<std::endl;

                    double time4 = ros::Time::now().toSec();
                    // PointType point0;
                    // std::vector<int> knearID(5);
                    // PointVector knearPoint(5);
                    // std::vector<float> knearDist(5);
                    // point0.x = p_body[0]; point0.y = p_body[1]; point0.z = p_body[2];
                    // // std::cout<<  point0.x <<" "<< point0.y <<" "<< point0.z << std::endl;
                    // mykdtree.Nearest_Search(point0, 5, knearPoint, knearDist);
                    // // mykdtree.Radius_Search(point0, 1, knearPoint)
                    // cloud->clear();
                    // for(int ii=0;ii<knearPoint.size();ii++){
                    //     // std::cout<<  knearPoint[ii].x <<" "<< knearPoint[ii].y <<" "<< knearPoint[ii].z << std::endl;
                    //     pcl::PointXYZ point = {  knearPoint[ii].x, knearPoint[ii].y, knearPoint[ii].z};
                    //     cloud->insert(cloud->begin() + ii, point);
                    // }
                    // std::cout << "kdtree2" << (ros::Time::now().toSec() - time4)*1000 << std::endl;

                    // std::cout<<"\n";
                    Eigen::Matrix3d cov_data = 0.1*Eigen::MatrixXd::Identity(3,3);
                    // cov_data = compute_COV(cloud);
                    // std::cout<<"cov_data:\n"<< cov_data <<std::endl;
                    normMlist[i] = Eigen::MatrixXd::Identity(3,3);
                    normMlist[i] = (cov_ref + (s.rot*s.offset_R_L_I).toRotationMatrix()*cov_data*(s.rot*s.offset_R_L_I).toRotationMatrix().transpose()).inverse();

                    point_selected_surf[i] = true;
                    pointSearchSqDis[0]+=0.0000001;
                    pointSearchSqDis[1]+=0.0000001;
                    pointSearchSqDis[2]+=0.0000001;
                    double sw = 1/pointSearchSqDis[0]+1/pointSearchSqDis[1]+1/pointSearchSqDis[2];
                    NearPoint->points[i].x = (points_near[0].x/pointSearchSqDis[0]+points_near[1].x/pointSearchSqDis[1]+points_near[2].x/pointSearchSqDis[2])/sw;
                    NearPoint->points[i].y = (points_near[0].y/pointSearchSqDis[0]+points_near[1].y/pointSearchSqDis[1]+points_near[2].y/pointSearchSqDis[2])/sw;
                    NearPoint->points[i].z = (points_near[0].z/pointSearchSqDis[0]+points_near[1].z/pointSearchSqDis[1]+points_near[2].z/pointSearchSqDis[2])/sw;
                    // NearPoint->points[i].x = points_near[0].x;
                    // NearPoint->points[i].y = points_near[0].y;
                    // NearPoint->points[i].z = points_near[0].z;
                    // NearPoint->points[i].normal_x = points_near[0].normal_x; // jhuai: is this needed?
                    normvec->points[i].x = pabcd(0);
                    normvec->points[i].y = pabcd(1);
                    normvec->points[i].z = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i] = abs(pd2);
                }
            }
            else{
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float ss = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
                if (ss > icp_dist_thresh){
                    normMlist[i] = Eigen::MatrixXd::Identity(3,3);
                    point_selected_surf[i] = true;
                    NearPoint->points[i].x = points_near[0].x;
                    NearPoint->points[i].y = points_near[0].y;
                    NearPoint->points[i].z = points_near[0].z;
                    normvec->points[i].x = pabcd(0);
                    normvec->points[i].y = pabcd(1);
                    normvec->points[i].z = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i] = abs(pd2);
                }
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            corr_NearPoint->points[effct_feat_num] = NearPoint->points[i];
            corr_normMlist[effct_feat_num] = normMlist[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < min_effct_feat_num)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    if (!point_to_plane_cost) {
        ekfom_data.h_x = MatrixXd::Zero(3*effct_feat_num, 12); //23
        ekfom_data.h.resize(3*effct_feat_num);
        for (int i = 0; i < effct_feat_num; i++)
        {
            const PointType &laser_p  = laserCloudOri->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat<<SKEW_SYM_MATRX(point_this);
            V3D point_gthis = s.rot * point_this + s.pos;

            /*** get the normal vector of closest surface/corner ***/
            const PointType &norm_p = corr_normvect->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, Ndim);
            M3D H_p = Eigen::MatrixXd::Identity(3,3);
            M3D H_q = -1*s.rot.toRotationMatrix()*skew_sym_mat(V3D(point_this));
            M3D H_q_b_r = -1*s.rot.toRotationMatrix() * s.offset_R_L_I.toRotationMatrix()* skew_sym_mat(V3D(point_this_be));
            M3D H_l_b_r = s.rot.toRotationMatrix();
            H.block<3, 3>(0, 0) = H_p;
            H.block<3, 3>(0, 3) = H_q;
            H.block<3, 3>(0, 6) = H_q_b_r;
            H.block<3, 3>(0, 9) = H_l_b_r;

            // normM = norm_vec*norm_vec.transpose();
            Eigen::MatrixXd normM = Eigen::MatrixXd::Identity(3,3); 
            if(dist_type==1){
                normM = norm_vec*norm_vec.transpose();
            }
            else if(dist_type==2){
                normM = corr_normMlist[i];
            }
            // residual and H
            double Range_wgt = 1.0;
            Range_wgt = exp(-point_this_be.norm()/range_th);
            V3D r = Range_wgt * normM * (V3D(corr_NearPoint->points[i].x,corr_NearPoint->points[i].y,corr_NearPoint->points[i].z) - point_gthis);
            H = Range_wgt * normM * H;
            if (extrinsic_est_en){
                ekfom_data.h_x.block<3, Ndim>(3*i, 0) =  H;
                ekfom_data.h.block<3, 1>(3*i, 0) =  r;
            }
            else{
                H.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();
                H.block<3, 3>(0, 9) = Eigen::Matrix3d::Zero();
                ekfom_data.h_x.block<3, Ndim>(3*i, 0) =  H;
                ekfom_data.h.block<3, 1>(3*i, 0) =  r;
            }
        }
    }
    else{
        ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
        ekfom_data.h.resize(effct_feat_num);
        for (int i = 0; i < effct_feat_num; i++)
        {
            const PointType &laser_p  = laserCloudOri->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat<<SKEW_SYM_MATRX(point_this);
            V3D point_gthis = s.rot * point_this + s.pos;

            /*** get the normal vector of closest surface/corner ***/
            const PointType &norm_p = corr_normvect->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            /*** calculate the Measuremnt Jacobian matrix H ***/
            V3D C(s.rot.conjugate() *norm_vec);
            V3D A(point_crossmat * C);
            if (extrinsic_est_en)
            {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            /*** Measuremnt: distance to the closest surface/corner ***/
            ekfom_data.h(i) = -norm_p.intensity;
        }
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LIOdometer {
private:
    ros::Time lastMsgTime;
    TlsPositionVector tls_position_ids;
    ofstream fout_pre, fout_out;
    FILE *fp = NULL;

    ros::Subscriber sub_pcl;
    ros::Subscriber sub_imu;
    ros::Publisher pubLaserCloudFull;
    ros::Publisher pubLaserCloudFull_body;
    ros::Publisher pubLaserCloudEffect;
    ros::Publisher pubLaserCloudMap;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubPath;

    ros::Publisher pubMapPath;
    ros::Publisher pubMapFrame;
    ros::Publisher pubMapPose;
    ros::Publisher pubPriorMap;

    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;

    Pose3d B_T_S; // transform the output reference frame from B frame to S frame.
    std::string output_ref_frame = "lidar"; // output reference frame, "imu" or "lidar"
    double init_pos_noise = 0.0;
    double init_rot_noise = 0.0;
    bool show_submap = false;
    std::string tls_ref_traj_files = "";
    double tls_dist_thresh = 8; // the max distance to the tls trajectory to abort the odometer.
    DistCheckup dist_checkup;
    int nearest_scan_idx = -1; // scan idx within the tls_position_ids.
    std::string state_filename = "scan_states.txt";
    std::string pos_log_filename;

    LidarLocalizer lidar_localizer;
    int loc_accum_window;
    double loc_follow_odom;

public:

bool is_livox_custom_msg() const {
    return p_pre->lidar_type == AVIA;
}

int initializeSystem(ros::NodeHandle &nh) {
    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("mapping/max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("bagfile", bagfile, "");
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("mapping/filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("mapping/filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("mapping/cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("mapping/point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("point_to_plane_cost", point_to_plane_cost, false);
    nh.param<bool>("publish/runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<bool>("publish/publish_cloud_in_imu_frame", publish_cloud_in_imu_frame, true);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<int>("odom_mode", odom_mode, 0);
    nh.param<int>("loc_accum_window", loc_accum_window, 5);
    nh.param<double>("loc_follow_odom", loc_follow_odom, 0.0);

    nh.param<std::string>("publish/output_ref_frame", output_ref_frame, "lidar");
    nh.param<bool>("publish/show_submap", show_submap, false);

    nh.param<std::string>("tls_dir", tls_dir, "");
    nh.param<std::string>("init_lidar_pose_file", init_lidar_pose_file, "");
    nh.param<double>("msg_start_time", msg_start_time, 0.0);
    nh.param<double>("msg_end_time", msg_end_time, 0.0);
    nh.param<bool>("stationary_start", stationary_start, true);
    nh.param<std::string>("tls_ref_traj_files", tls_ref_traj_files, "");
    nh.param<double>("mapping/tls_dist_thresh", tls_dist_thresh, 8);
    nh.param<double>("mapping/init_pos_noise", init_pos_noise, 0.0);
    nh.param<double>("mapping/init_rot_noise", init_rot_noise, 0.0);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<double>("mapping/gravity_m_s2", p_imu->G_m_s2, 9.81);
    nh.param<double>("mapping/accelerometer_scale", accelerometer_scale, 1.0);
    nh.param<double>("mapping/gyro_scale", gyro_scale, 1.0);
    nh.param<string>("save_dir", state_log_dir, "");
    nh.param<string>("state_filename", state_filename, "scan_states.txt");
    nh.param<vector<double>>("mapping/init_world_t_imu", init_world_t_imu, vector<double>());
    nh.param<vector<double>>("mapping/init_world_rpy_imu", init_world_rpy_imu, vector<double>());
    nh.param<double>("mapping/icp_dist_thresh", icp_dist_thresh, 0.9);
    nh.param<double>("mapping/est_plane_thresh", est_plane_thresh, 0.1);
    nh.param<int>("mapping/min_effct_feat_num", min_effct_feat_num, 10);
    nh.param<int>("mapping/dist_type", dist_type, 2);
    nh.param<double>("mapping/range_th", range_th, 1.0);

    init_world_t_imu_vec = V3D(init_world_t_imu[0], init_world_t_imu[1], init_world_t_imu[2]);
    init_world_R_imu = EulerToRotM(init_world_rpy_imu);
    msg_start_time_ros = ros::Time(msg_start_time);
    msg_end_time_ros = ros::Time(msg_end_time);
    if (state_log_dir.empty()) {
      cerr << "You have to provide save_dir to make the saving functions work properly." << std::endl;
      return 0;
    } else {
      make_log_dirs(state_log_dir); // make directories, state_log_dir, and state_log_dir/"pcd"
    }
    cout << "state log dir: " << state_log_dir << ", state filename: " << state_filename << endl;
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    ROS_INFO("lidar extrinsic p: %.3f %.3f %.3f, R:\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f",
             Lidar_T_wrt_IMU(0), Lidar_T_wrt_IMU(1), Lidar_T_wrt_IMU(2),
             Lidar_R_wrt_IMU(0,0), Lidar_R_wrt_IMU(0,1), Lidar_R_wrt_IMU(0,2),
             Lidar_R_wrt_IMU(1,0), Lidar_R_wrt_IMU(1,1), Lidar_R_wrt_IMU(1,2),
             Lidar_R_wrt_IMU(2,0), Lidar_R_wrt_IMU(2,1), Lidar_R_wrt_IMU(2,2));

    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->stationary_start_ = stationary_start;

    if (output_ref_frame == "lidar") {
        B_T_S = Pose3d(Lidar_R_wrt_IMU, Lidar_T_wrt_IMU);
    } else if (output_ref_frame == "imu") {
        B_T_S = Pose3d();
    } else {
        cerr << "Invalid output_ref_frame: " << output_ref_frame << std::endl;
        return 0;
    }

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    pos_log_filename = state_log_dir + "/" + state_filename.substr(0, state_filename.size() - 4) + "_odom.txt";
    fp = fopen(pos_log_filename.c_str(),"w");
    if (fp == NULL) {
        std::cout << "Failed to create state file " << pos_log_filename << "." << std::endl;
        return 0;
    }

    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << ROOT_DIR<<" file opened" << endl;
    else
        cout << ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk, ros::TransportHints().tcpNoDelay()) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk, ros::TransportHints().tcpNoDelay());
    sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk, ros::TransportHints().tcpNoDelay());
    pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 100000);

    pubMapPath = nh.advertise<nav_msgs::Path>("/map_path", 100);
    pubMapFrame = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 10);
    pubMapPose = nh.advertise<nav_msgs::Odometry>("/map_pose", 10);
    pubPriorMap = nh.advertise<sensor_msgs::PointCloud2>("/prior_map", 10);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    lastMsgTime = ros::Time::now();
    nearest_scan_idx = -1;

    if (odom_mode == ODOM_MODE::LocToMap) {
        std::cout << "init_lidar_pose_file: " << init_lidar_pose_file << std::endl;
        V3D init_w_t_lidar;
        M3D init_w_R_lidar;
        V3D init_w_v_lidar;
        std::string timestr;
        load_initial_lidar_pose(init_lidar_pose_file, init_w_t_lidar, init_w_R_lidar, init_w_v_lidar, timestr);
        add_pose_noise(init_w_t_lidar, init_w_R_lidar, init_pos_noise, init_rot_noise);
        init_world_R_imu = init_w_R_lidar * Lidar_R_wrt_IMU.transpose();
        init_world_t_imu_vec = init_w_t_lidar - init_world_R_imu * Lidar_T_wrt_IMU;
        init_world_v_imu_vec = init_w_v_lidar;
        std::cout << "TLS dir: " << tls_dir << std::endl;
        std::string tls_project_dir = tls_dir + "/project1/regis";
        load_tls_project_poses(tls_project_dir, tls_position_ids);
        tls_project_dir = tls_dir + "/project2/regis";
        load_tls_project_poses(tls_project_dir, tls_position_ids);

        ikdtree.set_downsample_param(filter_size_map_min);
        if (!tls_ref_traj_files.empty()) {
            std::vector<std::string> filenames;
            boost::split(filenames, tls_ref_traj_files, boost::is_any_of(";"));
            ROS_INFO("TLS reference trajectory files: %s", tls_ref_traj_files.c_str());
            ROS_INFO("TLS distance threshold: %.2f", tls_dist_thresh);
            Trajectory ref_traj;
            load_ref_traj(filenames, ref_traj);
            dist_checkup.initialize(ref_traj, tls_dist_thresh);
        } else {
            ROS_WARN("No TLS reference trajectory files provided, TLS checkup disabled.");
        }
    } else if (odom_mode == ODOM_MODE::LocWithOdom) {
        lidar_localizer.initializeImu(Lidar_R_wrt_IMU, Lidar_T_wrt_IMU, gyr_cov, acc_cov, b_gyr_cov, b_acc_cov, p_imu->G_m_s2);
        std::string loc_state_file = state_log_dir + "/" + state_filename;
        lidar_localizer.initialize(init_lidar_pose_file, tls_dir, tls_ref_traj_files,
                                   Lidar_R_wrt_IMU, Lidar_T_wrt_IMU, p_imu->G_m_s2,
                                   filter_size_surf_min, filter_size_map_min, tls_dist_thresh, loc_state_file);
        state_ikfom init_state = lidar_localizer.getLatestState();
        init_world_t_imu_vec = init_state.pos;
        init_world_R_imu = init_state.rot.toRotationMatrix();
        init_world_v_imu_vec = init_state.vel;
        p_imu->set_init_pose(init_world_t_imu_vec, init_world_R_imu, init_world_v_imu_vec);

        // lidar_localizer.setPublishers(&pubMapPath, &pubMapFrame, &pubMapPose, &pubPriorMap);
        lidar_localizer.setPublishers(&pubMapPath, &pubMapFrame, &pubMapPose, nullptr); // disable prior map publishing for efficiency.
        lidar_localizer.setFollowOdometer(loc_follow_odom);
        lidar_localizer.setAccumWindow(loc_accum_window);
    }
    cout << "odom_mode? " << OdomModeToString(odom_mode) << ", loc_accum_window " << loc_accum_window
         << ", loc_follow_odom " << loc_follow_odom << ", filter size map " << filter_size_map_min << std::endl;
    cout << "init_world_t_imu_vec: " << init_world_t_imu_vec[0] << " " << init_world_t_imu_vec[1] << " " << init_world_t_imu_vec[2] << endl;
    cout << "init_world_R_imu: " << endl << init_world_R_imu << endl;
    cout << "init_world_v_imu_vec: " << init_world_v_imu_vec[0] << " " << init_world_v_imu_vec[1] << " " << init_world_v_imu_vec[2] << endl;
    cout << "msg_start_time: " << msg_start_time << ", msg_start_time_ros: " 
         << msg_start_time_ros.sec << "." << std::setw(9) << std::setfill('0')
         << msg_start_time_ros.nsec << std::endl;
    cout << "msg_end_time: " << msg_end_time << ", msg_end_time_ros: "
         << msg_end_time_ros.sec << "." << std::setw(9) << std::setfill('0')
         << msg_end_time_ros.nsec << std::endl;
    cout << "stationary start? " << stationary_start << endl;
    cout << "p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    cout << "show submap " << show_submap << endl;
    return 1;
}

    bool spinOnce() {
        // return true if to exit, false otherwise.
        if (flg_exit) return true;
        // if(!flg_first_scan && ros::Time::now()-lastMsgTime > ros::Duration(20.0)){
        //     ros::shutdown();
        //     flg_exit = true;
        //     return true; //20s no data, exit
        // }
        ros::spinOnce();
        if(sync_packages(Measures)) 
        {
            lastMsgTime = ros::Time::now();
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                if (odom_mode == ODOM_MODE::LocToMap) {
                    p_imu->set_init_pose(init_world_t_imu_vec, init_world_R_imu, init_world_v_imu_vec);
                }
                flg_first_scan = false;
                return false;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            lidar_localizer.propagateCov(Measures);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this undistorted scan!");
                return false;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            if (odom_mode == ODOM_MODE::LocToMap) {
                ikdtree.set_downsample_param(filter_size_map_min);
                pcl::PointCloud<pcl::PointXYZ>::Ptr TLS_submap(new pcl::PointCloud<pcl::PointXYZ>());
                int prev_nearest_idx = nearest_scan_idx;
                nearest_scan_idx = load_close_tls_scans(tls_position_ids, state_point.pos, tls_dir, TLS_submap, nearest_scan_idx);
                if (TLS_submap->size()) {
                    updateKdTree(TLS_submap, tls_position_ids, prev_nearest_idx, nearest_scan_idx);
                }
            }
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if (odom_mode == ODOM_MODE::LocToMap)
                    ROS_WARN("ikdtree should have been initialized ealier in odom_mode!");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                PointCloudXYZI::Ptr feats_down_body2(new PointCloudXYZI());
                *feats_down_body2 = *feats_down_body;
                lidar_localizer.push(feats_down_body2, lidar_end_time, state_point);
                return false;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this downsampled scan!");
                return false;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
                    <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if (show_submap) {// If you need to see map point
                // But this can drastically slow down the program.
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
                publish_map(pubLaserCloudMap);
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            
            t3 = omp_get_wtime();
            if (odom_mode == ODOM_MODE::LocToMap) {
                Eigen::Vector4d timed_position;
                timed_position[0] = lidar_end_time;
                timed_position.segment(1, 3) = pos_lid;
                bool inbound = dist_checkup.check(timed_position);
                if (!inbound) {
                    ROS_WARN("The lidar pose is out of the TLS trajectory, abort the odometer.");
                    return true;
                }
            } else {
                map_incremental();
                PointCloudXYZI::Ptr feats_down_body2(new PointCloudXYZI());
                *feats_down_body2 = *feats_down_body;
                lidar_localizer.push(feats_down_body2, lidar_end_time, state_point);
                if (lidar_localizer.shouldAbort()) {
                    return true;
                }
            }
            t5 = omp_get_wtime();

            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf(("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f "
                       "map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n"), 
                       t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
            }

            if (output_ref_frame == "lidar") { // in case we are estimating the extrinsics.
                B_T_S = Pose3d(state_point.offset_R_L_I, state_point.offset_T_L_I);
            } // else pass
            dump_lio_state_to_log(lidar_end_time, state_point, fp, B_T_S);
        }
        return false;
    }

void saveMap() {
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(state_log_dir + "/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to " << all_points_dir <<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();
    if (fp)
      fclose(fp);
    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = state_log_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    // if (pcd_save_en && pcd_save_interval == 1) {
    //     const std::string pcd_dir = state_log_dir + "/pcd";
    //     std::vector<std::pair<std::string, std::string>> pcd_pose_pairs;
    //     pcd_pose_pairs.emplace_back(pcd_dir, pos_log_filename);
    //     aggregatePointCloudsWithPose(pcd_pose_pairs, state_log_dir, 0.0, 0.0);
    // }
}

void saveImu(const std::string& bagfile) {
    fs::path imu_csv = fs::path(state_log_dir) / "imu.csv";
    if (output_ref_frame == "imu") {
        extractAndCompensateImu(bagfile, pos_log_filename, imu_topic, imu_csv.string(), 
                                gyro_scale, accelerometer_scale);
    } else if (output_ref_frame == "lidar") {
        Eigen::Isometry3d B_T_L = Eigen::Isometry3d::Identity();
        B_T_L.linear() = Lidar_R_wrt_IMU;   // 3x3 rotation matrix
        B_T_L.translation() = Lidar_T_wrt_IMU; // 3x1 translation vector
        extractAndConvertImu(bagfile, pos_log_filename, imu_topic, B_T_L, imu_csv.string(),
                             gyro_scale, accelerometer_scale);
    }
}
};

int main(int argc, char** argv) {
    if (argc >= 4) {
        // Usage: rosrun fast_lio fastlio_mapping [config_file.yaml] [ros1.bag] [output_dir]
        std::string config_file = argv[1];
        std::string ros1_bagfile = argv[2];
        std::string output_dir = argv[3];
        fs::copy_file(config_file, fs::path(output_dir) / "lio_config.yaml", fs::copy_options::overwrite_existing);
    }

    Eigen::setNbThreads(1);
    setenv("OMP_NUM_THREADS","1",1);
    setenv("MKL_NUM_THREADS","1",1);
    setenv("OPENBLAS_NUM_THREADS","1",1);

    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    LIOdometer odometer;
    int res = odometer.initializeSystem(nh);
    if (!res) return 0;

    auto t0 = std::chrono::high_resolution_clock::now();

    bool abort = false;
    ros::Rate rate(500);
    ros::Publisher lidar_publisher;
    if (odometer.is_livox_custom_msg()) {
        lidar_publisher = nh.advertise<fast_lio::CustomMsg>(lid_topic, 100);
    } else {
        lidar_publisher = nh.advertise<sensor_msgs::PointCloud2>(lid_topic, 100);
    }

    ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>(imu_topic, 5000);
    rosbag::Bag bag(bagfile, rosbag::bagmode::Read);
    if (bag.isOpen()) {
        ROS_INFO("Successfully opened bag %s", bagfile.c_str());
    } else {
        ROS_ERROR("Failed to open bag %s", bagfile.c_str());
        return 1;
    }

    std::vector<std::string> topics;
    topics.push_back(lid_topic);
    topics.push_back(imu_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ros::Duration epsi(0.05);
    ros::Time min_time_ros = view.getBeginTime();
    if (msg_start_time_ros > ros::Time())
        min_time_ros = msg_start_time_ros;
    ros::Time max_time_ros = view.getEndTime();
    if (msg_end_time_ros > ros::Time())
        max_time_ros = msg_end_time_ros;
    int lid_cnt = 0;
    int imu_cnt = 0;
    foreach(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == lid_topic) {
            if (odometer.is_livox_custom_msg()) {
                fast_lio::CustomMsg::ConstPtr lidar_msg = m.instantiate<fast_lio::CustomMsg>();
                if (lidar_msg->header.stamp < min_time_ros || lidar_msg->header.stamp > max_time_ros) {
                    continue;
                }
                lidar_publisher.publish(lidar_msg);
            } else {
                sensor_msgs::PointCloud2::ConstPtr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
                if (lidar_msg->header.stamp < min_time_ros || lidar_msg->header.stamp > max_time_ros) {
                    continue;
                }
                // checking if a earlier message arrives is not possible with ROS1 C++ implementation as
                // it sorts the msgs at a topic when reading the topic.
                // So we have to use some hacky way like below to discard out-of-order (and usually bad) msgs.
                // The out-of-order msg timestamp is actually found by reading the topic msgs in ROS1 bag python API.
                // if (lidar_msg->header.stamp == ros::Time(1699177125, 344055000)) {
                //     ROS_INFO_STREAM("Skip frame at " << lidar_msg->header.stamp);
                //     continue;
                // }
                lidar_publisher.publish(lidar_msg);
            }
            ++lid_cnt;
            abort = odometer.spinOnce();
        } else if (m.getTopic() == imu_topic) {
            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            if (imu_msg->header.stamp < min_time_ros || imu_msg->header.stamp > max_time_ros) {
                continue;
            }
            ++imu_cnt;
            imu_publisher.publish(imu_msg);
        }
        if (abort) break;
    }
    bag.close();
    odometer.saveMap();
    odometer.saveImu(bagfile);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
    std::cout << "LIO took " << dt.count() / 1000.f << " sec" << std::endl;

    ROS_INFO("Finished processing bag file %s, lidar msgs %d, imu msgs %d", bagfile.c_str(), lid_cnt, imu_cnt);
    return 0;
}
