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
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
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
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.hpp"
#include <ikd-Tree/ikd_Tree.h>
#include <pcl/common/transforms.h>  
#include <pcl/kdtree/kdtree_flann.h>
#include <map>
#include <unordered_map>
#include <queue>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

/*** Map Param Variables ***/
float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
/**************************/

mutex mtx_buffer;
condition_variable sig_buffer;

/*** File Path Variables ***/
string root_dir = ROOT_DIR;
string map_file_name, lid_topic, imu_topic, keyframe_topic, keyframe_id_topic;;
/**************************/

/*** Parameter Variables ***/
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
bool    recontruct_kdtree = false;
bool    update_state = false;
int     update_frequency = 100;

// visualize
bool visualize_map = false;
/**************************/

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
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_mat;
vect3 pos_lid;

nav_msgs::Path path, path_updated;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose, msg_body_pose_updated;

shared_ptr<Preprocess> p_pre;
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// TODO rename for consistency
/*** Maintain keyframe mechanism ***/
// Cache the historical lidar frames, and decide which frame is the keyframe according to the subscribed seq
vector<PointCloudXYZI::Ptr> cloudKeyFrames; // store the historical keyframe point cloud
queue<pair<uint32_t, PointCloudXYZI::Ptr> > cloudBuff; // Cache partial historical lidar frames for extracting keyframe point clouds
vector<uint32_t> idKeyFrames; // id of keyframes
queue<uint32_t> idKeyFramesBuff; // id buffer of keyframes
nav_msgs::Path pathKeyFrames; // keyframes
uint32_t data_seq; // serial number of data
uint32_t lastKeyFramesId; // ID of the odometer corresponding to the latest key frame
geometry_msgs::Pose lastKeyFramesPose; // Pose of the latest keyframe (world to imu)
vector<geometry_msgs::Pose> odoms;
/*** Maintain submap ***/
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<pcl::PointXYZ>()); // kdtree for surrounding keyframe poses
pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
/**************************/

/*** Map variables ***/
BoxPointType LocalMap_Points;
bool is_local_map_initialized = false;
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI()); // global map
/**************************/

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline float pointDistance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

inline void logAlgState(FILE *fp)  
{
    V3D rot_ang(Log(state_mat.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_mat.pos(0), state_mat.pos(1), state_mat.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_mat.vel(0), state_mat.vel(1), state_mat.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_mat.bg(0), state_mat.bg(1), state_mat.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_mat.ba(0), state_mat.ba(1), state_mat.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_mat.grav[0], state_mat.grav[1], state_mat.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

/*** Transform Functions ***/
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_world(state_mat.rot * (state_mat.lidar_to_imu_rot*p_body + state_mat.lidar_to_imu_trans) + state_mat.pos);

    po->x = p_world(0);
    po->y = p_world(1);
    po->z = p_world(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_world(state_mat.rot * (state_mat.lidar_to_imu_rot*p_body + state_mat.lidar_to_imu_trans) + state_mat.pos);

    po[0] = p_world(0);
    po[1] = p_world(1);
    po[2] = p_world(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_world(state_mat.rot * (state_mat.lidar_to_imu_rot*p_body + state_mat.lidar_to_imu_trans) + state_mat.pos);

    po->x = p_world(0);
    po->y = p_world(1);
    po->z = p_world(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_mat.lidar_to_imu_rot*p_body_lidar + state_mat.lidar_to_imu_trans);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}
/**************************/

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    V3D pos_LiD = pos_lid;

    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);

    if (!is_local_map_initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        is_local_map_initialized = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        // Calculate between of lidar pos from min and max vertex
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            need_move = true;
        }
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

void keyframe_cbk(const nav_msgs::Path::ConstPtr &msg_keyframes){
    // update keyframes
    pathKeyFrames = *msg_keyframes;
}

void keyframe_id_cbk(const std_msgs::Header::ConstPtr &msg_keyframe_id){
    // Add the subscribed key frame id to idKeyFramesBuff first
    idKeyFramesBuff.push(msg_keyframe_id->seq);
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

    PointCloudXYZI::Ptr ptr = p_pre->process(msg);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

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

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    laserCloudmsg.header.seq = data_seq;
    pubLaserCloudFull_body.publish(laserCloudmsg);
    cloudBuff.push( pair<int, PointCloudXYZI::Ptr>(data_seq ,laserCloudIMUBody) ); // Cache all point clouds sent to the backend
    publish_count -= PUBFRAME_PERIOD;
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
    out.pose.position.x = state_mat.pos(0);
    out.pose.position.y = state_mat.pos(1);
    out.pose.position.z = state_mat.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.header.seq = data_seq;
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);

    odoms.push_back(odomAftMapped.pose.pose);

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

    /*** if path is too large, the rviz will crash ***/
    static int jjj = 0;
    jjj++;
    // if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_world(s.rot * (s.lidar_to_imu_rot*p_body + s.lidar_to_imu_trans) + s.pos);
        point_world.x = p_world(0);
        point_world.y = p_world(1);
        point_world.z = p_world(2);
        point_world.intensity = point_body.intensity;

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
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
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
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.lidar_to_imu_rot * point_this_be + s.lidar_to_imu_trans;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.lidar_to_imu_rot.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    // Publishing params
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();

    p_pre = std::make_shared<Preprocess>(nh);
    nh->param<bool>("publish/path_en",path_en, true);
    nh->param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh->param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh->param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh->param<bool>("recontruct_kdtree",recontruct_kdtree,false);
    nh->param<bool>("update_state",update_state,false);
    nh->param<int>("update_frequency",update_frequency,100);
    nh->param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh->param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh->param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh->param<bool>("common/time_sync_en", time_sync_en, false);
    nh->param<string>("common/keyframe_topic", keyframe_topic,"/aft_pgo_path");
    nh->param<string>("common/keyframe_id_topic", keyframe_id_topic,"/key_frames_ids");
    nh->param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh->param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh->param<double>("filter_size_map",filter_size_map_min,0.5);
    nh->param<double>("cube_side_length",cube_len,200);
    nh->param<float>("mapping/det_range",DET_RANGE,300.f);
    nh->param<double>("mapping/fov_degree",fov_deg,180);
    nh->param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh->param<double>("mapping/acc_cov",acc_cov,0.1);
    nh->param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh->param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh->param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh->param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh->param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh->param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh->param<string>("pcd_save/map_file_name",map_file_name,"scans.pcd");
    nh->param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh->param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh->param<bool>("visualize_map", visualize_map, false);
    path_updated.header.stamp    = ros::Time::now();
    path_updated.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = min(179.9, (fov_deg + 10.0)); //(fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    downSizeFilterSurroundingKeyPoses.setLeafSize(0.2,0.2,0.2);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = nh->subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh->subscribe(imu_topic, 200000, imu_cbk);
    ros::Subscriber sub_keyframes = nh->subscribe(keyframe_topic, 10, keyframe_cbk);
    ros::Subscriber sub_keyframes_id = nh->subscribe(keyframe_id_topic, 10, keyframe_id_cbk);
    ros::Publisher pubLaserCloudFull = nh->advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh->advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh->advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh->advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubKeyFramesMap = nh->advertise<sensor_msgs::PointCloud2>
            ("/Keyframes_map", 100000);
    ros::Publisher pubOdomAftMapped = nh->advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    ros::Publisher pubPath          = nh->advertise<nav_msgs::Path> 
            ("/path", 100000);
    ros::Publisher pubPath_updated          = nh->advertise<nav_msgs::Path>
            ("/path_updated", 100000);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    uint32_t count = 1;
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        while( !cloudBuff.empty() && !idKeyFramesBuff.empty() ){
            while( idKeyFramesBuff.front() > cloudBuff.front().first )
            {
                cloudBuff.pop();
            }
            assert(idKeyFramesBuff.front() == cloudBuff.front().first);
            idKeyFrames.push_back(idKeyFramesBuff.front());
            cloudKeyFrames.push_back( cloudBuff.front().second );
            idKeyFramesBuff.pop();
            cloudBuff.pop();
        }
        assert(pathKeyFrames.poses.size() <= cloudKeyFrames.size() );

        // Record the information of the latest key frame
        if(pathKeyFrames.poses.size() >= 1){
            lastKeyFramesId = idKeyFrames[pathKeyFrames.poses.size() - 1];
            lastKeyFramesPose = pathKeyFrames.poses.back().pose;
        }
        // sync lidar and imu data currently stored in buffer
        if(sync_packages(Measures)) 
        {
            // record starting data
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }
            
            // time variables for evaluation
            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            // set up covariances and get the state
            p_imu->Process(Measures, kf, feats_undistort);
            state_mat = kf.get_x();
            pos_lid = state_mat.pos + state_mat.rot * state_mat.lidar_to_imu_trans;

            // if no features, skip the scan
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            // initialize EKF
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            // TODO: change to multi-threaded ikdtree update
            if(count % update_frequency == 0 ){
                count = 1;
                if(recontruct_kdtree && pathKeyFrames.poses.size() > 20){
                    /*** Sub-image composed of close keyframes ***/
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyPoses3D(new pcl::PointCloud<pcl::PointXYZ>());    // 历史关键帧位姿（位置）
                    pcl::PointCloud<pcl::PointXYZ>::Ptr surroundingKeyPoses(new pcl::PointCloud<pcl::PointXYZ>());    
                    pcl::PointCloud<pcl::PointXYZ>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<pcl::PointXYZ>());    

                    for(auto keyFramePose:pathKeyFrames.poses){
                        cloudKeyPoses3D->points.emplace_back(keyFramePose.pose.position.x, 
                                                                keyFramePose.pose.position.y, 
                                                                keyFramePose.pose.position.z);
                    }
                    double surroundingKeyframeSearchRadius = 5;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;
                    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); 
                    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
                    // Traverse the search results, pointSearchInd stores the index of the result under cloudKeyPoses3D
                    unordered_map<float, int> keyFramePoseMap;  // Take the x coordinate of pose as the key of the hash table
                    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
                    {
                        int id = pointSearchInd[i];
                        // Add to the adjacent keyframe pose set
                        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
                        keyFramePoseMap[cloudKeyPoses3D->points[id].x] = id;
                    }

                    // Downsample
                    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
                    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

                    // Add offset frames close to the current keyframe, these frames are reasonable to add
                    int numPoses = cloudKeyPoses3D->size();
                    int offset = 10;
                    for (int i = numPoses-1; i >= numPoses-1 - offset && i >= 0; --i)
                    {
                        surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
                        keyFramePoseMap[cloudKeyPoses3D->points[i].x] = i;
                    }

                    // Add the points corresponding to the adjacent keyframe sets to the local map as the local point cloud map matched by the scan-to-map
                    
                    PointCloudXYZI::Ptr keyFramesSubmap(new PointCloudXYZI());
                    // Traverse the current frame (actually take the nearest key frame to find its adjacent key frame set) the adjacent key frame sets in the space-time dimensionfor (int i = 0; i < (int)surroundingKeyPosesDS->size(); ++i)
                    for (int i = 0; i < (int)surroundingKeyPosesDS->size(); ++i)
                    {
                        ROS_INFO("surroundingKeyPosesDS->points[i].x: %f", surroundingKeyPosesDS->points[i].x);
                        ROS_INFO("surroundingKeyPosesDS->points[i].x: %d", keyFramePoseMap.size());
                        if(keyFramePoseMap.count(surroundingKeyPosesDS->points[i].x) == 0)
                            continue;

                        // If the distance exceeds the threshold, discard
                        if (pointDistance(surroundingKeyPosesDS->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)    // 丢弃那些满足时间临近，不满足空间临近的点
                            continue;

                        // Adjacent keyframe index
                        int thisKeyInd = keyFramePoseMap[ surroundingKeyPosesDS->points[i].x ];  // 以intensity作为红黑树的索引

                        PointCloudXYZI::Ptr keyframesTmp(new PointCloudXYZI());
                        Eigen::Isometry3d poseTmp;
                        assert(pathKeyFrames.poses.size() <= cloudKeyFrames.size() );   // 有可能id发过来了，但是节点还未更新
                        int keyFramesNum = pathKeyFrames.poses.size();

                        downSizeFilterMap.setInputCloud(cloudKeyFrames[thisKeyInd]);
                        downSizeFilterMap.filter(*keyframesTmp);
                        tf::poseMsgToEigen(pathKeyFrames.poses[thisKeyInd].pose,poseTmp);
                        pcl::transformPointCloud(*keyframesTmp , *keyframesTmp, poseTmp.matrix());
                        *keyFramesSubmap += *keyframesTmp;
                    }
                    downSizeFilterMap.setInputCloud(keyFramesSubmap);
                    downSizeFilterMap.filter(*keyFramesSubmap);

                    ikdtree.reconstruct(keyFramesSubmap->points);
                }

                // Update state
                if(update_state)
                {
                    state_ikfom state_updated = kf.get_x();
                    Eigen::Isometry3d lastPose(state_updated.rot);
                    lastPose.pretranslate(state_updated.pos);


                    Eigen::Isometry3d lastKeyFramesPoseEigen;       // Latest keyframe pose
                    tf::poseMsgToEigen(lastKeyFramesPose, lastKeyFramesPoseEigen);

                    Eigen::Isometry3d lastKeyFrameOdomPoseEigen;    // The pose of the odom corresponding to the latest key frame
                    tf::poseMsgToEigen(odoms[lastKeyFramesId], lastKeyFrameOdomPoseEigen);

                    // lastPose represents the transformation from the world coordinate system to the current coordinate system. The following two formulas are equivalent
                    lastPose = lastPose * lastKeyFrameOdomPoseEigen.inverse() * lastKeyFramesPoseEigen;

                    Eigen::Quaterniond lastPoseQuat( lastPose.rotation() );
                    Eigen::Vector3d lastPoseQuatPos( lastPose.translation() );
                    state_updated.rot = lastPoseQuat;
                    state_updated.pos = lastPoseQuatPos;
                    kf.change_x(state_updated);

                    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_updated = kf.get_P();  // 获取当前的状态估计的协方差矩阵
                    P_updated.setIdentity();
                    // Unsure if the state's covariance matrix be updated to a relatively small value?
                    P_updated(6,6) = P_updated(7,7) = P_updated(8,8) = 0.00001;
                    P_updated(9,9) = P_updated(10,10) = P_updated(11,11) = 0.00001;
                    P_updated(15,15) = P_updated(16,16) = P_updated(17,17) = 0.0001;
                    P_updated(18,18) = P_updated(19,19) = P_updated(20,20) = 0.001;
                    P_updated(21,21) = P_updated(22,22) = 0.00001; 
                    kf.change_P(P_updated);

                    msg_body_pose_updated.pose.position.x = state_updated.pos(0);
                    msg_body_pose_updated.pose.position.y = state_updated.pos(1);
                    msg_body_pose_updated.pose.position.z = state_updated.pos(2);
                    msg_body_pose_updated.pose.orientation.x = state_updated.rot.x();
                    msg_body_pose_updated.pose.orientation.y = state_updated.rot.y();
                    msg_body_pose_updated.pose.orientation.z = state_updated.rot.z();
                    msg_body_pose_updated.pose.orientation.w = state_updated.rot.w();
                    msg_body_pose_updated.header.stamp = ros::Time().fromSec(lidar_end_time);
                    msg_body_pose_updated.header.frame_id = "camera_init";

                    /*** if path is too large, the rviz will crash ***/
                    static int jjj = 0;
                    jjj++;
                    // if (jjj % 10 == 0)
                    {
                        path_updated.poses.push_back(msg_body_pose_updated);
                        pubPath_updated.publish(path_updated);
                    }
                }
            }
            ++count;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {    // only initialize if sufficient features remain
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    // convert feature points from body to world frame
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    // build bew ikd tree
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            
            kdtree_size_st = ikdtree.size();
            
            // int featsFromMapNum = ikdtree.validnum();
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_mat.lidar_to_imu_rot);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_mat.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_mat.lidar_to_imu_trans.transpose()<< " " << state_mat.vel.transpose() \
            <<" "<<state_mat.bg.transpose()<<" "<<state_mat.ba.transpose()<<" "<<state_mat.grav<< endl;

            if(visualize_map) // If you need to see map point
            {
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
            state_mat = kf.get_x();
            euler_cur = SO3ToEuler(state_mat.rot);
            pos_lid = state_mat.pos + state_mat.rot * state_mat.lidar_to_imu_trans;
            geoQuat.x = state_mat.rot.coeffs()[0];
            geoQuat.y = state_mat.rot.coeffs()[1];
            geoQuat.z = state_mat.rot.coeffs()[2];
            geoQuat.w = state_mat.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);


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
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_mat.lidar_to_imu_rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_mat.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_mat.lidar_to_imu_trans.transpose()<<" "<< state_mat.vel.transpose() \
                <<" "<<state_mat.bg.transpose()<<" "<<state_mat.ba.transpose()<<" "<<state_mat.grav<<" "<<feats_undistort->points.size()<<endl;
                logAlgState(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + map_file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current map saved to /PCD/" << map_file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
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

    return 0;
}
