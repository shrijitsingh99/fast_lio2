#pragma once

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "point_types.hpp"

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELO16, OUST64}; //{1, 2, 3}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};

class FeatureExtraction{
  public:
  FeatureExtraction(PointCloudXYZI &pl_corn_, PointCloudXYZI &pl_surf_, int &point_filter_num_, double &blind_, int &group_size_, double &jump_up_limit_, double &jump_down_limit_, double &cos160_, double &smallp_intersect_, double &smallp_ratio_, double &disA_, double &disB_, double &inf_bound_, double &limit_maxmin_, double &p2l_ratio_, double &vx_, double &vy_, double &vz_, double &edgea_, double &edgeb_);
  ~FeatureExtraction();

  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  private:
  // Variables needed from other class
  PointCloudXYZI &pl_corn, &pl_surf;
  int &point_filter_num;
  double &blind;
  int &group_size;
  double &jump_up_limit, &jump_down_limit;
  double &cos160;
  double &smallp_intersect;
  double &smallp_ratio;
  double &disA,&disB,&inf_bound;
  double &limit_maxmin;
  double &p2l_ratio;
  double &vx,&vy,&vz;
  double &edgea,&edgeb;
};

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess(std::weak_ptr<ros::NodeHandle> parent);
  ~Preprocess();
  
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool feature_enabled, given_offset_time;
  ros::Publisher pub_full, pub_surf, pub_corn;
  
  private:
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;

  FeatureExtraction feature_extraction;
};
