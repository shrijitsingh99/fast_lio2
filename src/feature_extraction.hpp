#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include "point_types.hpp"

using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class FeatureExtraction{
  public:
  FeatureExtraction(PointCloudXYZI &pl_surf_, int &point_filter_num_, double &blind_, double &vx_, double &vy_, double &vz_);
  ~FeatureExtraction();

  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  void corner_reset(int plsize);

  private:
  // Variables needed from other class
  PointCloudXYZI &pl_surf;
  int &point_filter_num;
  double &blind;
  double &vx,&vy,&vz;

  // Local variables
  PointCloudXYZI pl_corn;
  int group_size;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double smallp_intersect;
  double smallp_ratio;
  double disA,disB,inf_bound;
  double limit_maxmin;
  double p2l_ratio;
  double edgea,edgeb;
};
