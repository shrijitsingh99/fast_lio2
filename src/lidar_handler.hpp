#pragma once

#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "point_types.hpp"
#include "feature_extraction.h"


struct LidarConfig
{
  float time_unit_scale;
  double blind;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  bool feature_enabled;
};

class LidarHandler
{
public:
  LidarHandler(LidarConfig cfg)
  : cfg_(cfg) {}
  virtual ~LidarHandler() = default;
  virtual PointCloudXYZI process(const sensor_msgs::PointCloud2::ConstPtr & msg) {}

protected:
  LidarConfig cfg_;

  PointCloudXYZI pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  std::vector<orgtype> typess[128]; //maximum 128 line lidar

  double vx, vy, vz;

};
