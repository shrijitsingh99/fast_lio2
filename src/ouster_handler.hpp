#pragma once

#include "lidar_handler.hpp"

class OusterHandler : public LidarHandler
{
public:
  OusterHandler(LidarConfig cfg)
  : LidarHandler(cfg)
  {
  }

  PointCloudXYZI process(const sensor_msgs::PointCloud2::ConstPtr & msg) override
  {
    pl_surf.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    pl_surf.reserve(plsize);

    // Set up pl_buff for feature extraction
    if (cfg_.feature_enabled) {
      // Clear and setup buffers for each layer
      for (int i = 0; i < cfg_.N_SCANS; i++) {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }
    }
    // Set up buffers by going through all points in original data
    for (uint i = 0; i < plsize; i++) {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y *
        pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (cfg_.blind * cfg_.blind)) {continue;}
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * cfg_.time_unit_scale;
      if (cfg_.feature_enabled) {
        if (pl_orig.points[i].ring < cfg_.N_SCANS) {
          pl_buff[pl_orig.points[i].ring].push_back(added_pt);
        }
      } else{
        if (i % cfg_.point_filter_num == 0){
          pl_surf.points.push_back(added_pt);
        }
      }
    }
    // Run feature extraction for each layer
    if (cfg_.feature_enabled) {
      for (int j = 0; j < cfg_.N_SCANS; j++) {
        PointCloudXYZI & pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) {
          continue;
        }
        vector<orgtype> & types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++) {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(
          pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        // Do feature extraction to update pl_surf
        feature_extraction.give_feature(pl, types);
      }
    }
    return pl_surf;
  }
};
