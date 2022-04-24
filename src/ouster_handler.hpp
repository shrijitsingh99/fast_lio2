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
    if (cfg_.feature_enabled) {
      for (int i = 0; i < cfg_.N_SCANS; i++) {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }

      for (uint i = 0; i < plsize; i++) {
        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y *
          pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
        if (range < (cfg_.blind * cfg_.blind)) {continue;}
        Eigen::Vector3d pt_vec;
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
        if (yaw_angle >= 180.0) {
          yaw_angle -= 360.0;
        }
        if (yaw_angle <= -180.0) {
          yaw_angle += 360.0;
        }

        added_pt.curvature = pl_orig.points[i].t * cfg_.time_unit_scale;
        if (pl_orig.points[i].ring < cfg_.N_SCANS) {
          pl_buff[pl_orig.points[i].ring].push_back(added_pt);
        }
      }

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
        feature_extraction.give_feature(pl, types);
      }
    } else {
      double time_stamp = msg->header.stamp.toSec();
      // cout << "===================================" << endl;
      // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
      for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % cfg_.point_filter_num != 0) {continue;}

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y *
          pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (cfg_.blind * cfg_.blind)) {continue;}

        Eigen::Vector3d pt_vec;
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = pl_orig.points[i].t * cfg_.time_unit_scale; // curvature unit: ms

        pl_surf.points.push_back(added_pt);
      }
    }
    // pub_func(pl_surf, pub_full, msg->header.stamp);
    // pub_func(pl_surf, pub_corn, msg->header.stamp);
    return pl_surf;
  }
};
