#pragma once

#include "lidar_handler.hpp"

class VelodyneHandler : public LidarHandler
{
public:
  VelodyneHandler(LidarConfig cfg)
  : LidarHandler(cfg)
  {
  }

  PointCloudXYZI process(const sensor_msgs::PointCloud2::ConstPtr & msg) override
  {
    pl_surf.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) {
      return pl_surf;
    }
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * cfg_.SCAN_RATE;     // scan angular velocity
    std::vector<bool> is_first(cfg_.N_SCANS, true);
    std::vector<double> yaw_fp(cfg_.N_SCANS, 0.0);       // yaw of first scan point
    std::vector<float> yaw_last(cfg_.N_SCANS, 0.0);      // yaw of last scan point
    std::vector<float> time_last(cfg_.N_SCANS, 0.0);     // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
      given_offset_time_ = true;
    } else {
      given_offset_time_ = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--) {
        if (pl_orig.points[i].ring == layer_first) {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    if (cfg_.feature_enabled) {
      for (int i = 0; i < cfg_.N_SCANS; i++) {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }

      for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        int layer = pl_orig.points[i].ring;
        if (layer >= cfg_.N_SCANS) {
          continue;
        }
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * cfg_.time_unit_scale;         // units: ms

        if (!given_offset_time_) {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
          if (is_first[layer]) {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
            yaw_fp[layer] = yaw_angle;
            is_first[layer] = false;
            added_pt.curvature = 0.0;
            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
            continue;
          }

          if (yaw_angle <= yaw_fp[layer]) {
            added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
          } else {
            added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer]) {
            added_pt.curvature += 360.0 / omega_l;
          }

          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
        }

        pl_buff[layer].points.push_back(added_pt);
      }

      for (int j = 0; j < cfg_.N_SCANS; j++) {
        PointCloudXYZI & pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) {
          continue;
        }
        std::vector<orgtype> & types = typess[j];
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

        // TODO: (shrijitsingh99) Enable after integrating feature extraction
        // give_feature(pl, types);
      }
    } else {
      for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * cfg_.time_unit_scale;         // curvature unit: ms // cout<<added_pt.curvature<<endl;

        if (!given_offset_time_) {
          int layer = pl_orig.points[i].ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer]) {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
            yaw_fp[layer] = yaw_angle;
            is_first[layer] = false;
            added_pt.curvature = 0.0;
            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
            continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer]) {
            added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
          } else {
            added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer]) {
            added_pt.curvature += 360.0 / omega_l;
          }

          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
        }

        if (i % cfg_.point_filter_num == 0) {
          if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
            (cfg_.blind * cfg_.blind))
          {
            pl_surf.points.push_back(added_pt);
          }
        }
      }
    }

    return pl_surf;
  }

private:
  bool given_offset_time_ = false;

};
