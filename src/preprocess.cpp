#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess(std::weak_ptr<ros::NodeHandle> parent)
  :feature_enabled(0), lidar_type(VELO16), blind(0.01), point_filter_num(1),
  feature_extraction(pl_corn, pl_surf, point_filter_num, blind, vx, vy, vz)
{
  N_SCANS   = 6;
  SCAN_RATE = 10;
  limit_maxmid =6.25;
  limit_midmin =6.25;
  given_offset_time = false;
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;
  
  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;
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
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;
      if(pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      feature_extraction.give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end  = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    if(feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }
      
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        int layer  = pl_orig.points[i].ring;
        if (layer >= N_SCANS) continue;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // units: ms

        if (!given_offset_time)
        {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        pl_buff[layer].points.push_back(added_pt);
      }

      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) continue;
        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        feature_extraction.give_feature(pl, types);
      }
    }
    else
    {
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

        if (!given_offset_time)
        {
          int layer = pl_orig.points[i].ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        if (i % point_filter_num == 0)
        {
          if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
          {
            pl_surf.points.push_back(added_pt);
          }
        }
      }
    }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}
