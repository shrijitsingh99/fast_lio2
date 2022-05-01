#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "point_types.hpp"
#include "ouster_handler.hpp"
#include "velodyne_handler.hpp"

using namespace std;

#define IS_VALID(a)  ((abs(a) > 1e8) ? true : false)

enum LID_TYPE {AVIA = 1, VELO16, OUST64}; //{1, 2, 3}
enum TIME_UNIT {SEC = 0, MS = 1, US = 2, NS = 3};

class Preprocess
{
public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess(std::weak_ptr<ros::NodeHandle> parent);
  ~Preprocess();

  PointCloudXYZI::Ptr process(const sensor_msgs::PointCloud2::ConstPtr & msg);

  ros::Publisher pub_full, pub_surf, pub_corn;

private:
  std::unique_ptr<LidarHandler> lidar_handler_;

  int lidar_type_;

  // void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void pub_func(PointCloudXYZI & pl, const ros::Time & ct);
};
