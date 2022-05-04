#include "preprocess.hpp"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess(std::weak_ptr<ros::NodeHandle> parent)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  LidarConfig lidar_cfg;
  int time_unit;

  node->param<double>("preprocess/blind", lidar_cfg.blind, 0.01);
  node->param<int>("preprocess/lidar_type", lidar_type_, VELO16);
  node->param<int>("preprocess/scan_line", lidar_cfg.N_SCANS, 16);
  node->param<int>("preprocess/timestamp_unit", time_unit, US);
  node->param<int>("preprocess/scan_rate", lidar_cfg.SCAN_RATE, 10);
  // TODO: (shrijitsingh99) Add preprocess namespace
  node->param<int>("point_filter_num", lidar_cfg.point_filter_num, 2);
  node->param<bool>("feature_extract_enable", lidar_cfg.feature_enabled, false);

  // TODO: (shrijitsingh99) Print out LiDAR type string
  ROS_INFO("Initalizing LiDAR of type %d", lidar_type_);

  switch (time_unit) {
    case SEC:
      lidar_cfg.time_unit_scale = 1.e3f;
      break;
    case MS:
      lidar_cfg.time_unit_scale = 1.f;
      break;
    case US:
      lidar_cfg.time_unit_scale = 1.e-3f;
      break;
    case NS:
      lidar_cfg.time_unit_scale = 1.e-6f;
      break;
    default:
      lidar_cfg.time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type_) {
    case OUST64:
      lidar_handler_ = std::make_unique<OusterHandler>(lidar_cfg);
      break;
    case VELO16:
      lidar_handler_ = std::make_unique<VelodyneHandler>(lidar_cfg);
      break;
    default:
      throw std::runtime_error{"Invalid LiDAR type"};
      break;
  }

}

Preprocess::~Preprocess() {}

PointCloudXYZI::Ptr Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  auto pcl_out = pcl::make_shared<PointCloudXYZI>();
  lidar_handler_->process(msg);
  *pcl_out = lidar_handler_->process(msg);
  return pcl_out;
}

void Preprocess::pub_func(PointCloudXYZI & pl, const ros::Time & ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}
