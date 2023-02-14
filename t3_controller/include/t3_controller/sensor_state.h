#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H

#include <vector>
#include <algorithm>

#include "t3_msgs/BoundingBox.h"
#include "t3_msgs/object_data.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

namespace sensor
{

struct Sign
{
  bool online;
  std::vector<t3_msgs::BoundingBox> bboxes;

  Sign() : online(false), bboxes(std::vector<t3_msgs::BoundingBox>{})
  {
  }

  void reduce(const t3_msgs::object_data::ConstPtr& msg)
  {
    if (!online)
      online = true;

    // Reset
    bboxes.clear();

    if (sizeof(msg->bounding_boxes))
    {
      for (const auto& bbox : msg->bounding_boxes)
      {
        bboxes.emplace_back(bbox);
      }
    }
  }
};

struct Lidar
{
  pcl::PointCloud<pcl::PointXY> cloud;

  Lidar() : cloud(pcl::PointCloud<pcl::PointXY>{})
  {
  }

  void reduce(const pcl::PointCloud<pcl::PointXY>::ConstPtr& msg)
  {
    // Reset
    cloud.clear();

    if (sizeof(msg->points))
    {
      for (const auto& p : msg->points)
      {
        cloud.emplace_back(p);
      }
    }
  }
};

// Combine states
struct State
{
  Sign sign;
  Lidar lidar;
};

}  // namespace sensor

#endif
