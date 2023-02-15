#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H

#include <vector>
#include <numeric>


#include "t3_msgs/object_data.h"
//#include "t3_msgs/lidar_array.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>


namespace sensor_state
{

constexpr int WIDTH = 640;
constexpr int SMA_NUM = 10;

// Define states
struct BoundingBox
{
  int id;
  int xmin;
  int ymin;
  int xmax;
  int ymax;
  float probability;

  BoundingBox()
    : id(-1)
   , xmin(0)
   , ymin(0)
   , xmax(0)
   , ymax(0)
   ,probability(0)
   {};
};

struct Object
{
  std::vector<BoundingBox> boundingBoxes;
  void reduce(const t3_msgs::object_data::ConstPtr& msg)
  {
    boundingBoxes.clear();
    auto boxes = msg->bounding_boxes;
    for(auto& box : boxes){
      BoundingBox boundingBox = BoundingBox();
      boundingBox.id = box.id;
      boundingBox.xmin = box.xmin;
      boundingBox.ymin = box.ymin;
      boundingBox.xmax = box.xmax;
      boundingBox.ymax = box.ymax;
      boundingBox.probability = box.probability;
      boundingBoxes.emplace_back(boundingBox);
    }
  };
};

struct LidarPoint
{
  float x;
  float y;

  LidarPoint()
    : x(0)
   , y(0)
   {};
  
};

struct LidarPoints
{
  std::vector<LidarPoint> lidarPoints;
  void reduce(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
  {
    lidarPoints.clear();

    auto pts = msg->points;
    for(auto& point : pts){
      LidarPoint lidar_point = LidarPoint();
      lidar_point.x = point.x;
      lidar_point.y = point.y;
      lidarPoints.emplace_back(lidar_point);
    }
  };
};


// Combine states
struct State
{
  Object object;
  LidarPoints lidar_points;
};

}  // namespace sensor

#endif
