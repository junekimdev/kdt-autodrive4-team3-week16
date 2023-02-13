#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H

#include <vector>
#include <numeric>


#include "t3_msgs/object_data.h"
#include "t3_msgs/lidar_array.h"

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
  float z;

  LidarPoint()
    : x(0)
   , y(0)
   , z(0)
   {};
  
};

struct LidarPoints
{
  std::vector<LidarPoint> LidarPoints;
  void reduce(const t3_msgs::lidar_array::ConstPtr& msg)
  {
    LidarPoints.clear();
    auto points = msg->lidar_points;
    for(auto& point : points){
      LidarPoint lidar_point = Lidarpoint();
      LidarPoint.x = point.x;
      LidarPoint.y = point.y;
      LidarPoint.z = point.z;
      LidarPoints.emplace_back(LidarPoint);
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
