#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H

#include <vector>
#include <numeric>

#include "t3_msgs/object_data.h"
#include "sensor_msgs/LaserScan.h"

namespace sensor
{

// Define states
struct BoundingBox
{
  int id;
  int xmin;
  int ymin;
  int xmax;
  int ymax;
  float probability;

  BoundingBox() : id(-1), xmin(0), ymin(0), xmax(0), ymax(0), probability(0){};
};

struct Object
{
  std::vector<BoundingBox> boundingBoxes;
  void reduce(const t3_msgs::object_data::ConstPtr& msg)
  {
    boundingBoxes.clear();
    auto boxes = msg->bounding_boxes;
    for (auto& box : boxes)
    {
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

struct Lidar
{
  std::vector<float32_t> ranges;

  Lidar() : ranges(std::vector<float32_t>{}){};
  void reduce(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    ranges = std::vector<float32_t>{ std::begin(msg->ranges), std::end(msg->ranges) };
  };
};

// Combine states
struct State
{
  Object object;
  Lidar lidar;
};

}  // namespace sensor

#endif
