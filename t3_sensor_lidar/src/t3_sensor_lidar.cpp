// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_geometry/laser_geometry.h"

namespace sensor
{
const std::string NAME = "lidar";
const std::string SUB_TOPIC = "scan";
const std::string PUB_TOPIC = "lidar_data";
constexpr double LASER_PROJECTION_RANGE_CUTOFF = -1.;
constexpr int LASER_PROJECTION_OPTIONS =
    laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance;

class Lidar
{
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  laser_geometry::LaserProjection projector;

public:
  bool enable_debug;

  Lidar() : left_detected(false), right_detected(false), lpos(0), rpos(WIDTH - 1)
  {
    node.param<bool>("sensor_lidar_enable_debug", enable_debug, true);
    sub = node.subscribe(SUB_TOPIC, 1, &Lidar::callback, this);
    pub = node.advertise<sensor_msgs::PointCloud2>(PUB_TOPIC, 1);
    if (enable_debug)
      cv::namedWindow(NAME);
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  // void publish();
  // void process();
};

void Lidar::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  try
  {
    sensor_msgs::PointCloud2 output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = NAME;

    projector.projectLaser(*msg, output, LASER_PROJECTION_RANGE_CUTOFF, LASER_PROJECTION_OPTIONS);

    pub.publish(output);
    if (enable_debug)
      ROS_INFO("Pointcloud height: %d | width: %d", output.height, output.width);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("callback exception: %s", e.what());
    return;
  }
}

// void Lidar::publish(sensor_msgs::PointCloud2 msg)
// {
// }

// void Lidar::process()
// {
//   publish();
// }

}  // namespace sensor

int main(int argc, char** argv)
{
  ros::init(argc, argv, sensor::NAME);
  sensor::Lidar lidar;
  ROS_INFO("%s is ONLINE", sensor::NAME.c_str());

  ros::spin();  // forever

  return 0;
}
