// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "laser_geometry/laser_geometry.h"

namespace sensor
{
const std::string NAME = "lidar";
const std::string SUB_TOPIC = "scan";
const std::string PUB_TOPIC = "lidar_data";
constexpr double LASER_PROJECTION_RANGE_CUTOFF = -1.;
constexpr int LASER_PROJECTION_OPTIONS =
    laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance;
constexpr float CLOUD_MIN_X = -2.f;
constexpr float CLOUD_MIN_Y = -.5f;
constexpr float CLOUD_MAX_Y = .5f;

class Lidar
{
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  laser_geometry::LaserProjection projector;

public:
  bool enable_debug;

  Lidar()
  {
    node.param<bool>("sensor_lidar_enable_debug", enable_debug, true);
    sub = node.subscribe(SUB_TOPIC, 1, &Lidar::callback, this);
    pub = node.advertise<pcl::PointCloud<pcl::PointXY>>(PUB_TOPIC, 1);
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  pcl::PointCloud<pcl::PointXY> process(pcl::PointCloud<pcl::PointXY>& cloud_data);
  void publish(pcl::PointCloud<pcl::PointXY>& msg);
};

void Lidar::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  try
  {
    // Convert from sensor_msgs::LaserScan to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 sensor_data;
    projector.projectLaser(*msg, sensor_data, LASER_PROJECTION_RANGE_CUTOFF, LASER_PROJECTION_OPTIONS);

    // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXY>
    pcl::PointCloud<pcl::PointXY> cloud_data;
    pcl::fromROSMsg(sensor_data, cloud_data);

    pcl::PointCloud<pcl::PointXY> msg = process(cloud_data);
    publish(msg);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("callback exception: %s", e.what());
    return;
  }
}

pcl::PointCloud<pcl::PointXY> Lidar::process(pcl::PointCloud<pcl::PointXY>& cloud_data)
{
  pcl::PointCloud<pcl::PointXY> msg;

  // cloud_data's coordinates:
  // +x:= back | -x:= front
  // +y:= right | -y:= left
  // +z:= top | -z: bottom
  for (const auto& p : cloud_data.points)
  {
    if (p.y >= CLOUD_MIN_Y && p.y <= CLOUD_MAX_Y && p.x >= CLOUD_MIN_X)
      msg.points.emplace(p);
  }
  return msg;
}

void Lidar::publish(pcl::PointCloud<pcl::PointXY>& msg)
{
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = NAME;
  msg.height = 1;                 // for unorganized dataset
  msg.width = msg.points.size();  // for unorganized dataset
  pub.publish(msg);
  if (enable_debug)
    ROS_INFO("Pointcloud length: %d", msg.points.size());
}

}  // namespace sensor

int main(int argc, char** argv)
{
  ros::init(argc, argv, sensor::NAME);
  sensor::Lidar lidar;
  ROS_INFO("%s is ONLINE", sensor::NAME.c_str());

  ros::spin();  // forever

  return 0;
}
