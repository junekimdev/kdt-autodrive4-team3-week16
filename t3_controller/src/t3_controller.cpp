#include <string>

#include "ros/console.h"
#include "ros/ros.h"

#include "t3_controller/sensor_state.h"

namespace control
{
const std::string NAME = "controller";
const std::string SUB_TOPIC_OBJECT = "object_data";
const std::string SUB_TOPIC_LIDAR = "lidar_data";

class Controller
{
  ros::NodeHandle node;
  ros::Subscriber sub_object;
  ros::Subscriber sub_lidar;
  sensor::State sensor_state;

public:
  bool enable_debug;

  Controller()
  {
    node.param<bool>("controller_enable_debug", enable_debug, true);
    sub_object = node.subscribe(SUB_TOPIC_OBJECT, 1, &Controller::callbackObject, this);
    sub_lidar = node.subscribe(SUB_TOPIC_LIDAR, 1, &Controller::callbackLidar, this);
  }

  void callbackObject(const t3_msgs::object_data::ConstPtr& msg);
  void callbackLidar(const pcl::PointCloud<pcl::PointXY>::ConstPtr& msg);
  void control();
};

void Controller::callbackObject(const t3_msgs::object_data::ConstPtr& msg)
{
  sensor_state.object.reduce(msg);
}
void Controller::callbackLidar(const pcl::PointCloud<pcl::PointXY>::ConstPtr& msg)
{
  sensor_state.lidar.reduce(msg);
}

void Controller::control()
{
  // TODO
}

}  // namespace control

int main(int argc, char** argv)
{
  ros::init(argc, argv, control::NAME);
  control::Controller controller;
  ROS_INFO("%s is ONLINE", control::NAME.c_str());

  while (ros::ok())
  {
    ros::spinOnce();
    controller.control();
  }

  return 0;
}
