#include <string>

#include "ros/console.h"
#include "ros/ros.h"
#include <vector>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <cmath>

// Messages
#include "t3_controller/control_state.h"
#include "t3_controller/sensor_state.h"




namespace control{
  const std::string NAME = "controller";
  const std::string SUB_TOPIC_OBJECT = "object_data";
  const std::string SUB_TOPIC_LIDAR = "lidar_data";
  //const std::string PUB_TOPIC = "xycar_motor";
  //constexpr int SPEED = 5;
  //constexpr float ANGLE_DIV = 1.f;

  control_state::State control_state_;
  sensor_state::State sensor_state_;

  struct TmpObject
  {
    int id;
    int xmin;
    int ymin;
    int xmax;
    int ymax;
    float probability;
    TmpObject(){};

 };


  class Controller
  {
    ros::NodeHandle node;
    ros::Subscriber sub_object;
    ros::Subscriber sub_lidar;
    ros::Publisher pub;
    int count =0;
    const float FOV_H = 1.60469;
    const float FOV_V = 1.31969;
    const int PINGPONG_BALL = 4;
    const int CENTER_POINT_X = 320;
    const int CENTER_POINT_Y = 240;
    const float FOCAL_LENGTH = 348.14820298;
    const double PI = 3.1415926;
    const float LiDAR_CENTER_X;
    const float LiDAR_CENTER_Y;


  public:
    bool enable_debug;

    Controller()
    {
      node.param<bool>("controller_enable_debug", enable_debug, false);
      this->sub_object = this->node.subscribe(SUB_TOPIC_OBJECT, 1, &Controller::callbackObject, this);
      this->sub_lidar = this->node.subscribe(SUB_TOPIC_LIDAR, 1, &Controller::callbackLidar, this);

    }

    void callbackObject(const t3_msgs::object_data::ConstPtr& msg);
    void callbackLidar(const t3_msgs::lidar_array::ConstPtr& msg);
    //void callbackTrafficLight(const t3_msgs::traffic_light_data::ConstPtr& msg);
    void control();
    float calculate_azimuth(control::TmpObject bbox);
    float Controller::get_width();
    float Controller::Clustering(float r, float theta);

    std::pair<float, float> Controller::Polar_to_Orthogonal(float r, float theta);
    std::pair<float, float> Controller::Orthogonal_to_Polar(control::TmpObject lidarpoint);
    std::pair<float,float> distance(float azimuth, control::TmpObject bbox);
  };

  float Controller::calculate_azimuth(control::TmpObject bbox){
  	float cx = (bbox.xmin+bbox.xmax)/2;
    float azimuth = (cx - 320)*tan(FOV_H/2)/320;
  	// float azimuth = (cx - 320)*FOV_H/640;
  	return azimuth;
  }
  
  std::pair<float,float> Controller::distance(float azimuth, control::TmpObject bbox){
    float depth = 0;
    float p;
    if (azimuth == 0){
      depth = PINGPONG_BALL * FOCAL_LENGTH /(bbox.xmax - bbox.xmin);
      p = 0.1 * (exp(depth-64) - exp(-depth+64))/(exp(depth-64) - exp(-depth+64)) + 0.82;
      return {0.0f, depth * p};
    }

    else{
      depth = PINGPONG_BALL * FOCAL_LENGTH /(bbox.xmax - bbox.xmin);
      p = 0.1 * (exp(depth-64) - exp(-depth+64))/(exp(depth-64) - exp(-depth+64)) + 0.82;
      // std::cout<< p<< std::endl;
      return {depth*azimuth, depth*p};
    }
  }
  // point_cloud clustering in Polar Coordinate
  std::pair<float, float> Controller::Orthogonal_to_Polar(control::TmpObject lidarpoint) {
    float r;
    float theta;
    if (lidarpoint.y == 0 && lidarpoint.x < 0) {
      theta = -PI/2;
      r = -lidarpoint.x;
    }
    else if (lidarpoint.y == 0 && lidarpoint.x > 0) {
      theta = PI/2;
      r = lidarpoint.x;
    }
    else {
      theta = atan(lidarpoint.y/lidarpoint.x);
      r = pow(pow(lidarpoint.x,2)+pow(lidarpoint.y, 2) , 0.5);
    }
    return {r, theta};
  }
  std::pair<float, float> Controller::Polar_to_Orthogonal(float r, float theta) {
    float x_ = r * cos(theta);
    float y_ = r * sin(theta);
    return {x_, y_};
  }

  float Controller::get_width(const vector<pair<float,float>> objectpoints) {
    float width = pow(pow(objectpoints[0].first-objectpoints[-1].first,2) + pow(objectpoints[0].second-objectpoints[-1].second,2),0.5);
    return width;
  }
  bool compare(pair<float,float> a, pair<float, float> b) {
    return a.second < b.second
  }


  float Controller::Clustering(const vector<pair<float, float>> lidarpoints) {
    // theta 기준 오름차순 정렬 //
    sort(lidarpoints.begin(), lidarpoints.end(), compare);
    /// 어려워서 일단 파이썬 형식으로...
    for (int i = 0; i < lidarpoints.size(); i++){
      /// 배열안 배열 어떻게?
      // 점 하나 사이의 theta가 얼마인지 확인 후 theta 차이로 묶기
      if theta_ - lidarpoints.second()

    }

  }




  // //// ///////////////////////////////// 
  void Controller::callbackObject(const t3_msgs::object_data::ConstPtr& msg)
  {
    sensor_state_.object.reduce(msg);
  }

  void Controller::callbackLidar(const t3_msgs::lidar_array::ConstPtr& msg)
  {
    sensor_state_.lidar.reduce(msg);
  }
  
  void Controller::control()
  {
    std::vector<control::TmpObject> tmpObject;

    if(sensor_state_.object.boundingBoxes.size()>0)
    {
      for(auto objectBox :sensor_state_.object.boundingBoxes){
        control::TmpObject tmp;
        tmp.id = objectBox.id;
        tmp.xmin = (int)objectBox.xmin*640/352;
        tmp.ymin = (int)objectBox.ymin*480/352;
        tmp.xmax = (int)objectBox.xmax*640/352;
        tmp.ymax = (int)objectBox.ymax*480/352;
        tmp.probability = objectBox.probability;
        float azimuth =  calculate_azimuth(tmp);
        auto depth =  distance(azimuth,tmp);
        std::cout << "[" <<depth.first << "," << depth.second << "] ";

        tmpObject.emplace_back(tmp);
      }
      std::cout << "" <<std::endl;
    }


    for(auto lidarpoint :sensor_state_.lidar_points.LidarPoints)
    {
      // points beyond xycar //
      control::TmpObject lidar;
      if(lidarpoint.x != 0 || lidarpoint.y != 0 ) {
        lidar.x = lidarpoint.x;
        lidar.y = lidarpoint.y;
      }
    }




  }
}
  // namespace control



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
