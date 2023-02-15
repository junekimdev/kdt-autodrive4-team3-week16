#include <string>
#include <vector>
#include <iostream>
#include <cmath>

#include <unistd.h>

#include "ros/console.h"
#include "ros/ros.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


// Messages
// #include "t3_controller/control_state.h"
#include "t3_controller/sensor_state.h"

#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"

namespace control
{
  const std::string NAME = "controller";
  const std::string SUB_TOPIC_OBJECT = "object_data";
  const std::string SUB_TOPIC_LIDAR = "output";
  const std::string SUB_TOPIC_CAM = "usb_cam/image_raw";
  //const std::string PUB_TOPIC = "xycar_motor";
  //constexpr int SPEED = 5;
  //constexpr float ANGLE_DIV = 1.f;
  sensor_state::State sensor_state_;

  // control_state::State control_state_;
  float m[3][4] = {{-3.84733185e+02 , 2.79117468e+02 ,-1.20071482e+01 ,-6.18678984e+01},
   {-2.46860484e+02 ,-5.28075672e+01 ,-3.52051629e+02 , 1.21485809e+01},
   {-9.80913534e-01, -1.92942836e-01 ,-2.41184835e-02 ,-4.76059423e-03}};


  // const cv::Mat PROJECT_MATRIX = cv::Mat(3,4, CV_64F, m);
 const cv::Mat PROJECT_MATRIX = cv::Mat_<float>({3, 4}, {-3.44775116e+02 ,  3.27100769e+02 , -1.43375635e+01,   9.36617661e+00,
  -2.52258957e+02 , -2.30760689e+01 , -3.51428497e+02 ,  3.69152336e+01,
  -9.97794509e-01 , -6.26784712e-02 , -2.18525622e-02  , 4.97312471e-02});
 const cv::Mat CAMERA_MATRIX = cv::Mat_<float>({3, 3}, {348.14820298,   0.,         323.82585721, 
  0.,         345.88660809, 260.82857638,
  0.,           0.,           1.       });

const cv::Mat DISTORTION_COEFFICIENTES_ERROR = cv::Mat_<float>({1,5}, {-0.3460703,   0.14435681,  0.00177496 , 0.00118325 ,-0.0321834});



  constexpr int WIDTH = 640;
  constexpr int HEIGHT = 480;

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

  struct TmpLidar
  {
    float x;
    float y;
    TmpLidar(){};
  };


  class Controller
  {
    ros::NodeHandle node;
    ros::Subscriber sub_object;
    ros::Subscriber sub_lidar;
    ros::Subscriber sub_cam;
    cv::Mat vFrame;
    cv::Mat dst;
    //ros::Publisher pub;
    // int count =0;
    const float FOV_H = 1.60469;
    const float FOV_V = 1.31969;
    const int PINGPONG_BALL = 4;
    const int CENTER_POINT_X = 320;
    const int CENTER_POINT_Y = 240;
    const float FOCAL_LENGTH = 348.14820298;
    const double PI = 3.1415926;

    std::vector<control::TmpObject> tmpObject;
    std::vector<control::TmpLidar> tmpLidar;
    std::vector<cv::Point> points;

  public:
    bool enable_debug;

    Controller()
    {
      node.param<bool>("controller_enable_debug", enable_debug, false);
      this->sub_object = this->node.subscribe(SUB_TOPIC_OBJECT, 1, &Controller::callbackObject, this);
      this->sub_lidar = this->node.subscribe(SUB_TOPIC_LIDAR, 1, &Controller::callbackLidar, this);
      this->sub_cam = node.subscribe(SUB_TOPIC_CAM, 1, &Controller::callbackCam, this);
    }

    void callbackObject(const t3_msgs::object_data::ConstPtr& msg);
    void callbackLidar(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
    void callbackCam(const sensor_msgs::ImageConstPtr& msg);
    
    //void callbackTrafficLight(const t3_msgs::traffic_light_data::ConstPtr& msg);
    void control();
    void process();
    float calculate_azimuth(control::TmpObject bbox);
    std::pair<float,float> distance(float azimuth, control::TmpObject bbox);
    std::pair<float, float> LiDAR_to_CAMERA(control::TmpLidar lidar);
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
      return {depth*azimuth, depth*p};
    }
  }


  std::pair<float, float> Controller::LiDAR_to_CAMERA(control::TmpLidar lidar){
    float cam_x;
    float cam_y;
    cam_x = 320 + (FOCAL_LENGTH * lidar.y*100) / (lidar.x*100+8.9);
    cam_y = 240 - (FOCAL_LENGTH * 9.5) / (lidar.x*100+8.9);
    return {cam_x, cam_y};
  }



  void Controller::callbackObject(const t3_msgs::object_data::ConstPtr& msg)
  {
    sensor_state_.object.reduce(msg);
  }

  void Controller::callbackLidar(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
  {
    sensor_state_.lidar_points.reduce(msg);
    //for (const auto& pt: msg->points){
        // std::cout << count++ << ": ";
        //std::cout <<pt.x << ", "<<pt.y << ", "<< pt.z << std::endl;
    
  }
  
  void Controller::callbackCam(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      vFrame = cv::Mat(HEIGHT, WIDTH, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step);
      cv::undistort(this->vFrame, dst, CAMERA_MATRIX, DISTORTION_COEFFICIENTES_ERROR);

      process();
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("callback exception: %s", e.what());
      return;
    }
  }

  void Controller::process()
  {
    
    for (auto point :points){
      cv::drawMarker(dst,point, cv::Scalar(0, 0, 255));
    }
    cv::imshow("test",dst);

  }


  void Controller::control()
  {
    tmpObject.clear();
    tmpLidar.clear();
    
    points.clear();

    if(sensor_state_.object.boundingBoxes.size()>0)
    {
      for(auto objectBox :sensor_state_.object.boundingBoxes)
      {
        control::TmpObject tmp_object;
        tmp_object.id = objectBox.id;
        tmp_object.xmin = (int)objectBox.xmin*640/352;
        tmp_object.ymin = (int)objectBox.ymin*480/352;
        tmp_object.xmax = (int)objectBox.xmax*640/352;
        tmp_object.ymax = (int)objectBox.ymax*480/352;
        tmp_object.probability = objectBox.probability;
        float azimuth =  calculate_azimuth(tmp_object);
        auto depth =  distance(azimuth, tmp_object);
        std::cout << "[" <<depth.first << "," << depth.second << "] ";
        tmpObject.emplace_back(tmp_object);
      }
      std::cout << "" <<std::endl;
    }
    for(auto ptr :sensor_state_.lidar_points.lidarPoints)
    {
      if (ptr.y > -5 && ptr.y < 5 &&  ptr.x < 0){
        control::TmpLidar tmp_lidar;
        //float mulmax[4][1] = {{ptr.x}, {ptr.y}, {0.}, {1.}};
       // cv::Mat mul1 = cv::Mat(4,1,CV_64F, mulmax);
         // {-9.80913534e-01, -1.92942836e-01 ,-2.41184835e-02 ,-4.76059423e-03}};

        cv::Mat mul1 = cv::Mat_<float>({4, 1}, {ptr.x, ptr.y, 0.0, 1.0});
        // tmp_lidar.x = ptr.x;
        // tmp_lidar.y = ptr.y;
      // std::cout <<mul1 << std::endl;
      // std::cout <<PROJECT_MATRIX << std::endl;
      cv::Mat projected_point = PROJECT_MATRIX*mul1;
      float x = projected_point.at<float>(0,0);
      float y = projected_point.at<float>(0,1);
      float z = projected_point.at<float>(0,2);
      std::cout << "X: " << x/z << "Y: " << y/z << std::endl;
      int u = x/z;
      int v = y/z;
      if(u>=0 && u<640 && v>=0 && v<480){
        //cv::drawMarker(vFrame,cv::Point(u,v), cv::Scalar(0, 0, 255));
        points.push_back(cv::Point(u,v));
      }
      // std::cout << PROJECT_MATRIX*mul1 << std::endl;
        // auto lidar_ptr = LiDAR_to_CAMERA(tmp_lidar);
        
        // std::cout << "[" <<lidar_ptr.first << "," << lidar_ptr.second << "] "<< std::endl ;
      }
    }
    //cv::imshow("test", this->vFrame);

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
    int k = cv::waitKey(1);
    if (k == 27 || k == ' ')  // ESC key or space bar
      break;
  }
  cv::destroyAllWindows();
  return 0;
}
