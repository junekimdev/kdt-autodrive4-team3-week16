#include <string>
#include <vector>
// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "t3_msgs/camera_calibrate_data.h"

// Include OpenCV
#include "opencv2/opencv.hpp"

namespace color
{
const cv::Scalar WHITE = cv::Scalar(255, 255, 255);
const cv::Scalar BLACK = cv::Scalar(0, 0, 0);
const cv::Scalar RED = cv::Scalar(0, 0, 255);
const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
}  // namespace color

namespace sensor
{

const std::string NAME = "stop_line";
const std::string SUB_TOPIC = "usb_cam/image_raw";
const std::string PUB_TOPIC = "camera_calibrate_data";
const cv::Mat CAMERA_MATRIX = cv::Mat_<float>({3, 3}, {348.14820298,   0.,         323.82585721, 
  0.,         345.88660809, 260.82857638,
  0.,           0.,           1.       });

const cv::Mat DISTORTION_COEFFICIENTES_ERROR = cv::Mat_<float>({1,5}, {-0.3460703,   0.14435681,  0.00177496 , 0.00118325 ,-0.0321834});


constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;

class CameraCalibration
{
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat vFrame;


public:
  bool enable_debug;
  t3_msgs::camera_calibrate_data msg;

  CameraCalibration()
  {
    node.param<bool>("camera_calibration_enable_debug", enable_debug, true);
    sub = node.subscribe(SUB_TOPIC, 1, &CameraCalibration::callback, this);
    pub = node.advertise<t3_msgs::camera_calibrate_data>(PUB_TOPIC, 1);
    if (enable_debug)
      cv::namedWindow(NAME);
  }

  void callback(const sensor_msgs::ImageConstPtr& msg);
  void publish();
  void process();
};

void CameraCalibration::callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    vFrame = cv::Mat(HEIGHT, WIDTH, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step);
    process();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("callback exception: %s", e.what());
    return;
  }
}

void CameraCalibration::publish()
{
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = NAME;
  pub.publish(msg);
  
}




void CameraCalibration::process()
{
  cv::Mat undistorted_frame;
  cv::Mat newcameraMtx, roi;

  newcameraMtx, roi = cv::getOptimalNewCameraMatrix(CAMERA_MATRIX,DISTORTION_COEFFICIENTES_ERROR, cv::Size(WIDTH, HEIGHT), 1, cv::Size(WIDTH, HEIGHT));
  cv::undistort(this->vFrame, undistorted_frame, CAMERA_MATRIX, DISTORTION_COEFFICIENTES_ERROR, newcameraMtx);


  cv::imshow(NAME, undistorted_frame);

}

}  // namespace sensor

int main(int argc, char** argv)
{
  ros::init(argc, argv, sensor::NAME);
  sensor::CameraCalibration camera_calibration;
  ROS_INFO("%s is ONLINE", sensor::NAME.c_str());

  while (ros::ok())
  {
    ros::spinOnce();
    if (camera_calibration.enable_debug)
    {
      int k = cv::waitKey(1);
      if (k == 27 || k == ' ')  // ESC key or space bar
        break;
    }
  }
  cv::destroyAllWindows();
  return 0;
}
