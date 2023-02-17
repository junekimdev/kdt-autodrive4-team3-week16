
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
// #include <iostream>       // std::cout
// #include <typeinfo>       // operator typeid

using namespace std;
ros::Publisher pub;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;
template <class T>
void print_pc(pcl::PointCloud<T>& cloud){
    int count = 0;
    for (const auto& pt: cloud.points){
        cout << count++ << ": ";
        cout <<pt.x << ", "<<pt.y << ", "<< pt.z << endl;
    }
}
void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_colored,
              const std::vector<int> &color) {

    int N = pc.points.size();

    pc_colored->clear();
    pcl::PointXYZRGB pt_tmp(0,255,0);
    // std::uint32_t rgb = ((std::uint32_t)255 << 16 | (std::uint32_t)0 << 8 | (std::uint32_t)0);

    // for (int i = 0; i < N; i++)
    // {
    //     const auto &pt = pc.points[i];
    //     if(-1<pt.x && pt.x <1 && pt.y>0){
    //         rgb = ((std::uint32_t)color[0] << 0 | (std::uint32_t)255 << 8 | (std::uint32_t)0);
    //     }else{
    //         rgb = ((std::uint32_t)0 << 16 | (std::uint32_t)0 << 8 | (std::uint32_t)255);
    //     }
    //     pt_tmp.x = pt.x;
    //     pt_tmp.y = pt.y;
    //     pt_tmp.z = pt.z;
    //     pt_tmp.rgb=*reinterpret_cast<float*>(&rgb);;
    //     pc_colored->points.emplace_back(pt_tmp);
    // }
    
    std::uint32_t rgb = ((std::uint32_t)255 << 16 | (std::uint32_t)0 << 8 | (std::uint32_t)0);
    for (int i = 0; i < N/3; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.rgb=*reinterpret_cast<float*>(&rgb);;
        pc_colored->points.emplace_back(pt_tmp);
    }
    rgb = ((std::uint32_t)0 << 16 | (std::uint32_t)255 << 8 | (std::uint32_t)0);
    for (int i = N/3; i < 2*N/3; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.rgb=*reinterpret_cast<float*>(&rgb);;
        pc_colored->points.emplace_back(pt_tmp);
    }
    rgb = ((std::uint32_t)0 << 16 | (std::uint32_t)0 << 8 | (std::uint32_t)255);
    for (int i = 2*N/3; i < N; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.rgb=*reinterpret_cast<float*>(&rgb);;
        pc_colored->points.emplace_back(pt_tmp);
    }
}
sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser)
    {
      static laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 pc2_dst;
      projector.projectLaser(laser, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
      pc2_dst.header.frame_id = "map";

      return pc2_dst;
    }

// Laser Scan -> PointCloud2 -> pcl::PointCloud -> PointCloud2
void cloud_cb (const sensor_msgs::LaserScan laser){
    sensor_msgs::PointCloud2 tmp = laser2cloudmsg(laser);
    point_cloud_t::Ptr input_ptr(new point_cloud_t());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::fromROSMsg(tmp,*input_ptr);
    
    colorize(*input_ptr,output_ptr,{0, 255, 0});
    sensor_msgs::PointCloud2 ros_output;
    
    pcl::toROSMsg(*output_ptr, ros_output);
    print_pc(*output_ptr);
    ros_output.header.frame_id="abc";
    pub.publish(ros_output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/scan", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
