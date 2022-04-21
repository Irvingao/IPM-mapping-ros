//common headers
#include <iostream>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <string>

//headers of ros
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/PointCloud2.h>

//headers of pcl
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("realtime pcl"));

ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
   // 声明存储原始数据与滤波后的数据的点云的格式
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // 转化为PCL中的点云的数据格式
  pcl_conversions::toPCL(*input, *cloud);

  pub.publish (*cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
  cloud1.reset (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud1);

//  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//  viewer.showCloud(cloud1,"Simple Cloud Viewer");

  viewer1->removeAllPointClouds();  // 移除当前所有点云
  viewer1->addPointCloud(cloud1, "realtime pcl");
  viewer1->updatePointCloud(cloud1, "realtime pcl");
  viewer1->spinOnce(0.001);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //"/camera/depth/color/points"realsense发布的点云数据
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 10, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 10);
  // Spin
  ros::spin ();
}
