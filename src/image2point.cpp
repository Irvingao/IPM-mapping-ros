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

//headers of opencv
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

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


// 全局变量
ros::Publisher pubCloud;
int R, G, B; // R,G,B值

cv::Mat image; // 接收到图像信息

pcl::PointCloud<pcl::PointXYZRGB> cloud;  //建立了一个pcl的点云中间量（不能直接发布）
sensor_msgs::PointCloud2 output_msg;  //建立一个可以直接发布的点云

void imagepointcloudCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8") -> image; // 获取图像

    pcl::PointCloud<pcl::PointXYZRGB> cloud;  //建立了一个pcl的点云（不能直接发布）
    cloud.width = image.rows;
    cloud.height =image.cols;
    cloud.points.resize(cloud.width * cloud.height); // 创建等于像素个数的点云

    sensor_msgs::PointCloud2 output_msg;  //建立一个可以直接发布的点云
    output_msg.header.stamp = ros::Time::now();

    int n = 0; // 点云中的点的标号
    for (int i = 0; i < image.rows; i++)
    { 
      for(int j=0;j<image.cols;j++)
      {
        R = image.at<cv::Vec3b>(i, j)[2];
        G = image.at<cv::Vec3b>(i, j)[1];
        B = image.at<cv::Vec3b>(i, j)[0];
        if (R==0 && G==0 && B==0){
          continue;
        } else {
          cloud.points[n].x = 5 * j / (double)cloud.height; // 放缩比例长
          cloud.points[n].y = 5 * i / (double)cloud.width; // 宽
          cloud.points[n].z = 0;
          cloud.points[n].r = R; // R通道
          cloud.points[n].g = G; // G通道
          cloud.points[n].b = B; // B通道
        }
        n = n + 1;
      }
    }
    pcl::toROSMsg(cloud, output_msg); //将点云转化为消息才能发布
    output_msg.header.frame_id = "map";
    pubCloud.publish(output_msg); //发布调整之后的点云数据，主题为/adjustd_cloud
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
}
  cv::waitKey(1);
}


int main(int argc, char** argv) {
  // 初始化节点
  ros::init(argc, argv, "pcl_process_node");  //初始化了一个节点，名字为pcl_process_node
  // 局部变量
  ros::NodeHandle n;
  ros::Subscriber subCloud;

  // opencv节点
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("/rgb_camera/image_ipm", 1, imagepointcloudCallback);
  
  // 点云节点
  pubCloud = n.advertise<sensor_msgs::PointCloud2>("/map/pointcloud_ipm", 1000);  //建立了一个发布器，主题是/adjusted_cloud，方便之后发布调整后的点云

  ros::spin();
  return 0;
}