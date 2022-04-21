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

int R, G, B;

pcl::PointCloud<pcl::PointXYZRGB> cloud;  //建立了一个pcl的点云中间量（不能直接发布）
sensor_msgs::PointCloud2 output_msg;  //建立一个可以直接发布的点云

sensor_msgs::PointCloud2 msg;  //接收到的点云消息
sensor_msgs::PointCloud2 adjust_msg;  // 等待发送的点云消息
pcl::PointCloud<pcl::PointXYZRGB> adjust_pcl;   //建立了一个pcl的点云，作为中间过程
cv::Mat image; // 接收到图像信息


//回调函数
void getcloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr adjust_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);   //放在这里是因为，每次都需要重新初始化
  pcl::fromROSMsg(*laserCloudMsg, *adjust_pcl_ptr);  //把msg消息转化为点云
  adjust_pcl = *adjust_pcl_ptr;  
  for (int i = 0; i < adjust_pcl.points.size(); i++)
  //把接收到的点云位置不变，颜色全部变为绿色
  {
    adjust_pcl.points[i].r = 0;
    adjust_pcl.points[i].g = 255;
    adjust_pcl.points[i].b = 0;
  }
  pcl::toROSMsg(adjust_pcl, adjust_msg);  //将点云转化为消息才能发布
  // pubCloud.publish(adjust_msg); //发布调整之后的点云数据，主题为/adjustd_cloud
}




void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO_STREAM("Get Msg");
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8") -> image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 
void imagepointcloudCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8") -> image; // 获取图像

    // pcl::PointCloud<pcl::PointXYZRGB> cloud;  //建立了一个pcl的点云（不能直接发布）
    cloud.width = image.rows;
    cloud.height =image.cols;
    cloud.points.resize(cloud.width * cloud.height); // 创建这么多个点

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
    output_msg.header.frame_id = "camera_link";
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
  // image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imagepointcloudCallback);
  image_transport::Subscriber sub = it.subscribe("/rgb_camera/image_ipm", 1, imagepointcloudCallback);
  
  // 点云节点
  subCloud = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, getcloud); //接收点云数据，进入回调函数getcloud()
  pubCloud = n.advertise<sensor_msgs::PointCloud2>("/adjustd_cloud", 1000);  //建立了一个发布器，主题是/adjusted_cloud，方便之后发布调整后的点云

  ros::spin();
  return 0;
}
