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

using namespace std;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("realtime pcl"));

    
    // pcl::PointCloud<pcl::PointXYZRGB> tmp;
    // for(size_t v=0;v<rgb.rows;v++)
    // {
    //     for(size_t u=0;u<rgb.cols;u++)
    //     {
    //         float d=depth.ptr<float>(v)[u];
    //         if(d<0.01||d>10)
    //             continue;
    //         PointT p;
    //         p.z=d;
    //         p.x=(u-320)*d;
    //         p.y=(v-240.5)*d;
    //         // p.x=(u-320)*d/554.25468;
    //         // p.y=(v-240.5)*d/554.254;

    //         p.b=rgb.ptr<unsigned>(v)[u*3];
    //         p.g=rgb.ptr<unsigned>(v)[u*3+1];
    //         p.r=rgb.ptr<unsigned>(v)[u*3+2];
    //         tmp.points.push_back(p);
    //     }
    // }

    // sensor_msgs::PointCloud2  output_msh;
    // pcl::toROSMsg(tmp,output_msh);

ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // 声明存储原始数据与滤波后的数据的点云的格式
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // 转化为PCL中的点云的数据格式
    pcl_conversions::toPCL(*input, *cloud);
    // 发布ROS点云消息
    pub.publish (*cloud);
    // 转换成PointCloud格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
    cloud1.reset (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud1);
    

    // 显示更新
    // viewer1->removeAllPointClouds();  // 移除当前所有点云
    // viewer1->addPointCloud(cloud1, "realtime pcl");
    // viewer1->updatePointCloud(cloud1, "realtime pcl");
    // viewer1->spinOnce(0.001);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr cvMatToPcl(cv::Mat &mat) {
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(
            new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "begin parsing file" << std::endl;
    for (int ki = 0; ki < mat.rows; ki++) {
        for (int kj = 0; kj < mat.cols; kj++) {
            pcl::PointXYZ pointXYZ;

            pointXYZ.x = mat.at<cv::Point3f>(ki, kj).x;
            pointXYZ.y = mat.at<cv::Point3f>(ki, kj).y;
            pointXYZ.z = 0;

            // if(pointXYZ.z <= 0)
            //     continue;
            cloud->points.push_back(pointXYZ);
        }
    }
    return cloud;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_STREAM("Get Msg");
    // OpenCV get image 
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8") -> image;
        // cv::imshow("view", image);
        
        // Mat转PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cvMatToPcl(image);

        // 点云显示更新
        viewer1->removeAllPointClouds();  // 移除当前所有点云
        viewer1->addPointCloud(cloud, "realtime pcl");
        viewer1->updatePointCloud(cloud, "realtime pcl");
        viewer1->spinOnce(0.001);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    // cv::waitKey(1);
}

int main (int argc, char** argv)
{
    // Initialize ROS node
    ros::init (argc, argv, "image2pcl");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    // /camera/depth/color/points"realsense发布的点云数据
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 10, cloudCallback);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PCLPointCloud2> ("output", 10);

    // Cerat a ROS subscriber for the input image
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
    
    
    // Spin
    ros::spin();
    cv::destroyWindow("view");

    ros::spin ();
}