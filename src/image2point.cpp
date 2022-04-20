//
// Created by bat2 on 2021/9/13.
// the purpose of this demo is to show how to turn rgb and depth into Pointcloud and publish it
//

#include "ros/ros.h"

#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>

#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>


using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> testcloud;
class picture2Point
{
public:
    ros::NodeHandle nh;
    ros::Publisher pcl_pub;

    picture2Point(): nh("~"){
        message_filters::Subscriber<sensor_msgs::Image>rgb_sub(nh,"/camera/color/image_raw",1);
        message_filters::Subscriber<sensor_msgs::Image>depth_sub(nh,"/camera/depth/image_rect_raw",1);
        pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("pointCloud",1);// 添加的  发布点云

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
        sync.registerCallback(boost::bind(&picture2Point::callback, this, _1, _2));

        ros::spin();
    }
    ~picture2Point(){};

    sensor_msgs::PointCloud2 output;

    void callback(const sensor_msgs::ImageConstPtr &RGB_image,const sensor_msgs::ImageConstPtr &depth_image)
    {
        try {
            cv_bridge::CvImageConstPtr image_ptr=cv_bridge::toCvShare(RGB_image);
            cv::Mat RGB_image=image_ptr->image;

            cv_bridge::CvImageConstPtr image_Ptr1=cv_bridge::toCvShare(depth_image);
            cv::Mat Depth_image=image_Ptr1->image;

       cv::imshow("rgb",RGB_image);
        cv::imshow("depth",Depth_image);
        cv::waitKey(10);

            picture2Point::generatePointcloud(RGB_image,Depth_image);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge Exception %s",e.what());
        }
    }
    void generatePointcloud(const cv::Mat &rgb,const cv::Mat &depth)  //点云生存
    {
        pcl::PointCloud<pcl::PointXYZRGB> tmp;
        for(size_t v=0;v<rgb.rows;v++)
        {
            for(size_t u=0;u<rgb.cols;u++)
            {
                float d=depth.ptr<float>(v)[u];
                if(d<0.01||d>10)
                    continue;
                PointT p;
                p.z=d;
                p.x=(u-320)*d;
                p.y=(v-240.5)*d;
                // p.x=(u-320)*d/554.25468;
                // p.y=(v-240.5)*d/554.254;

                p.b=rgb.ptr<unsigned>(v)[u*3];
                p.g=rgb.ptr<unsigned>(v)[u*3+1];
                p.r=rgb.ptr<unsigned>(v)[u*3+2];
                tmp.points.push_back(p);
            }
        }
        pcl::visualization::CloudViewer viewer("pcd viewer");

        sensor_msgs::PointCloud2  output_msh;
        pcl::toROSMsg(tmp,output_msh);
        output_msh.header.frame_id="support_depth";
        output_msh.header.stamp=ros::Time::now();
        pcl_pub.publish(output_msh);

    }
};



int main(int argc,char *argv[])
{
    ros::init(argc,argv,"image2point");
    picture2Point node;
    return 0;
}
