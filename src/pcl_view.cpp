#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//将所有函数封装在一个类中，提供一个使用回调函数分享变量的简介方式
class cloudHandler
{
public:
    cloudHandler()
    : viewer("Cloud Viewer")
    {
        pcl_sub = nh.subscribe("/camera/depth/color/points", 10, &cloudHandler::cloudCB, this);
        //创建topic为pcl_output,处理队列为10，&cloudHandler::cloudCB回调函数地址，this回调函数所处的类即此类
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);
        //设置一个定时器，每100ms会触发一次回调，定时器用来周期性检查窗口是否关闭，如果关闭则终止代码
    }
    
    //PCL点云通过showCloud函数直接传递给查看器，查看器自动更新显示
    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(input, cloud);

        viewer.showCloud(cloud.makeShared());
    }
    //timerCB回调函数通过一个ROS定时器每隔100ms调用一次
    void timerCB(const ros::TimerEvent&)
    {
        if (viewer.wasStopped())//如果查看器关闭了，我们的动作则是终止节点
        {
            ros::shutdown();
        }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_visualize");

    cloudHandler handler;

    ros::spin();

    return 0;
}

