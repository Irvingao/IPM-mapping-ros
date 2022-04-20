#!/usr/bin/env python3
# coding:utf-8

import argparse
import os
import rospy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from IPM_config import BevMap_size

def callback(data):
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    calibrated_img = cv2.warpPerspective(cv_img, transform_matrix, BevMap_size,flags=cv2.INTER_BITS2,  borderMode=cv2.BORDER_TRANSPARENT) 
    
    header = Header(stamp = rospy.Time.now())
    header.frame_id = "Camera"
    ros_frame.header=header
    ros_frame.data = np.array(calibrated_img).tostring() #图片格式转换
    ipm_pb.publish(ros_frame) #发布消息
    calibrated_img = np.zeros_like(calibrated_img)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--matrix", 
                        type=str,
                        default=os.path.join(os.path.split(os.path.realpath(__file__))[0], "transform_matrix.npy"),
                        help="IPM transform matrix")            
    args = parser.parse_args()

    transform_matrix = np.load(args.matrix)
    
    rospy.init_node('ipm_node', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber('/rgb_camera/image_seg', Image, callback)
    
    ipm_pb = rospy.Publisher("/rgb_camera/image_ipm", Image, queue_size=2)
    
    ros_frame = Image()
    ros_frame.width = BevMap_size[0]
    ros_frame.height = BevMap_size[1]
    ros_frame.encoding = "bgr8"
    
    rospy.spin()
