#!/usr/bin/env python3
# coding:utf-8

import argparse
import os
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from IPM_config import BevMap_size

def callback(data):
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    calibrated_img = cv2.warpPerspective(cv_img, transform_matrix, BevMap_size,flags=cv2.INTER_BITS2,  borderMode=cv2.BORDER_TRANSPARENT) 
    
    ipm_pb.publish(bridge.cv2_to_imgmsg(calibrated_img, "bgr8")) #发布消息
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
    
    ipm_pb = rospy.Publisher("/rgb_camera/image_ipm", Image, queue_size=1)
    
    rospy.spin()
