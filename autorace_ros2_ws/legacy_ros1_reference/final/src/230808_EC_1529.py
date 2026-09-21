#!/usr/bin/env python
# -*- coding : utf-8 -*-

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])
# To use OpenCV with ROS


rospy.init_node('cam_data', anonymous=True)
rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)


def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
    # imgmsg data -> opencv
    # "bgr8": CV_8UC3, color image with blue-green-red color order

while not rospy.is_shutdown():
    if (cv_image.size != (640*480*3)): continue
    # wait until image is received. resolution: 640 x 380, true color image
    gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray,(5,5),0)
    edge_img = cv2.Canny(blur_gray, 50, 150)

    cv2.imshow("original",cv_image)
    cv2.imshow("gray",gray)
    cv2.imshow("GaussianBlur",blur_gray)
    cv2.imshow("edge_img",edge_img)
    cv2.waitKey(1)