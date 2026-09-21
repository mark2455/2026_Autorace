#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image,CompressedImage
from math import *
import os # 명령창에서 실시간 값만 보이게 할려고 가져오는 놈인 듯 (5일차 오후 강의 중)
import cv2
from cv_bridge import CvBridge
import numpy as np


class Image_test:
    def __init__(self):
        rospy.init_node("wego_node")
        rospy.Subscriber(
            "/usb_cam/image_rect_color/compressed", CompressedImage, self.image_CB
        )
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "gray_color/compressed", CompressedImage, queue_size=5
        )  #
        self.image = []
    
    def image_CB(self, data):
        # print(data)
        # image = self.bridge.compressed_imagemsg_to_cv2(data)
        self.image = self.bridge.compressed_imagemsg_to_cv2(data)  #/usb_cam/image_rect_color/compressed home/wego/edu_ws/src/test/scripts/lane.jpg
        
        image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(image_hsv)
        lower_bound = np.array([0, 50 ,0])
        upper_bound = np.array([45, 255, 255])
        image_inrange = cv2.inRange(image_hsv, lower_bound, upper_bound) #4일 오후 56:41
        src = np.float32([[225, 700], [510, 340], [655, 344], [1060,700]]) #4일 오후 1:01:41
        dst = np.float32([[250, 700], [250, 0], [1060, 0], [1060,700]])
        matrix = cv2.getPerspectiveTransform(src,dst) #4일 오후 1:03:46
        matrix_inv = cv2.getPerspectiveTransform(src, dst)
        warp_image= cv2.warpPerspective(
                image_inrange, matrix, [image_hsv.shape[1]], [image_hsv.size[0]]
        )

        warp_origin_image = cv2.warpPerspective(
            self.image, matrix, [image_hsv.shape[1], image_hsv.shape[1]]
        )
        canny_image = cv2.Canny(warp_image, 50,150)
        line = cv2.HoughLinesP(warp_origin_image, 1, np.pi / 180, 1, 5, 50)
        for i in range(len(line)):
            for x1, y1, x2, y2 in line[i]:
                cv2.line(canny_image, (x1,y1), (x2,y2), [0, 0, 255])
        warp_inv_image = cv2.warpPerspective(
            warp_origin_image, matrix, [image_hsv.shape[1]], [image_hsv.size[0]]
        )
        print(line)
        cv2.imshow("image",self.image)
        cv2.imshow("image_inrange", image_inrange)
        cv2.imshow("warp_image", warp_image)
        cv2.imshow("canny_image", canny_image)
        cv2.imshow("warp_origin_image",warp_origin_image)


        cv_image_msg = self.bridge.cv2_to_compressed_imgmsg(data)
        self.image_pub.publish(cv_image_msg)


def main():
    image_test = Image_test()
    rospy.spin

if __name__=="__main__":
    main()

