#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class Image_test:
    def __init__(self):
        rospy.init_node("wego_node")
        rospy.Subscriber(
            "/usb_cam/image_rect_color/compressed", CompressedImage, self.image_CB
        )
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher(
            "gray_color/compressed", CompressedImage, queue_size=5
        )  #
    
    def image_CB(self, data):
        # print(data)
        # img = self.bridge.compressed_imgmsg_to_cv2(data)
        img = cv2.imread(
            "/usb_cam/image_rect_color/compressed", cv2.IMREAD_COLOR  #/usb_cam/image_rect_color/compressed home/wego/edu_ws/src/test/scripts/lane.jpg
        )
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)
        lower_bound = np.array([0, 50 ,0])
        upper_bound = np.array([45, 255, 255])
        img_inrange = cv2.inRange(img_hsv, lower_bound, upper_bound) #4일 오후 56:41
        src = np.float32([[225, 700], [510, 340], [655, 344], [1060,700]]) #4일 오후 1:01:41
        dst = np.float32([[250, 700], [250, 0], [1060, 0], [1060,700]])
        matrix = cv2.getPerspectiveTransform(src,dst) #4일 오후 1:03:46
        matrix_inv = cv2.getPerspectiveTransform(src, dst)
        warp_img= cv2.warpPerspective(
                img_inrange, matrix, [img_hsv.shape[1]], [img_hsv.size[0]]
        )

        warp_origin_img = cv2.warpPerspective(
            img, matrix, [img_hsv.shape[1], img_hsv.shape[1]]
        )
        canny_img = cv2.Canny(warp_img, 50,150)
        line = cv2.HoughLinesP(warp_origin_img, 1, np.pi / 180, 1, 5, 50)
        for i in range(len(line)):
            for x1, y1, x2, y2 in line[i]:
                cv2.line(canny_img, (x1,y1), (x2,y2), [0, 0, 255])
        warp_inv_img = cv2.warpPerspective(
            warp_origin_img, matrix, [img_hsv.shape[1]], [img_hsv.size[0]]
        )
        print(line)
        cv2.imshow("img",img)
        cv2.imshow("img_inrange", img_inrange)
        cv2.imshow("warp_img", warp_img)
        cv2.imshow("canny_img", canny_img)
        cv2.imshow("warp_origin_img",warp_origin_img)


        cv_img_msg = self.bridge.cv2_to_compressed_imgmsg(data)
        self.img_pub.publish(cv_img_msg)


def main():
    image_test = Image_test()
    rospy.spin

if __name__=="__main__":
    main()

