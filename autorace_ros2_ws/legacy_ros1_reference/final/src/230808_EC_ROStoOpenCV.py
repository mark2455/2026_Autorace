#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class Image_test:
    def __init__(self):
        rospy.init_node("wego_node")
        rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_CB)     #2:01:08
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("gray_color",Image, queue_size=5)
    
    def image_CB(self,data):
        img = self.bridge.imgmsg_to_cv2(data, "passthrough")
        # cv2.namedWindow("image", cv2)        
        cv2.imshow("img",img)
        cv2.waitKey(0)

        cv_img_msg = self.bridge.cv2_to_imgmsg(img)
        self.img_pub.publish(cv_img_msg)


def main():
    image_test = Image_test()
    rospy.spin

if __name__=="__main__":
    main()
