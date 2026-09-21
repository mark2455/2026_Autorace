#! /usr/bin python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import math
import cv2 as cv
import numpy as np

class LaneDetectorROS:

    def __init__(self):
        rospy.init_node("lane_detector_node")
        rospy.Subscriber("/usb_cam/rgb/image_rect/compressed", CompressedImage, self.camera_CB) 
        self.pub = rospy.Publisher("/cam_data", CompressedImage, queue_size=1)

    def camera_CB(self, msg):
        # Convert compressed image data to OpenCV image format
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img = cv2.resize(img, (640, 360))

        dst = cv.Canny(img, 50, 200, None, 3)

        cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)

        lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)

        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                cv.line(cdst, pt1, pt2, (0, 0, 255), 3, cv.LINE_AA)

        linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv.LINE_AA)

        # Convert OpenCV image to compressed image format and publish
        ret, buf = cv2.imencode(".jpg", cdst)
        if ret:
            compressed_img_msg = CompressedImage()
            compressed_img_msg.header = msg.header
            compressed_img_msg.format = "jpeg"
            compressed_img_msg.data = buf.tobytes()
            self.pub.publish(compressed_img_msg)

if __name__ == "__main__":
    try:
        lane_detector = LaneDetectorROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
