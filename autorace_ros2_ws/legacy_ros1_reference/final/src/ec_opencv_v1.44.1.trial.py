#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class Webot_control:
    def __init__(self):
        rospy.init_node("webot_node")
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.comp_img_CB)
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("gray_color", Image, queue_size=5)

    def comp_img_CB(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        lines_image = image.copy()

        canny_conversion = self.canny_edge(image)
        roi_conversion = self.reg_of_interest(canny_conversion)
        lines = cv2.HoughLinesP(roi_conversion, 1, np.pi/180, 100, minLineLength=40, maxLineGap=5)
        
        if lines is not None:
            averaged_lines = self.average_slope_intercept(image, lines)
            lines_image = self.show_lines(image, averaged_lines)
            combine_image = cv2.addWeighted(image, 0.8, lines_image, 1, 1)

            # Convert OpenCV image to ROS Image message and publish
            processed_image_msg = self.bridge.cv2_to_imgmsg(combine_image, "bgr8")
            self.img_pub.publish(processed_image_msg)

            # # Display the combined image with lane detection using cv2.imshow()
            # cv2.imshow("Lane Detection", combine_image)
            # cv2.waitKey(1)  # Wait for a short period (1ms) to update the window



    def canny_edge(self, image):
        gray_conversion = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur_conversion = cv2.GaussianBlur(gray_conversion, (5, 5), 0)
        canny_conversion = cv2.Canny(blur_conversion, 50, 150)
        return canny_conversion

    def reg_of_interest(self, image):
        image_height = image.shape[0]
        polygons = np.array([[ (200, image_height), (1100, image_height), (550, 250)]])
        image_mask = np.zeros_like(image)
        cv2.fillPoly(image_mask, polygons, 255)
        masking_image = cv2.bitwise_and(image, image_mask)
        return masking_image

    def show_lines(self, image, lines):
        lines_image = np.zeros_like(image)
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
        return lines_image

    def average_slope_intercept(self, image, lines):
        left_fit = []
        right_fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameter = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameter[0]
            intercept = parameter[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        left_line = self.make_coordinates(image, left_fit_average)
        right_line = self.make_coordinates(image, right_fit_average)

        return np.array([[left_line, right_line]])

    def make_coordinates(self, image, line_parameters):
        slope, intercept = line_parameters
        y1 = image.shape[0]
        y2 = int(y1 * (3/5))
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return np.array([x1, y1, x2, y2])

    cv2.imshow('ori', lanelines_image)
    cv2.imshow("roi", lines_image)
    cv2.imshow("combined", combine_image)

def main():
    webot_control = Webot_control()
    rospy.spin()

if __name__ == "__main__":
    main()







