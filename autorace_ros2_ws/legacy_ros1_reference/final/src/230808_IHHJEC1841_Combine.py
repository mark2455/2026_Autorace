#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
#from math import *
import os # 명령창에서 실시간 값만 보이게 할려고 가져오는 놈인 듯 (5일차 오후 강의 중)
import cv2
from cv_bridge import CvBridge
import numpy as np

import sys
import math


class Webot_control:
    def __init__(self):
            rospy.init_node("webot_node") # 노드 이름 정하기
            rospy.Subscriber("usb_cam/image_rect_color/compressed",CompressedImage,self.comp_img_CB)
            self.cvbridge = CvBridge()
            self.comp_img = [] #이 부분은 없어도 동작 하는 것 같음



    def comp_img_CB(self,msg):

        # <이미지 필수 전처리 과정 start>
        self.comp_img = self.cvbridge.compressed_imgmsg_to_cv2(msg) #잘림 1:57:14
        os.system("clear") 
        # <이미지 필수 전처리 과정 end>


        # <이미지 뷰어 예시 start>
        cv2.imshow("comp_img",self.comp_img) #이미지 opencv 뷰어
        # <이미지 뷰어 예시 end>


        ## <HSV 성분별 색깔 분리 start>
        img_hsv = self.comp_img
        #img_hsv =cv2.cvtColor(self.comp_img,cv2.COLOR_BGR2HSV)  #BGR2HSV, RGB2HSV 색변환시 이상한 색이므로 , 카메라 색상 기본값이 HSV인 것으로 추정
        h,s,v = cv2.split(img_hsv)
        # cv2.imshow("h",h) #이미지 opencv 뷰어
        # cv2.imshow("s",s) #이미지 opencv 뷰어
        # cv2.imshow("v",v) #이미지 opencv 뷰어
        ## <HSV 성분별 색깔 분리 end>


        # # <이미지 인덱스 값 출력부위 start>
        # print(f"H_average: {np.average(h)}")
        # print(f"S_average: {np.average(s)}")
        # print(f"V_average: {np.average(v)}")
        # print(msg)
        # # <이미지 인덱스 값 출력 end>


        # <이미지 HSV기준 추출 색상 정하기 start> 여기 값은 광원(형광등 on, off, 구름 낄 때 안 낄때) 에 따라 매우 달라질 수 있습니다. 
        lower_bound = np.array([185,190,175]) 
        upper_bound = np.array([225,225,210]) 
        img_inrange = cv2.inRange(img_hsv, lower_bound, upper_bound)
        # cv2.imshow("img_inrange", img_inrange)
        ### 동아리 방의 경우 lower_bound = np.array([185,190,175]) upper_bound = np.array([225,225,210])  
        # <이미지 HSV기준 추출 색상 정하기 end>


        # <ROI 설정: 공중에서 보는 View로 start>
        src = np.float32([[70,479],[224,347],[415,347],[569,479]])
        dst = np.float32([[70,479],[70,0],[569,0],[569,479]])
        matrix = cv2.getPerspectiveTransform(src,dst)

        warp_origin_img = cv2.warpPerspective(img_hsv, matrix, [img_hsv.shape[1],img_hsv.shape[0]])
        cv2.imshow("warp_origin_img", warp_origin_img)

        warp_img = cv2.warpPerspective(img_inrange, matrix, [img_hsv.shape[1],img_hsv.shape[0]])
        # cv2.imshow("warp_inrange_img", warp_img)
        # <ROI 설정: 공중에서 보는 View로 end>







        # <Line 추출 by HJ    start>
        src = warp_img = cv2.warpPerspective(img_inrange, matrix, [img_hsv.shape[1],img_hsv.shape[0]])

        dst = cv2.Canny(src, 50, 200, None, 3)

        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)

        # lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)

        # if lines is not None:
        #     for i in range(0, len(lines)):
        #         rho = lines[i][0][0]
        #         theta = lines[i][0][1]
        #         a = math.cos(theta)
        #         b = math.sin(theta)
        #         x0 = a * rho
        #         y0 = b * rho
        #         pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
        #         pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
        #         cv2.line(cdst, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

        linesP = cv2.HoughLinesP(dst, 1, 5 / 180, 50, None, 50, 10)

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

        #cv2.imshow("Source", src)
        # cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)




        #라인 이어주기

        lanelines_image = warp_origin_img

        def make_coordinates(image, line_parameters):
            slope, intercept = line_parameters
            y1 = 479
            y2 = 0
            x1 = int((y1- intercept)/slope)
            x2 = int((y2 - intercept)/slope)
            return np.array([x1, y1, x2, y2])

        def average_slope_intercept(image, linesP):
            left_fit = []
            right_fit = []
            for line in linesP:
                x1, y1, x2, y2 = line.reshape(4)
                parameter = np.polyfit((x1, x2), (y1, y2), 1)
                slope = parameter[0]
                intercept = parameter[1]
                if slope < 0:
                    left_fit.append((slope, intercept))
                else:
                    right_fit.append((slope, intercept))
            left_fit_average =np.average(left_fit, axis=0)
            right_fit_average = np.average(right_fit, axis =0)
            left_line =make_coordinates(image, left_fit_average)
            right_line = make_coordinates(image, right_fit_average)

            return np.array([[left_line, right_line]])

        def show_lines(image, lines) : 
            lines_image = np.zeros_like(image)
            if linesP is not None :
                for i in range(len(lines)):
                    for x1,y1,x2,y2 in linesP[i]:
                        cv2.line(lines_image,(x1,y1),(x2,y2),(255,0,0), 10 )
            return lines_image



        averaged_lines = average_slope_intercept(lanelines_image, linesP)

        #선을 기울기 평균값으로 적용
        lines_image = show_lines(lanelines_image, averaged_lines)

        #원본 이미지에 라인 그리기
        combine_image = cv2.addWeighted(lanelines_image, 0.8, lines_image, 1, 1)

        cv2.imshow('ori', warp_origin_img)
        cv2.imshow("roi", lines_image)
        cv2.imshow("combined", combine_image)












        # <Line 추출 by HJ    end>





        # <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  start>
        cv2.waitKey(1) # wait 처리 안해주면 컴퓨터에서 opencv 열고 닫는 속도가 너무 빨라 못 봄
        # <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  end>














    

def main():
    webot_control = Webot_control()
    rospy.spin()

if __name__=="__main__":
    main()
