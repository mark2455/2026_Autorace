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
        # <HSV 성분별 색깔 분리 start>      
        img_hsv = cv2.cvtColor(self.comp_img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)

        lower_bound = np.array([185, 190, 175])
        upper_bound = np.array([225, 225, 210])
        img_inrange = cv2.inRange(img_hsv, lower_bound, upper_bound)
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







        # ...

        # <Line 추출 by HJ    start>
        src = warp_img = cv2.warpPerspective(img_inrange, matrix, [img_hsv.shape[1], img_hsv.shape[0]])

        # Canny 엣지 검출
        dst = cv2.Canny(src, 50, 200, None, 3)

        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)

        # 확률적 허프 변환을 사용하여 선 검출
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

        if linesP is not None:
            # 좌측과 우측 그룹을 나누기 위한 임계값 (이 값은 상황에 따라 조정할 필요가 있습니다)
            mid_x = (img_hsv.shape[1] // 2)  # 이미지의 중간 x 좌표를 기준으로 좌우를 나눕니다.

            # 좌측 그룹과 우측 그룹 초기화
            left_group = []
            right_group = []

            # 시작점 및 끝점의 x 좌표 비교하여 그룹 나누기
            for line in linesP:
                x1, _, x2, _ = line[0]
                if x1 < mid_x and x2 < mid_x:
                    left_group.append(line)
                elif x1 >= mid_x and x2 >= mid_x:
                    right_group.append(line)

            # 좌측 그룹과 우측 그룹의 시작점 및 끝점 x 좌표 평균 계산
            def calculate_average(group):
                if len(group) == 0:
                    return None
                x1_avg = sum(line[0][0] for line in group) / len(group)
                x2_avg = sum(line[0][2] for line in group) / len(group)
                return int(x1_avg), int(x2_avg)

            left_avg_x1, left_avg_x2 = calculate_average(left_group)
            right_avg_x1, right_avg_x2 = calculate_average(right_group)

            # 평균값을 사용하여 좌측과 우측의 선 그리기
            if left_avg_x1 is not None and left_avg_x2 is not None:
                cv2.line(cdst, (left_avg_x1, 0), (left_avg_x2, img_hsv.shape[0]), (0, 0, 255), 3, cv2.LINE_AA)

            if right_avg_x1 is not None and right_avg_x2 is not None:
                cv2.line(cdst, (right_avg_x1, 0), (right_avg_x2, img_hsv.shape[0]), (0, 0, 255), 3, cv2.LINE_AA)
                
        # ...

        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdst)
        # <Line 추출 by HJ    end>










# <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  start>
cv2.waitKey(1) # wait 처리 안해주면 컴퓨터에서 opencv 열고 닫는 속도가 너무 빨라 못 봄
# <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  end>














    

def main():
    webot_control = Webot_control()
    rospy.spin()

if __name__=="__main__":
    main()
