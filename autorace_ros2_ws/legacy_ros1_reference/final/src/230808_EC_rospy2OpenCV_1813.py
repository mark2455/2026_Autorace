#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from math import *
import os # 명령창에서 실시간 값만 보이게 할려고 가져오는 놈인 듯 (5일차 오후 강의 중)
import cv2
from cv_bridge import CvBridge
import numpy as np



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
        cv2.imshow("h",h) #이미지 opencv 뷰어
        cv2.imshow("s",s) #이미지 opencv 뷰어
        cv2.imshow("v",v) #이미지 opencv 뷰어
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
        cv2.imshow("img_inrange", img_inrange)
        ### 동아리 방의 경우 lower_bound = np.array([185,190,175]) upper_bound = np.array([225,225,210])  
        # <이미지 HSV기준 추출 색상 정하기 end>


        # <ROI 설정: 공중에서 보는 View로 start>
        src = np.float32([[70,479],[224,347],[415,347],[569,479]])
        dst = np.float32([[70,479],[70,0],[569,0],[569,479]])
        matrix = cv2.getPerspectiveTransform(src,dst)

        warp_origin_img = cv2.warpPerspective(img_hsv, matrix, [img_hsv.shape[1],img_hsv.shape[0]])
        cv2.imshow("warp_origin_img", warp_origin_img)

        warp_img = cv2.warpPerspective(img_inrange, matrix, [img_hsv.shape[1],img_hsv.shape[0]])
        cv2.imshow("warp_inrange_img", warp_img)
        # <ROI 설정: 공중에서 보는 View로 end>


        # <>






        # <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  start>
        cv2.waitKey(1) # wait 처리 안해주면 컴퓨터에서 opencv 열고 닫는 속도가 너무 빨라 못 봄
        # <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  end>




    

def main():
    webot_control = Webot_control()
    rospy.spin()

if __name__=="__main__":
    main()
