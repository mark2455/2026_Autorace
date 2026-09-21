#!/usr/bin/env python3

import rospy #ros python 연동
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage, Image #sensor 메세지 가져오는 부분인데, usb_cam 부분을 이걸로 쓴 듯
from math import *
import os # 명령창에서 실시간 값만 보이게 할려고 가져오는 놈인 듯 (5일차 오후 강의 중)
import cv2 # opencv2 읽어오는 부위 
from cv_bridge import CvBridge #ros 와 opencv 연동 관련
import numpy as np # ?





class Webot_control:
    def __init__(self):
            rospy.init_node("webot_node") # 노드 이름 정하기
            rospy.Subscriber("usb_cam/image_rect_color/compressed",CompressedImage,self.comp_img_CB) #usb_cam 영상 Subscriber
            self.webot_speed_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1) #모터 스피트 Publisher
            self.cvbridge = CvBridge() #ros 와 opencv 연동 관련
            self.comp_img = [] #이 부분은 없어도 동작 하는 것 같음



    def comp_img_CB(self,msg):

        # <이미지 필수 전처리 과정 start>
        self.comp_img = self.cvbridge.compressed_imgmsg_to_cv2(msg) #usb_cam 영상을 opencv로 보내주는 역할
        os.system("clear") #터미널(명령창) 에서 현재 데이터만 보이도록 (깔끔하게 보기위해) 하는 코드
        # <이미지 필수 전처리 과정 end>




        # <ROI 설정: 공중에서 보는 View로 start>
        img_original = self.comp_img #(img_original이란 변수에 self.comp_img의 내용(영상)을 넣는다)

        src = np.float32([[70,479],[224,347],[415,347],[569,479]]) #(usb_cam 영상에서 ROI를 어디로 잡을 것인지 좌표 선정)
        dst = np.float32([[70,479],[70,0],[569,0],[569,479]]) #(ROI로 잡은 영역을 사각형 형태로 어떻게 필지 좌표 지정)

        matrix = cv2.getPerspectiveTransform(src,dst) #(ROI로 지정한 src좌표를, 사각형 형태 dst좌표로 변환시키는(펴주는) 행렬)

        warp_origin_img = cv2.warpPerspective(img_original, matrix, [img_original.shape[1],img_original.shape[0]]) 
        #warp_origin_img는 ROI부분만 나온 이미지로,  cv2.warpPerspective: img_original(usb_cam영상)을 matrix 행렬대로 ROI부분을 사각형 형태로 펴주는 명령어

        cv2.imshow("warp_origin_img", warp_origin_img) #cv2.imshow("창 제목", 창에 넣을 이미지 파일 )
        # <ROI 설정: 공중에서 보는 View로 end>




        # <이미지 인덱스 값 출력부위 start>
        h,s,v = cv2.split(warp_origin_img) # cv2.split(이미지파일) : 영상을 h,s,v로 분할(split)하는 코드

        print(f"명도_average: {np.average(v)}") # warp_origin_img의 명도 v(0~255 사이)의 텍스트 값을 터미널(명령창)에 출력해주는 부위


        # cv2.imshow("h",h) #이미지 opencv 뷰어
        # cv2.imshow("s",s) #이미지 opencv 뷰어
        # cv2.imshow("v",v) #이미지 opencv 뷰어
        # print(msg)
        # <이미지 인덱스 값 출력 end>



        # <횡단보도 코드 start>
        if (np.average(v) > 105) :  # roi의 평균 명도가 105 보다 크면(=밝으면), 
            speed = 0 # speed 란 변수에 0 넣기
        else : # roi의 평균 명도가 105 보다 작으면(=어두우면), 
            speed = 1000 # speed 란 변수에 1000 넣기

        self.webot_speed_pub.publish(speed) # speed란 변수에 있는 값을 받아서 모터에 보내는 코드
        # <횡단보도 코드 end>


        # <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  start>
        cv2.waitKey(1) # wait 처리 안해주면 컴퓨터에서 opencv 열고 닫는 속도가 너무 빨라 못 봄
        # <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  end>




# ROS토픽을 Class로 작성한 이상, 필수로 있어야 하는 부분     start

def main():
    webot_control = Webot_control()
    rospy.spin()

if __name__=="__main__":
    main()

# ROS토픽을 Class로 작성한 이상, 필수로 있어야 하는 부분     end

