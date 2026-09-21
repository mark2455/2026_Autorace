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
        img = self.comp_img

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



        hsv = cv2.cvtColor(self.comp_img, cv2.COLOR_BGR2HSV)   #=========================================== img가 아닌 image로 해야지 않나? img 선언필요

        # 하얀색 차선을 검출하기 위한 범위 설정
        lower_white = np.array([0, 0, 100]) #([0, 0, 100])
        upper_white = np.array([255, 255, 255])

        # 범위 내의 픽셀을 마스킹하여 하얀색 차선 검출
        mask = cv2.inRange(hsv, lower_white, upper_white)
        result = cv2.bitwise_and(img, img, mask=mask)

        diameter = 15
        sigmaColor = 75
        sigmaSpace = 75

        # 미디언 필터 적용
        result = cv2.medianBlur(result, 11)

        # 가우시안 블러링 적용
        result = cv2.GaussianBlur(result, (13,13), 0)

        # 양방향 필터 적용
        result = cv2.bilateralFilter(result, diameter, sigmaColor, sigmaSpace)

        white_image = result
        

        #def draw_line_and_intersection(img, a, b, c, d):






        # <ROI 설정: 공중에서 보는 View로 start>
        src = np.float32([[0,480],[233,300],[407,300],[640,480]])
        dst = np.float32([[0,480],[0,0],[640,0],[640,480]])
        matrix = cv2.getPerspectiveTransform(src,dst)

        warp_origin_img = cv2.warpPerspective(img_hsv, matrix, [img_hsv.shape[1],img_hsv.shape[0]])
        cv2.imshow("warp_origin_img", warp_origin_img)

        transformed_img = cv2.warpPerspective(white_image, matrix, [img_hsv.shape[1],img_hsv.shape[0]])
        # cv2.imshow("warp_inrange_img", warp_img)
        # <ROI 설정: 공중에서 보는 View로 end>

        edges = cv2.Canny(transformed_img, threshold1=10, threshold2=20)





        x = int(edges.shape[1])
        y = int(edges.shape[0])  #np.array([[0, 480],[282, 1000], [358, 1000],[640, 480]]) #
        #범위지정                             
        section = np.array([[40, 200],[40, 2000], [600, 2000],[600, 200]])  #원래는  np.array([[40, 200],[40, 1000], [480, 1000],[480, 200]])
        # 마스크 생성
        mask = np.zeros_like(edges) 
        
        if len(edges.shape) > 2:
            # 이미지가 다중 채널 (칼라)인 경우, 채널 수에 맞게 색상 설정
            ignore_mask_color = (255,) * edges.shape[2]
        else:
            # 이미지가 단일 채널 (흑백)인 경우, 흰색으로 설정
            ignore_mask_color = 255

        
        # 관심 영역을 채움
        cv2.fillPoly(mask, np.int32([section]), ignore_mask_color)
        # 관심 영역만 남기고 나머지 부분 제거
        masked_image = cv2.bitwise_and(edges, mask)


        
        lines = cv2.HoughLinesP(masked_image, rho=1, theta=np.pi / 180, threshold=10, minLineLength=25, maxLineGap=10)



        left_x_coords = []
        left_y_coords = []
        right_x_coords = []
        right_y_coords = []

        lf_x = []
        lf_y = []
        rt_x = []
        rt_y = []

        if lines is not None:
            x_list = []
            min_x = None
            max_x = 0
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dmx = x2 -x1
                if dmx == 0:
                    dmx = 0.01
                    
                slope = (y2 - y1) / dmx
                

                if slope < 0:  # 왼쪽 차선
                    left_x_coords.extend([x1, x2])
                    left_y_coords.extend([y1, y2])
                    

                    lf_x = left_x_coords if not lf_x else []
                    lf_y = left_y_coords if not lf_y else []

                else:  # 오른쪽 차선
                    right_x_coords.extend([x1, x2])
                    right_y_coords.extend([y1, y2])

                    rt_x = right_x_coords if not rt_x else []
                    rt_y = right_y_coords if not rt_y else [] 

                cv2.line(transformed_img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                

                prev_coefficients = [0,0]
            

                # 왼쪽 차선에 대한 선형 회귀
                if len(left_x_coords) > 1 and len(left_y_coords) > 1:
                    left_coefficients = np.polyfit(left_x_coords, left_y_coords, 1)
                    a = left_coefficients[0]
                    b = left_coefficients[1]
                    prev_coefficients = left_coefficients  # 현재 계수를 이전 계수로 저장

                else:
                    a = prev_coefficients[0]
                    b = prev_coefficients[1]
                

                # 오른쪽 차선에 대한 선형 회귀
                if len(right_x_coords) > 1 and len(right_y_coords) > 1:
                    right_coefficients = np.polyfit(right_x_coords, right_y_coords, 1)
                    c = right_coefficients[0]
                    d = right_coefficients[1]
                    prev_coefficients = right_coefficients  # 현재 계수를 이전 계수로 저장

                else:
                    c = prev_coefficients[0]
                    d = prev_coefficients[1]    

                # 화면에 직선과 교점을 그립니다.
                
                if c == 0:
                    c = 0.0001    
                if a == 0:
                    a = 0.0001

                # 중간 선의 y 좌표를 계산합니다.
                mid_y = transformed_img.shape[0] * 3 // 5

                # y=c인 중간 선을 그립니다.
                cv2.line(transformed_img, (0, mid_y), (transformed_img.shape[1] - 1, mid_y), (0, 255, 0), 2)

                if a is not None and b is not None:
                    intersection_x = int((mid_y - b) / a)
                    intersection_y = mid_y
                    x_list.append(intersection_x)
                else:
                    x_list.append(112)
                # 오른쪽 차선의 교점을 계산하고 원을 그립니다.
                if c is not None and d is not None:
                    intersection_w = int((mid_y - d) / c)
                    intersection_z = mid_y

                    x_list.append(intersection_w)
                else:
                    x_list.append(448)

                # 화면의 중앙에 원을 그립니다.
                mid_x = transformed_img.shape[0] * 67 // 100
                cv2.circle(transformed_img, (mid_x, mid_y), 5, (0, 0, 255), -1)
            
            img = np.empty(shape=[0])

            # 화면의 중앙에 원을 그립니다.
            mid_x = img.shape[0] * 67 // 100
            mid_y = img.shape[0] * 3 // 5
            cv2.circle(transformed_img, (mid_x, mid_y), 5, (0, 0, 255), -1)

            first_value = x_list[0]
            for value in x_list:
                if first_value is None or value < first_value:
                    min_x = value
                if first_value is None or value > first_value:
                    max_x = value

            print(min_x, max_x)
            if min_x != None:
                cv2.circle(transformed_img, (min_x, mid_y), 5, (255, 0, 0), -1)
            cv2.circle(transformed_img, (max_x, mid_y), 5, (255, 0, 0), -1)
            

            # 왼쪽 차선과 오른쪽 차선의 평균 값을 계산
            left_lane_avg = np.mean(left_x_coords), np.mean(left_y_coords)
            right_lane_avg = np.mean(right_x_coords), np.mean(right_y_coords)
            


            # 평균 값이 NaN인 경우 예외 처리합니다.
            # 왼쪽 차선이 감지되지 않은 경우 처리할 내용을 작성합니다.
            # 예를 들어, 왼쪽 차선을 무시하고 오른쪽 차선만을 사용할 수 있습니다.
            if np.isnan(left_lane_avg[0]) or np.isnan(left_lane_avg[1]):
                pass

            # 오른쪽 차선이 감지되지 않은 경우 처리할 내용을 작성합니다.
            # 예를 들어, 오른쪽 차선을 무시하고 왼쪽 차선만을 사용할 수 있습니다.
            if np.isnan(right_lane_avg[0]) or np.isnan(right_lane_avg[1]):
                pass

            # 선들의 중심 값에 원을 표기
            if not np.isnan(left_lane_avg[0]) and not np.isnan(right_lane_avg[0]):
                avg_x = int((left_lane_avg[0] + right_lane_avg[0]) / 2)
                mid_y = img.shape[0] * 3 // 5
                cv2.circle(transformed_img, (avg_x, mid_y), 5, (0, 255, 0), -1)

            # # 이전 프레임에서의 조향각과 현재 프레임에서의 조향각 계산                                #=========================================
            # angle = (avg_x - mid_x) * 0.25

            # # 조향각에 따라 속도 조정
            # if angle != prev_angle:  # 조향각이 틀어졌을 때
            #     if angle > prev_angle:  # 조향각이 증가했을 때
            #         current_speed -= speed_step  # 속도를 감소
            #     else:  # 조향각이 감소했을 때
            #         current_speed += speed_step  # 속도를 증가

            #     current_speed = max(min_speed, min(target_speed, current_speed))  # 속도 범위 제한
            # else:  # 조향각이 틀어지지 않았을 때
            #     current_speed = target_speed  # 목표 속도로 설정

            # # 조향각 범위 제한
            # angle = max(-100, min(100, angle))

            # print(angle)            

            # # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
            # drive(angle, current_speed)

            # prev_angle = angle  # 이전 프레임의 조향각 업데이트

        # 결과 이미지 출력
        cv2.imshow("Original View", self.comp_img)

        cv2.imshow("h",h) #이미지 opencv 뷰어
        cv2.imshow("s",s) #이미지 opencv 뷰어
        cv2.imshow("v",v) #이미지 opencv 뷰어

        cv2.imshow("white_image", white_image)
        cv2.imshow("transformed_img", transformed_img)
        cv2.imshow("edges", edges)
        cv2.imshow("roi_image = masked_image", masked_image)


        # cv2.imshow("white_img", white_img)
        cv2.imshow("transformed_img", transformed_img)
        # cv2.imshow("edges", edges)
        # cv2.imshow("roi_image = masked_image", masked_image)

   





        # <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  start>
        cv2.waitKey(1) # wait 처리 안해주면 컴퓨터에서 opencv 열고 닫는 속도가 너무 빨라 못 봄
        # <OpenCV Viewer 볼 때 없으면 안되는 필수 항목 : waitKey  end>














    

def main():
    webot_control = Webot_control()
    rospy.spin()

if __name__=="__main__":
    main()
