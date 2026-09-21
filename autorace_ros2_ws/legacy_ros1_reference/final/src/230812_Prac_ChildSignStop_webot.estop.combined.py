#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy #ros python 연동
from std_msgs.msg import Float64, Int32, String
from sensor_msgs.msg import Image, CompressedImage #sensor 메세지 가져오는 부분인데, usb_cam 부분을 이걸로 쓴 듯
from math import * #수학 패키지 
import os # 명령창에서 실시간 값만 보이게 할려고 가져오는 놈인 듯 (5일차 오후 강의 중)
from fiducial_msgs.msg import Fiducial, FiducialArray #Fiducial 패키지 (aruco 마커 인식) 가져오기 



class Webot_control:
    def __init__(self):
            rospy.init_node("webot_node") # no 이름 정하기
            self.sign_id = 0
            rospy.Subscriber("/fiducial_vertices", FiducialArray, self.comp_img_CB) #Fiducial로 aruco 마커 인식값 가져오는 Subscriber
            self.sign_id_pub = rospy.Publisher("sign_id", Int32, queue_size=1)
            self.webot_speed_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1) #모터 스피트 Publisher
            self.pub_cnt = 0

    def comp_img_CB(self,_data):
        os.system("clear") #터미널(명령창) 에서 현재 데이터만 보이도록 (깔끔하게 보기위해) 하는 코드
        print(f"Child_Sign: {self.sign_id}") # aruco 마커에 어떤 값이 나오는지 출력해주는 부분
        if (len(_data.fiducials) > 0 ) : 
            self.sign_id = _data.fiducials[0].fiducial_id
            # rospy.loginfo("################## ID : {}".format(self.sign_id))
            self.sign_id_pub.publish(self.sign_id) 
            self.pub_cnt = 0
            speed = 1000 # speed 란 변수에 1000 넣기
            self.webot_speed_pub.publish(speed) # speed란 변수에 있는 값을 받아서 모터에 보내는 코드

        else :
            self.pub_cnt += 1
            if self.pub_cnt > 20:
                self.sign_id_pub.publish(0)
                self.pub_cnt = 0
                speed = 3000 # speed 란 변수에 3000 넣기
                self.webot_speed_pub.publish(speed) # speed란 변수에 있는 값을 받아서 모터에 보내는 코드

        
                
# ROS토픽을 Class로 작성한 이상, 필수로 있어야 하는 부분     start

def main():
    webot_control = Webot_control()
    rospy.spin()


if __name__=="__main__":
    main()

# ROS토픽을 Class로 작성한 이상, 필수로 있어야 하는 부분     end








# !/usr/bin/env python
# -*- coding: utf-8 -*-

# import rospy

# from std_msgs.msg import Int32, String
# from sensor_msgs.msg import Image 
# from fiducial_msgs.msg import Fiducial, FiducialArray

# class Sign():
#     def __init__(self):
#         self.sign_id = 0
#         rospy.Subscriber("/fiducial_vertices", FiducialArray, self.child_sign_callback)
#         self.sign_id_pub = rospy.Publisher("sign_id", Int32, queue_size=1)
#         self.pub_cnt = 0

#     def child_sign_callback(self, _data):
#         if (len(_data.fiducials) > 0 ) :
#             self.sign_id = _data.fiducials[0].fiducial_id
#             # rospy.loginfo("################## ID : {}".format(self.sign_id))
#             self.sign_id_pub.publish(self.sign_id)
#             self.pub_cnt = 0
#         else :
#             self.pub_cnt += 1
#             if self.pub_cnt > 20:
#                 self.sign_id_pub.publish(0)
#                 self.pub_cnt = 0



# def run():
#     rospy.init_node("sign_id")
#     new_class = Sign()
#     rospy.spin()


# if __name__ == '__main__':
#     run()