#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Int32, String
from sensor_msgs.msg import Image, CompressedImage
from math import *
import os # 명령창에서 실시간 값만 보이게 할려고 가져오는 놈인 듯 (5일차 오후 강의 중)
# import cv2
# from cv_bridge import CvBridge
# import numpy as np
from fiducial_msgs.msg import Fiducial, FiducialArray



class Webot_control:
    def __init__(self):
            rospy.init_node("webot_node") # no 이름 정하기
            #rospy.Subscriber()
            self.sign_id = 0
            rospy.Subscriber("/fiducial_vertices", FiducialArray, self.comp_img_CB)
            self.sign_id_pub = rospy.Publisher("sign_id", Int32, queue_size=1)
            #self.webot_speed_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)
            self.pub_cnt = 0

    def comp_img_CB(self,_data):
        os.system("clear") 
        print(f"Child_Sign: {self.sign_id}")
        if (len(_data.fiducials) > 0 ) :
            self.sign_id = _data.fiducials[0].fiducial_id
            # rospy.loginfo("################## ID : {}".format(self.sign_id))
            self.sign_id_pub.publish(self.sign_id)
            self.pub_cnt = 0
            #speed = 1000
            #self.webot_speed_pub.publish(speed)

        else :
            self.pub_cnt += 1
            if self.pub_cnt > 20:
                self.sign_id_pub.publish(0)
                self.pub_cnt = 0
                #speed = 3000
                #self.webot_speed_pub.publish(speed)

        
                


def main():
    webot_control = Webot_control()
    rospy.spin()


if __name__=="__main__":
    main()








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