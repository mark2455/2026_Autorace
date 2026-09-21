#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from math import *

class Webot_control:
    def __init__(self):
        rospy.init_node("webot_node") # node 이름 정하기
        self.webot_ctrl_pub = rospy.Publisher("/commands/servo/position",Float64,queue_size=1) # node 역할 정하기
        rospy.Subscriber("/scan",LaserScan,self.laser_CB)
        self.rate = rospy.Rate(10) # 주기설정
        self.laser_msg = LaserScan()
        self.safe_range = 0.30 # 전방 장애물 있을 경우, 장애물 앞 몇 m에서 정지신호 보낼지 지정
        self.detect_degree = 90 # 인덱스값으로 좌로 90도는 963 우로 90도는 321
        self.detect_range = 0.6
        self.sensitivity = 10
    def laser_CB(self,msg):
        #print(f"--------------------")
        num = 0    
        index_list=[]
        degree_list=[]
        left_degree=[]
        right_degree=[]
        degrees = [(msg.angle_min + msg.angle_increment * index)*180/pi for index, value in enumerate(msg.ranges)]
        for index, value in enumerate(msg.ranges):
            if self.detect_degree < abs(degrees[index]) and 0 < value <self.detect_range:
                degree_list.append(degrees[index])
        #print(degree_list)                                     #여기까지가 인지부
        for degree in degree_list:
            if degree > 0:
                right_degree.append(degree)
            else:
                left_degree.append(degree)
        num_right_degree = len(right_degree)
        num_left_degree = len(left_degree)
        print(f"right_index:{len(right_degree)}")
        print(f"left_index:{len(left_degree)}")                  #여기까지가 판단부
        if num_right_degree < self.sensitivity:
            num_right_degree = 0
        if num_left_degree < self.sensitivity:
            num_left_degree = 0

        if num_right_degree == 0 and num_left_degree == 0:
            steer =0.5
        elif num_right_degree > num_left_degree:
            steer = 0
        else :
            steer = 1
        self.webot_ctrl_pub.publish(steer)

        
        # if right_index !=[]:
        #     right_obj = max(right_index)
        #     print(f"right_obj:{right_obj}")
        #     print(f"right_space:{abs(right_obj-963)}")

        # if left_index !=[]:
        #     left_obj = min(left_index)
        #     print(f"left_obj:{left_obj}")
        #     print(f"left_space:{abs(left_obj-963)}")               




def main():
    webot_control = Webot_control()
    rospy.spin()


if __name__=="__main__":
    main()





        # print(degree_list)
        # if index_list != []: #예외처리, 인덱스가 빈 것이 아닐 경우만 나타내도록 (이거 안함 장애물 없을때만 error 표시 나올수도)
        #     for index in index_list:
        #         if index<322:
        #             left_index.append(index)
        #         elif index>963:
        #             right_index.append(index)
        #     os.system("clear")