#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import LaserScan
from math import *

class Choice_control:
    def __init__(self):
        rospy.init_node("Choice_node") # node 이름 정하기
        self.steer_rabacon_id_pub = rospy.Publisher("steer_rabacon_id",Int32, queue_size=1)
        self.steer_ordinary_id_pub = rospy.Publisher("steer_ordinary_id",Int32, queue_size=1)
        self.speed_obstacle_id_pub = rospy.Publisher("steer_obstacle_id",Int32, queue_size=1)
        self.speed_child_id_pub = rospy.Publisher("speed_child_id",Int32, queue_size=1)
        # rospy.Subscriber("aruco_id",Int32,self.aruco_node)



    def Choice_node(self,msg):
        # if 1==2: # steer raba on:
        #     self.steer_rabacon_id_pub.publish(1)
        # elif 3==4: # steer ordinary on:
        #     self.steer_ordinary_id_pub.publish(1)
        # # elif 5==5: # speed obstacle on:
        # #     self.speed_obstacle_id_pub.publish(1)
        # elif 5==6: # speed_child_id
        #     self.speed_child_id_pub.publish(1)
        # else: # speed obstacle on:
        self.speed_obstacle_id_pub.publish(1)
        print(self.speed_obstacle_id_pub)
        


            # speed = 1000
            # self.velocity_id_pub.publish(1000)
            # print("go")



def main():
    choice_control = Choice_control()
    rospy.spin()


if __name__=="__main__":
    main()
