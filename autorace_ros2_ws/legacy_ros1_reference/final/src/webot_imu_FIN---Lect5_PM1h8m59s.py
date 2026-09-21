#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from math import *
import os # 명령창에서 실시간 값만 보이게 할려고 가져오는 놈인 듯 (5일차 오후 강의 중)

class Webot_control:
    def __init__(self):
        rospy.init_node("webot_node") # node 이름 정하기
        rospy.Subscriber("/imu",Imu,self.imu_CB)
        imu_msg = Imu()
        imu_msg.orientation.x
        imu_msg.orientation.y
        imu_msg.orientation.z
        imu_msg.orientation.w
    def imu_CB(self, msg):
            imu_msg=msg
            os.system("clear")
            x = imu_msg.orientation.x
            y = imu_msg.orientation.y
            z = imu_msg.orientation.z
            w = imu_msg.orientation.w
            roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x,y,z,w)
            print(f"yaw_z:{yaw_z*180/pi}")

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

def main():
    webot_control = Webot_control()
    rospy.spin()


if __name__=="__main__":
    main()
