#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class Webot_control:
    def __init__(self):
        rospy.init_node("webot_node") # node 이름 정하기
        self.webot_ctrl_pub = rospy.Publisher("/commands/servo/position",Float64,queue_size=1) # node 역할 정하기
        self.rate = rospy.Rate(10) # 주기설정

    def run(self):
        steer_msg = Float64()
        steer_msg.data = 0.5 #조향각조정 (0.5가 중앙, 0으로 갈수록 왼쪽으로, 1로 갈수록 오른쪽으로 주행)
        self.webot_ctrl_pub.publish(steer_msg)
  
def main():
    webot_control = Webot_control()
    while not rospy.is_shutdown():
        webot_control.run()


if __name__=="__main__":
    main()
