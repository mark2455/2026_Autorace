#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class Webot_control:
    def __init__(self):
        rospy.init_node("webo_node") # node 이름 정하기
        self.webot_ctrl_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1) # node 역할 정하기
        self.rate=rospy.Rate(10) # 주기설정

    def run(self):
        speed_msg = Float64()
        speed_msg.data = 1500 #모터 구동속도 지정. = 0 이면 정지 = 1000 보통속도 
        self.webot_ctrl_pub.publish(speed_msg)

def main():
    webot_control = Webot_control()
    while not rospy.is_shutdown():
        webot_control.run()

if __name__=="__main__":
    main()


### Notice: 5일차 오전 강의 36:04 지점에선 webot_control.py 이었다가, 나중에 (1:34:50 지점) webot_speed.py 로 파일명 바꾸니 참고바랍니다.