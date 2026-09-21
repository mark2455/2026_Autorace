#!/usr/bin/env python3


import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class Loop_test():
    def __init__(self):
        rospy.init_node("wego_node")
        self.pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=1)
        rospy.Subscriber("/turtle1/pose",Pose,self.callback)
        self.pub_data = Twist()
        self.sub_data = Pose()

def callback(self,sub_data):
    if sub_data.x < 9:
        self.pub_data.linear.x = -1
    else:
        self.pub_data.linear.x = 0
    print(f"sub_data:\n{sub_data}")
    print("---------")
    self.pub.publish(self.pub_data)
    print(f"pub_data:\n{self.pub_data}")

def main():
    loop_test = Loop_test()
    rospy.spin()

if __name__=="__main__":
    main()


