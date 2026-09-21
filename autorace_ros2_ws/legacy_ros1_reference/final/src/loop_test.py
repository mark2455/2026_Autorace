#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist



rospy.init_node("wego_node") 

pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=1)
pub_data = Twist()
sub_data = Pose()

def callback(sub_data):
    if sub_data.x < 9:
        pub_data.linear.x = 1
    else:
        pub_data.linear.x = 0
    
    pub.publish(pub_data)

    print(f"sub_data:\n{sub_data}")
rospy.Subscriber("/turtle1/pose",Pose,callback)
rospy.spin()

