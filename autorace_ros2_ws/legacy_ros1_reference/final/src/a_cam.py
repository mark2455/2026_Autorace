#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Image,CompressedImage
from math import *
import os # 명령창에서 실시간 값만 보이게 할려고 가져오는 놈인 듯 (5일차 오후 강의 중)
import cv2
from cv_bridge import CvBridge
import numpy as np


class Webot_control:
    def __init__(self):
            rospy.init_node("cam_node") # no 이름 정하기
            #rospy.Subscriber()
            rospy.Subscriber("usb_cam/image_rect_color/compressed",CompressedImage,self.comp_img_CB) # 잘림
            self.camera_pub = rospy.Publisher("velocity_id",Int32, queue_size=1)
            #self.webot_speed_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)
            self.cvbridge = CvBridge()
            self.comp_img = []



    def comp_img_CB(self,msg):
        self.comp_img = self.cvbridge.compressed_imgmsg_to_cv2(msg) #잘림 1:57:14
        hsv_img =cv2.cvtColor(self.comp_img,cv2.COLOR_BGR2HSV)   #잘림 1:57:14 지점에 나옴
        h,s,v = cv2.split(hsv_img)
        os.system("clear") 
        print(f"H_average: {np.average(h)}")
        print(f"S_average: {np.average(s)}")
        print(f"V_average: {np.average(v)}")
        cv2.imshow("comp_img",self.comp_img)
        cv2.waitKey(1)
        if np.average(v) < 52:
            self.speed = 0
            self.camera_pub.publish(0)
            print("stop")
        else :
            self.speed = 1000
            self.camera_pub.publish(1000)
            print("go")






def main():
    webot_control = Webot_control()
    rospy.spin()


if __name__=="__main__":
    main()















# #!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Float64, Int32
# from sensor_msgs.msg import Image,CompressedImage
# from math import *
# import os # 명령창에서 실시간 값만 보이게 할려고 가져오는 놈인 듯 (5일차 오후 강의 중)
# import cv2
# from cv_bridge import CvBridge
# import numpy as np


# class Webot_control:
#     def __init__(self):
#             rospy.init_node("cam_node") # no 이름 정하기
#             #rospy.Subscriber()
#             rospy.Subscriber("usb_cam/image_rect_color/compressed",CompressedImage,self.comp_img_CB) # 잘림
#             self.velocity_id_pub = rospy.Publisher("velocity_id",Int32, queue_size=1)
#             self.cvbridge = CvBridge()
#             self.comp_img = []



#     def comp_img_CB(self,msg):
#         self.comp_img = self.cvbridge.compressed_imgmsg_to_cv2(msg) #잘림 1:57:14
#         hsv_img =cv2.cvtColor(self.comp_img,cv2.COLOR_BGR2HSV)   #잘림 1:57:14 지점에 나옴
#         h,s,v = cv2.split(hsv_img)
#         os.system("clear") 
#         print(f"H_average: {np.average(h)}")
#         print(f"S_average: {np.average(s)}")
#         print(f"V_average: {np.average(v)}")
#         cv2.imshow("comp_img",self.comp_img)
#         cv2.waitKey(1)
#         if np.average(v) < 52:
#             speed = 0
#         else :
#             speed = 1000
#         self.velocity_id_pub.publish(speed)

# def main():
#     webot_control = Webot_control()
#     rospy.spin()


