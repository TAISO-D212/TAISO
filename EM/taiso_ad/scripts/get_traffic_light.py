#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from std_msgs.msg import Bool

# image_lane_roi 는 카메라 센서를 통하여 받아온 이미지에 관심있는 부분만(차선) 만 남기고
# 나머지 부분은 마스킹 하는 이미리 처리입니다. 관심 영역을 지정하고, 마스크를 생성, 마스크를 이미지에 합치는 과정을
# 합니다. 

class Get_traffic_light:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/front_upper_cam", CompressedImage, self.callback)
        self.pub = rospy.Publisher("/exist_traffic_light",Bool,queue_size=1)
        x = 640
        y = 480
        self.pink_lower = np.array([238, 72, 253])
        self.pink_upper = np.array([242, 76, 255])
        self.np_arr = None
        self.exist_traffic_light = False
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.np_arr is not None: 
                self.img_bgr = cv2.imdecode(self.np_arr, cv2.IMREAD_COLOR)
                self.img_bgr = cv2.inRange(self.img_bgr, self.pink_lower, self.pink_upper)
                self.exist_traffic_light = False
                for i in range(self.img_bgr.shape[0]):
                    for j in range(self.img_bgr.shape[1]):
                        if self.img_bgr[i][j] != 0:
                            self.exist_traffic_light = True
                            break
                print(self.exist_traffic_light)
                self.pub.publish(self.exist_traffic_light)
                cv2.imshow("a", self.img_bgr)
            cv2.waitKey(2)
            rate.sleep()

    def callback(self, msg):
        self.np_arr = np.fromstring(msg.data, np.uint8)
        

if __name__ == '__main__':

    rospy.init_node('get_traffic_light', anonymous=True)

    image_parser = Get_traffic_light()

    rospy.spin() 