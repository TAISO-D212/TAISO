#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from std_msgs.msg import Int16

# image_lane_roi 는 카메라 센서를 통하여 받아온 이미지에 관심있는 부분만(차선) 만 남기고
# 나머지 부분은 마스킹 하는 이미리 처리입니다. 관심 영역을 지정하고, 마스크를 생성, 마스크를 이미지에 합치는 과정을
# 합니다. 

class Stop_line_detector:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/front_lower_cam", CompressedImage, self.callback)
        self.pub = rospy.Publisher("/stopline", Int16, queue_size=1)
        x = 640
        y = 480
        self.red_lower = np.array([0, 0, 253])
        self.red_upper = np.array([0, 0, 255])
        self.np_arr = None
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            max_y = 0
            if self.np_arr is not None: 
                self.img_bgr = cv2.imdecode(self.np_arr, cv2.IMREAD_COLOR)
                self.img_bgr = cv2.inRange(self.img_bgr, self.red_lower, self.red_upper)
                
                lines = cv2.HoughLinesP(self.img_bgr, 1, np.pi/180, threshold=1, minLineLength=5, maxLineGap=100)
    
                if lines is not None:
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        # 검출된 선을 그림
                        cv2.line(self.img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        max_y = max(max_y,max(y1, y2))
                print(max_y)
                cv2.imshow("a", self.img_bgr)
            self.pub.publish(max_y)
            cv2.waitKey(2)
            rate.sleep()

    def callback(self, msg):
        self.np_arr = np.fromstring(msg.data, np.uint8)
        
if __name__ == '__main__':

    rospy.init_node('stop_line_detector', anonymous=True)

    image_parser = Stop_line_detector()

    rospy.spin() 