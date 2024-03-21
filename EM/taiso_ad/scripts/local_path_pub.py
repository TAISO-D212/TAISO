#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        
        self.odom_sub = rospy.Subscriber('/odom',Odometry, self.odom_callback)
        self.global_path_sub = rospy.Subscriber( 'global_path', Path, self.global_path_callback)

        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)

        self.is_odom = False
        self.is_path = False

        self.local_path_size = 70

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
   
            if self.is_odom == True and self.is_path == True:
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y

                min_dis = float('inf')
                current_waypoint = -1
                for idx  in range(0, len(self.global_path_msg.poses)) :
                    now_dist = sqrt(((self.global_path_msg.poses[idx].pose.position.x - self.x)**2)+((self.global_path_msg.poses[idx].pose.position.y - self.y)**2))
                    if now_dist < min_dis :
                        min_dis = now_dist
                        current_waypoint = idx
                    
                if current_waypoint != -1 :
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        for idx in range(current_waypoint, current_waypoint + self.local_path_size):
                            read_pose = PoseStamped()
                            read_pose.pose.position.x = self.global_path_msg.poses[idx].pose.position.x
                            read_pose.pose.position.y = self.global_path_msg.poses[idx].pose.position.y
                            read_pose.pose.orientation.w = 1
                            local_path_msg.poses.append(read_pose)
                    else :
                        for idx in range(current_waypoint, len(self.global_path_msg.poses)):
                            read_pose = PoseStamped()
                            read_pose.pose.position.x = self.global_path_msg.poses[idx].pose.position.x
                            read_pose.pose.position.y = self.global_path_msg.poses[idx].pose.position.y
                            read_pose.pose.orientation.w = 1
                            local_path_msg.poses.append(read_pose)

                print(x,y)
                self.local_path_pub.publish(local_path_msg)

            rate.sleep()


    def odom_callback(self,msg):
        self.is_odom = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg      


if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass

