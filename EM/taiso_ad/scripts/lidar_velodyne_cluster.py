#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
from math import atan2

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose
from sklearn.cluster import DBSCAN


class SCANCluster:
    def __init__(self):

        self.scan_sub = rospy.Subscriber("/lidar3D", PointCloud2, self.callback)

        self.cluster_pub = rospy.Publisher("clusters", PoseArray, queue_size=10)

        self.pc_np = None

        self.dbscan = DBSCAN(eps = 0.5, min_samples=3)

    
    def callback(self, msg):    
        self.pc_np = self.pointcloud2_to_xyz(msg)

        if len(self.pc_np) == 0:

            cluster_msg = PoseArray()

        else:
            pc_xy = self.pc_np[:, :2]

            db = self.dbscan.fit_predict(pc_xy)

            print(db)

            n_cluster = np.max(db) + 1

            cluster_msg = PoseArray()

            cluster_list = []

            
            for cluster in range(n_cluster):
                tmp_pose=Pose()
                tmp_pose.position.x = pc_xy[cluster][0]
                tmp_pose.position.y = pc_xy[cluster][1]

                cluster_msg.poses.append(tmp_pose)
                
        self.cluster_pub.publish(cluster_msg)

    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = (point[0]**2 + point[1]**2 + point[2]**2)**0.5
            angle = atan2(point[1], point[0])
            
            
            if point[0] > 0 and 1.50 > point[2] > -1.25 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)

        return point_np



if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)

    scan_cluster = SCANCluster()

    rospy.spin() 