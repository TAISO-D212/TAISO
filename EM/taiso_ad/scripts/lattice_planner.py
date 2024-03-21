#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from morai_msgs.msg  import EgoVehicleStatus,ObjectStatusList
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Path
import numpy as np


class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        object_topic_name = arg[1]

        rospy.Subscriber(object_topic_name,ObjectStatusList, self.object_callback)

        #rospy.Subscriber( "/global_path", Path, self.global_path_callback )
        rospy.Subscriber( "/local_path", Path, self.path_callback )
        #rospy.Subscriber( "/odom", Odometry, self.odom_callback )
        rospy.Subscriber( "/Ego_topic", EgoVehicleStatus, self.status_callback )
        rospy.Subscriber( "/Object_topic", ObjectStatusList, self.object_callback )

        self.lattice_path_pub = rospy.Publisher("/lattice_path", Path, queue_size=10)
        # self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)

        # self.ctrl_cmd_msg = CtrlCmd()
        # self.ctrl_cmd_msg.longlCmdType = 1 # 2?

        self.is_path = False
        self.is_status = False
        self.is_obj = False

        rate = rospy.Rate(50) # 30hz
        while not rospy.is_shutdown():

            if self.is_path and self.is_status and self.is_obj:
                if self.checkObject(self.local_path, self.object_data):
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(self.object_data, lattice_path)

                    #TODO: (7) lattice 경로 메세지 Publish
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def checkObject(self, ref_path, object_data):
        is_crash = False
        for obstacle in object_data.obstacle_list:
            for path in ref_path.poses:  
                dis = sqrt((path.pose.position.x - obstacle.position.x)**2 + (path.pose.position.y - obstacle.position.y)**2)
                if dis < 2.35: # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일때 충돌이라 판단.
                    is_crash = True
                    break

        return is_crash

    def collision_check(self, object_data, out_path):
        selected_lane = -1        
        lane_weight = [ 3, 2, 1, 1, 2, 3 ] #reference path 
        
        for obstacle in object_data.obstacle_list:                        
            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses :                                
                    dis = sqrt(pow(obstacle.position.x - path_pos.pose.position.x, 2) + pow(obstacle.position.y - path_pos.pose.position.y, 2))
                    if dis < 1.8:
                        lane_weight[path_num] = lane_weight[path_num] + 80

        selected_lane = lane_weight.index(min(lane_weight))                    
        return selected_lane

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg  
        
    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg

    def latticePlanner(self,ref_path, vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x * 3.6

        look_distance = int(vehicle_velocity * 0.15 * 2)

        
        if look_distance < 20 : #min 10m   
            look_distance = 20              

        if len(ref_path.poses) > look_distance :

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
             # 시작점(global_ref_start_point)을 기준으로 변환한 끝점(global_ref_end_point) 좌표
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])

            # 시작점(global_ref_start_point)을 기준으로 변환한 자동차(world_ego_vehicle_position) 좌표
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)

            # [-3.0, -1.75, -1, 1, 1.75, 3.0] [-40.0, -20.75, -10, 10, 20.75, 40.0]
            lane_off_set = [-2.5, -1.45, -0.85, 0.85, 1.45, 2.5]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            for end_point in local_lattice_points:
                # lane_change_3.py의 내용 참고
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                waypoints_x = []
                waypoints_y = []
                x_interval = 0.3 # 생성할 Path 의 Point 간격을 0.5 로 한다.
                x_start = 0
                x_end = end_point[0]

                y_start = 0.0
                y_end = end_point[1]

                x_num = x_end / x_interval
                # End Point 까지의 길이를 Point 간 간격으로 나눠 필요한 Point 의 수를 계산한다.
                # 계산된 Point 의 숫자 만큼 X 좌표를 생성한다.
                for i in range(x_start, int(x_num)) : 
                    waypoints_x.append(i * x_interval)

                # 3차 곡선을 이용한 주행 경로 생성
                d = local_ego_vehicle_position[1][0]
                c = 0.0
                b = 3.0 * (y_end - y_start) / x_end**2
                a = -2.0 * (y_end - y_start) / x_end**3

                for x in waypoints_x:
                    # 3 차 방정식 수식을 작성한다. (f(x) = a*x^3 + b*x^2 + c*x + d)
                    result = a*x**3 + b*x**2 + c*x + d
                    waypoints_y.append(result)
                    

                # ros path 메시지 형식 경로 데이터 생성
                temp_path = Path()
                temp_path.header.frame_id='/map'
                for i in range(0,len(waypoints_y)) :
                    local_result = np.array([[waypoints_x[i]],[waypoints_y[i]],[1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose=PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0.
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1

                    temp_path.poses.append(read_pose)

                out_path.append(temp_path)

            # Add_point            
            # 3 차 곡선 경로가 모두 만들어 졌다면 이후 주행 경로를 추가 합니다.
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)

            for i in range(len(out_path)):  
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])

        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass
