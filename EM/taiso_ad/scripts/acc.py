#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,Point32,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList,GetTrafficLightStatus
import numpy as np
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from std_msgs.msg import Bool, Float64, Int16, String, Float32MultiArray


class pure_pursuit :
    def __init__(self):
        # arg = rospy.myargv(argv=sys.argv)
        # object_topic_name = arg[1]

        # rospy.Subscriber(object_topic_name, ObjectStatusList, self.object_info_callback)

        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_light_status_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        rospy.Subscriber("/stopline", Int16, self.stopline_callback)
        rospy.Subscriber("/exist_traffic_light", Bool, self.traffic_light_callback)
        rospy.Subscriber("/intersection", String, self.intersection_callback)
        rospy.Subscriber('/route', Float32MultiArray, self.route_callback)
        
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=10)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_position = None
        
        self.vehicle_length = 2.9
        self.lfd = 8
        self.min_lfd = 8
        self.max_lfd = 30
        self.lfd_gain = 0.8
        self.target_velocity = 30
        self.dis = 999
        self.traffic_light = False
        self.stopline = 0
        self.traffic_light_status = GetTrafficLightStatus()
        self.pid = pidControl()
        self.adaptive_cruise_control = AdaptiveCruiseControl(velocity_gain = 0.1, distance_gain = 1, time_gap = 0.8, vehicle_length = 4)
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        self.check_waypoint = True
        self.was_zone = False
        self.waiting = False
        self.count = 0
        self.intersection = 'forward'
        self.status_msg = EgoVehicleStatus()
        self.route_msg = Float32MultiArray().data

        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 25)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(50) # 10hz
        while not rospy.is_shutdown():

            if self.is_global_path == True and self.is_path == True and self.is_odom == True and self.is_status == True:

                # global_obj,local_obj
                result = self.calc_vaild_obj(self.status_msg, self.object_data)
                
                global_npc_info = result[0] 
                local_npc_info = result[1] 
                global_ped_info = result[2] 
                local_ped_info = result[3] 
                # global_obs_info = result[4] 
                # local_obs_info = result[5] 
                # waypoints 
                #(105.124124535, 1156.1241256),
                # (118.887, 1498.312),
                # (130.277, 1561.641)
                
                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6
                # ego_status x,y 좌표와 waypoint x,y 좌표간의 차이가 일정값 이하일때 정지후 rospy.sleep(180) 아닐 때 원래 로직
                self.check_waypoint = False
                for i in range(len(self.route_msg)//2):
                    if 0 < (abs(self.current_position.x - self.route_msg[2*i]) + abs(self.current_position.y - self.route_msg[2*i+1])) < 2:
                        self.check_waypoint = True
                        break

                if self.check_waypoint:
                    if self.was_zone == False:
                        self.waiting = True
                    self.was_zone = True
                else:
                    self.was_zone = False
                
                if self.waiting == True:
                    self.target_velocity = 0.0
                    if self.count == 300:
                        self.count = 0
                        self.waiting = False
                    else:
                        self.count += 1
                else:
                    target_sign = 16
                    if self.intersection == 'left':
                        target_sign = 32

                    if self.traffic_light_status.header.stamp.secs == self.status_msg.header.stamp.secs and abs(self.status_msg.wheel_angle) < 5  and self.traffic_light_status.trafficLightStatus&target_sign == 0 and self.stopline:
                        if 0 < self.stopline < 80:
                            self.target_velocity = 10
                        elif self.stopline < 350 :
                            self.target_velocity = 5
                        elif self.stopline != 0:
                            self.target_velocity = 0.0
                    else:
                        self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6     
                 
                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering

                    self.adaptive_cruise_control.check_object(self.path, global_npc_info, local_npc_info, global_ped_info, local_ped_info)
                                                              #, global_obs_info, local_obs_info)
                    
                    self.target_velocity = self.adaptive_cruise_control.get_target_velocity(local_npc_info, local_ped_info, self.status_msg.velocity.x, self.target_velocity / 3.6)

                    output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)
                        
                    if self.target_velocity < 5:
                        output = -1

                    if output > 0.0:
                        self.ctrl_cmd_msg.accel = output
                        self.ctrl_cmd_msg.brake = 0.0
                    else:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = -output

                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.brake = 1.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
            rate.sleep()

    def path_callback(self,msg):
        self.is_path = True
        self.path = msg  

    def odom_callback(self,msg):
        self.is_odom = True
        self.current_position = Point()
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status = True
        self.status_msg = msg    
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True

    def object_info_callback(self,msg): ## Object information Subscriber
        self.is_object_info = True
        self.object_data = msg

    def stopline_callback(self,msg):
        self.stopline = msg.data

    def traffic_light_callback(self,msg):
        self.traffic_light = msg.data

    def traffic_light_status_callback(self, msg):
        self.traffic_light_status = msg

    def intersection_callback(self, msg):
        self.intersection = msg.data

    def route_callback(self,msg):
        self.route_msg = msg.data

    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')
        currnet_waypoint = -1

        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_vaild_obj(self,ego_status,object_data):
        
        self.all_object = object_data        
        ego_pose_x = ego_status.position.x
        ego_pose_y = ego_status.position.y
        ego_heading = self.vehicle_yaw
        
        global_npc_info = []
        local_npc_info  = []
        global_ped_info = []
        local_ped_info  = []
        # global_obs_info = []
        # local_obs_info  = []

        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_obstacle + self.all_object.num_of_pedestrian

        if num_of_object > 0:

            #translation
            tmp_theta = ego_heading
            tmp_translation = [ego_pose_x, ego_pose_y]
            tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                            [sin(tmp_theta),  cos(tmp_theta), tmp_translation[1]],
                            [0, 0, 1]])
            tmp_det_t = np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0]*tmp_translation[1])],
                                [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1]*tmp_translation[1])],
                                [0, 0, 1]])

            #npc vehicle translation        
            for npc_list in self.all_object.npc_list:
                global_result = np.array([[npc_list.position.x], [npc_list.position.y], [1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0] > 0 :        
                    global_npc_info.append([npc_list.type, npc_list.position.x, npc_list.position.y, npc_list.velocity.x])
                    local_npc_info.append([npc_list.type, local_result[0][0], local_result[1][0], npc_list.velocity.x])

            #ped translation
            for ped_list in self.all_object.pedestrian_list:
                global_result = np.array([[ped_list.position.x], [ped_list.position.y], [1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0] > 0 :
                    global_ped_info.append([ped_list.type, ped_list.position.x, ped_list.position.y, ped_list.velocity.x])
                    local_ped_info.append([ped_list.type, local_result[0][0], local_result[1][0], ped_list.velocity.x])

            #obs translation
            # for obs_list in self.all_object.obstacle_list:
            #     global_result = np.array([[obs_list.position.x], [obs_list.position.y], [1]])
            #     local_result = tmp_det_t.dot(global_result)
            #     if local_result[0][0] > 0 :
            #         global_obs_info.append([obs_list.type, obs_list.position.x, obs_list.position.y, obs_list.velocity.x])
            #         local_obs_info.append([obs_list.type, local_result[0][0], local_result[1][0], obs_list.velocity.x])
                
        return global_npc_info, local_npc_info, global_ped_info, local_ped_info
    # , global_obs_info, local_obs_info

    def calc_pure_pursuit(self,):

        self.lfd = (self.status_msg.velocity.x) * self.lfd_gain

        if self.lfd < self.min_lfd : 
            self.lfd=self.min_lfd
        elif self.lfd > self.max_lfd :
            self.lfd=self.max_lfd
            
        rospy.loginfo(self.lfd)

        
        vehicle_position=self.current_position
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]], 
                                 [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]], 
                                 [0, 0, 1]])

        det_trans_matrix = np.array([[trans_matrix[0][0], trans_matrix[1][0], -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])],
                                     [trans_matrix[0][1], trans_matrix[1][1], -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                     [0, 0, 1]])

        for num, i in enumerate(self.path.poses) :
            path_point = i.pose.position

            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0] > 0 :
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd :
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
                
        if self.target_velocity > 0.0:
            theta = atan2(local_path_point[1], local_path_point[0])
            steering = atan2(2 * self.vehicle_length * sin(theta), self.lfd)
        else:
            steering = 0.0
        return steering


class pidControl:
    def __init__(self):
        self.p_gain = 0.2
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        if(target_vel < 5):
            output = -1.0
            return output

        error = target_vel - current_vel

        # 목표 속도에 도달한 경우 브레이크 비활성화
        if abs(error) <= 5:  # 예: 목표 속도와의 오차가 5 km/h 이하인 경우
            output = 0
        else:
            p_control = self.p_gain * error
            self.i_control += self.i_gain * error * self.controlTime
            d_control = self.d_gain * (error - self.prev_error) / self.controlTime
            output = p_control + self.i_control + d_control

        self.prev_error = error

        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton # 도로 마찰계수

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y, 1])
                y_list.append((-x*x) - (y*y))

            A = np.array(x_list)
            B = np.array(y_list)
            a, b, c = np.dot(np.linalg.pinv(A), B)
            
            r = (a**2 + b**2 - c)**0.5

            v_max = (r * 9.8 * self.road_friction)**0.5 + 1
            
            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

class AdaptiveCruiseControl:
    def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length):
        self.npc_vehicle = [False, 0]
        self.object = [False, 0]
        self.Person = [False, 0]
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length

        self.object_type = None
        self.object_distance = 0
        self.object_velocity = 0

        # change code
        self.object_detected = False
        rospy.Subscriber('/object_detected', Bool, self.object_detected_callback)

    def object_detected_callback(self, msg):
        # 콜백 함수에서 객체 감지 여부를 갱신합니다.

        was_detected = self.object_detected
    
        # 콜백 함수에서 객체 감지 여부를 갱신합니다.
        self.object_detected = msg.data

        # 이전에 객체가 감지되었고 지금은 감지되지 않으면 관련 변수를 초기화합니다.
        if was_detected and not self.object_detected:
            self.npc_vehicle = [False, 0]
            self.object = [False, 0]
            self.Person = [False, 0]
        print(self.object_detected)

    def check_object(self, ref_path, global_npc_info, local_npc_info, 
                                    global_ped_info, local_ped_info):
        
        # 주행 경로 상 보행자 유무 파악
        min_rel_distance = float('inf')
        if len(global_ped_info) > 0 :        
            for i in range(len(global_ped_info)):
                for path in ref_path.poses :      
                    if global_ped_info[i][0] == 0 : # type=0 [pedestrian]
                        dis = sqrt(pow(path.pose.position.x - global_ped_info[i][1], 2) + pow(path.pose.position.y - global_ped_info[i][2], 2))
                        if dis < 0.5:                            
                            rel_distance = sqrt(pow(local_ped_info[i][1], 2) + pow(local_ped_info[i][2], 2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.Person = [True, i]

        # 주행 경로 상 NPC 차량 유무 파악
        self.npc_vehicle = [False,0]
        if len(global_npc_info) > 0 :            
            for i in range(len(global_npc_info)):
                for path in ref_path.poses :      
                    if global_npc_info[i][0] == 1 : # type=1 [npc_vehicle] 
                        dis = sqrt(pow(path.pose.position.x - global_npc_info[i][1], 2) + pow(path.pose.position.y - global_npc_info[i][2], 2))
                        if dis < 0.3:
                            rel_distance = sqrt(pow(local_npc_info[i][1], 2) + pow(local_npc_info[i][2], 2))       
                            if rel_distance < min_rel_distance and (-1.8 < path.pose.position.y - global_npc_info[i][2] < 1.8):
                                min_rel_distance = rel_distance
                                self.npc_vehicle=[True,i]

        # # 주행 경로 상 Obstacle 유무 파악
        # # acc 예제는 주행 중 전방에 차량에 속도에 맞춰 움직이도록 하는 Cruise Control
        # # 예제 이기 때문에 정적 장애물(Obstacle) 의 정보는 받지 않는게 좋습니다.
        # # 정적 장애물은 움직이지 않기 때문에 Cruise Control 알고리즘 상
        # # 정적 장애물을 만나게 되면 속도가 0인 정적 장애물 바로 뒤에 정지하게 됩니다.
        # if len(global_obs_info) > 0 :            
        #     for i in range(len(global_obs_info)):
        #         for path in ref_path.poses :      
        #             if global_obs_info[i][0] == 2 : # type=1 [obstacle] 
        #                 dis = sqrt(pow(path.pose.position.x - global_obs_info[i][1], 2) + pow(path.pose.position.y - global_obs_info[i][2], 2))
        #                 if dis < 0.5:
        #                     rel_distance = sqrt(pow(local_obs_info[i][1], 2) + pow(local_obs_info[i][2], 2))
        #                     if rel_distance < min_rel_distance:
        #                         min_rel_distance = rel_distance
        #                         self.object=[True,i] 
        

    def get_target_velocity(self, local_npc_info, local_ped_info, ego_vel, target_vel): 
        #TODO: (9) 장애물과의 속도와 거리 차이를 이용하여 ACC 를 진행 목표 속도를 설정
        out_vel =  target_vel
        default_space = 7
        self.time_gap = self.time_gap
        self.v_gain = self.velocity_gain
        self.x_errgain = self.distance_gain
        
        if self.npc_vehicle[0] and len(local_npc_info) != 0: #ACC ON_vehicle
            # print("ACC ON NPC_Vehicle")
            
            front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2], local_npc_info[self.npc_vehicle[1]][3]]

            self.object_distance = sqrt(pow(front_vehicle[0],2) + pow(front_vehicle[1],2))
            if front_vehicle[2] < 2.0:
                if self.object_distance < 1.8:
                    out_vel = 0
                    return out_vel
                
            velocity_error = ego_vel - front_vehicle[2]
            safe_distance = ego_vel*self.time_gap + default_space
            distance_error = safe_distance - self.object_distance

            acceleration = -(self.velocity_gain*velocity_error + self.distance_gain*distance_error)
            out_vel = min(ego_vel+acceleration, target_vel)

            if self.object_distance < default_space:
                out_vel = 0
                return out_vel

        # if self.Person[0] and len(local_ped_info) != 0 or self.object_detected == True: #ACC ON_Pedestrian
        if self.object_detected == True:
            print("ACC ON Pedestrian")
            
            # Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2], local_ped_info[self.Person[1]][3]]

            # self.object_distance = sqrt(pow(Pedestrian[0],2) + pow(Pedestrian[1],2))
            default_space = 15

            if self.object_distance < default_space:
                out_vel = 0
   
        # if self.object[0] and len(local_obs_info) != 0: #ACC ON_obstacle     
        #     print("ACC ON Obstacle")                    
        #     Obstacle = [local_obs_info[self.object[1]][1], local_obs_info[self.object[1]][2]]
            
        #     self.object_distance = sqrt(pow(Obstacle[0],2) + pow(Obstacle[1],2))
        #     default_space = 15

        #     if self.object_distance < default_space:
        #         out_vel = 0

        return out_vel * 3.6

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass