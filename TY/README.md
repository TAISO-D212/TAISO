# 1. 위치 인식

### 1-1 Localization(좌표변환 (WGS84 -> UTM))

```py
# gps_parser.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import os
#TODO: (0) pyproj 라이브러리 Import [ pip install pyproj ]
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage, EgoVehicleStatus

# gps_parser 는 GPS의 위경도 데이터를 UTM 좌표로 변환하는 예제입니다.
# Pyproj 라이브러리를 사용

# 노드 실행 순서
# 1. 변환 하고자 하는 좌표계를 선언
# 2. 시뮬레이터에서 GPS 데이터를 받아오는 Callback 함수 생성
# 3. 위도 경도 데이터를 UTM 좌표로 변환
# 4. 위도 경도 데이터와 변환한 UTM 좌표를 터미널 창에 출력 하여 확인

class LL2UTMConverter:
    def __init__(self, zone=52) :
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        # 초기화
        self.x, self.y = None, None

        # TODO: (1) 변환 하고자 하는 좌표계를 선언
        '''
        # GPS 센서에서 수신되는 위도, 경도 데이터를 UTM 좌표료 변환 하기 위한 예제이다.
        # 해당 예제는 WGS84 좌표계에서 UTM 좌표계로의 변환을 진행한다.
        # 시뮬레이터 K-City Map 의 경우 UTM 좌표계를 사용하며 실제 지도 상 위치는 UTM 좌표계의 52 Zone 에 존재한다.
        # 맵 좌표계는 m 단위를 사용한다.
        # 아래 주소의 링크를 클릭하여 Ptoj 의 사용 방법을 확인한다.
        # https://pyproj4.github.io/pyproj/stable/api/proj.html
        # " proj= , zone= , ellps =  , preserve_units = "
        self.proj_UTM = Proj( 좌표 변환을 위한 변수 입력 )

        '''
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)


    #TODO: (2) 시뮬레이터에서 GPS 데이터를 받아오는 Callback 함수 생성
    def navsat_callback(self, gps_msg):
        '''
        GPS 센서에서 수신되는 위도 경도 데이터를 확인한다.
        self.lat =
        self.lon =

        '''
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude

        self.convertLL2UTM()

        utm_msg = Float32MultiArray()

        #TODO: (4) 위도 경도 데이터와 변환한 UTM 좌표를 터미널 창에 출력 하여 확인
        '''
        UTM 으로 변환 된 좌표 데이터와 위도 경도 데이터를 터미널 창에 출력되도록 한다.
        utm_msg.data = [self.x, self.y]
        os.system('clear')
        print(' lat : ', 위도 데이터)
        print(' lon : ', 경도 데이터)
        print(' utm X : ', utm 좌표로 변환한 x 좌표)
        print(' utm Y : ', utm 좌표로 변환한 y 좌표)

        '''
        utm_msg.data = [self.x, self.y]
        os.system('clear')
        print(' lat : ', self.lat)
        print(' lon : ', self.lon)
        print(' utm X : %15.10f' %utm_msg.data[0])
        print(' utm Y : %15.10f' %utm_msg.data[1])


    #TODO: (3) 위도 경도 데이터를 UTM 좌표로 변환
    def convertLL2UTM(self):
        '''
        # pyproj 라이브러리를 이용해 정의한 좌표 변환 변수를 이용하여 위 경도 데이터를 변환한다.
        xy_zone = self.proj_UTM(위도 데이터, 경도 데이터)

        self.x = xy_zone[0]
        self.y = xy_zone[1]

        '''
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0]
        self.y = xy_zone[1]
        print(xy_zone)

if __name__ == '__main__':

    rospy.init_node('gps_parser', anonymous=True)

    gps_parser = LL2UTMConverter()

    rospy.spin()
```

</br>

### 1-2 Odometry ROS 메시지 생성

```py
# gpsimu_parser.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from math import pi

# gpsimu_parser 는 GPS, IMU 센서 데이터를 받아 차량의 상대위치를 추정하는 예제입니다.

# 노드 실행 순서
# 1. 변환 하고자 하는 좌표계를 선언
# 2. 송신 될 Odometry 메세지 변수 생성
# 3. 위도 경도 데이터 UTM 죄표로 변환
# 4. Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
# 5. Odometry 메세지 Publish

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        # 초기화
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False

        #TODO: (1) 변환 하고자 하는 좌표계를 선언
        '''
        # GPS 센서에서 수신되는 위도, 경도 데이터를 UTM 좌표료 변환 하기 위한 예제이다.
        # 해당 예제는 WGS84 좌표계에서 UTM 좌표계로의 변환을 진행한다.
        # 시뮬레이터 K-City Map 의 경우 UTM 좌표계를 사용하며 실제 지도 상 위치는 UTM 좌표계의 52 Zone 에 존제한다.
        # 맵 좌표계는 m 단위를 사용한다.
        # 아래 주소의 링크를 클릭하여 Ptoj 의 사용 방법을 확인한다.
        # https://pyproj4.github.io/pyproj/stable/api/proj.html
        # " proj= , zone= , ellps =  , preserve_units = "
        self.proj_UTM = Proj( 좌표 변환을 위한 변수 입력 )

        '''
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        #TODO: (2) 송신 될 Odometry 메세지 변수 생성
        '''
        # ROS 메세지 중 물체의 위치와 자세 데이터를 나타내는 Odometry 메세지를 사용한다.
        # 차량의 현재 위치와 자세 데이터를 GPS IMU 센서에 담아서 Publsih 한다.
        # 이때 frame_id 는 '/odom' child_frame_id 는 '/base_link' 로 한다.

        self.odom_msg =
        self.odom_msg.header.frame_id =
        self.odom_msg.child_frame_id =

        '''
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_imu==True and self.is_gps == True:
                self.convertLL2UTM()

                #TODO: (5) Odometry 메세지 Publish
                '''
                # Odometry 메세지 를 전송하는 publisher 를 만든다.
                self.odom_pub.

                '''
                self.odom_pub.publish(self.odom_msg)
                os.system('clear')
                print(" ROS Odometry Msgs Pose ")
                print(self.odom_msg.pose.pose.position)
                print(" ROS Odometry Msgs Orientation ")
                print(self.odom_msg.pose.pose.orientation)

                rate.sleep()

    def navsat_callback(self, gps_msg):

        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.is_gps=True

    #TODO: (3) 위도 경도 데이터 UTM 죄표로 변환
    def convertLL2UTM(self):
        '''
        # pyproj 라이브러리를 이용해 정의한 좌표 변환 변수를 이용하여 위 경도 데이터를 변환한다.
        # 변환 시 이전 gps_parser.py 예제와 달리 시뮬레이터 GPS 센서의 offset 값을 적용 한다.
        # GPS 센서에서 출력되는 Offset 값은 시뮬레이터에 맵 좌표계로 변경을 위한 값이다.
        # UTM 좌표로 변환 된 x, y 값에 offset 값을 빼주면 된다.
        xy_zone = self.proj_UTM(위도 데이터, 경도 데이터)

        # if 문을 이용 예외처리를 하는 이유는 시뮬레이터 음영 구간 설정 센서 데이터가 0.0 으로 나오기 때문이다.
        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o

        '''
        xy_zone = self.proj_UTM(self.lon, self.lat)
        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o

        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        '''
        # Offset 을 적용하여 시뮬레이터 맵 좌표계 값으로 변환 된 좌표 데이터를 Odometry 메세지에 넣는다.
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x =
        self.odom_msg.pose.pose.position.y =
        self.odom_msg.pose.pose.position.z =

        '''
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0

    def imu_callback(self, data):

        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        '''
        # IMU 를 통해 받은 물체의 자세 데이터를 Odometry 메세지에 넣는다.
        # if 문을 이용 예외처리를 하는 이유는 시뮬레이터 음영 구간 설정 센서 데이터가 0.0 으로 나오기 때문이다.
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w

        '''
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w
        self.is_imu=True

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
```

</br>

### 1-3 ROS TF 좌표계 생성

```py
# tf_pub.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from math import pi
from nav_msgs.msg import Odometry

# tf 는 물체의 위치와 자세 데이터를 좌표계로 나타내는 예제입니다.

# 노드 실행 순서
# 1. Callback 함수 생성
# 2. 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅

class Ego_listener():
    def __init__(self):
        rospy.init_node('status_listener', anonymous=True)

        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.spin()


    #TODO: (1) Callback 함수 생성
    def odom_callback(self,odom_msg):
        self.is_odom = True

        '''
        # gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
        # Odometry 메세지 에 담긴 물체의 위치 와 자세 데이터를 아래 변수에 넣어준다.
        self.x = 물체의 x 좌표
        self.y = 물체의 y 좌표

        self.orientation_x = 물체의 quaternion x 값
        self.orientation_y = 물체의 quaternion y 값
        self.orientation_z = 물체의 quaternion z 값
        self.orientation_w = 물체의 quaternion ㅈ 값

        '''
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y

        self.orientation_x = odom_msg.pose.pose.orientation.x
        self.orientation_y = odom_msg.pose.pose.orientation.y
        self.orientation_z = odom_msg.pose.pose.orientation.z
        self.orientation_w = odom_msg.pose.pose.orientation.w

        #TODO: (2) 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅
        '''
        # TF 데이터를 broadcast 해주는 변수를 선언한다.
        # TF 데이터에 물체의 좌표와 자세 데이터를 시간 그리고 Frame ID 를 넣어주면 된다.
        # TF 예제는 map 좌표 를 기준으로 Ego 차량의 위치를 좌표를 나타낸다
        br = tf.TransformBroadcaster()
        br.sendTransform((x 좌표, y 좌표, z 좌표),
                        (물체의 quaternion x 값,물체의 quaternion y 값,물체의 quaternion z 값,물체의 quaternion w 값),
                        rospy.Time.now(),
                        "Ego",
                        "map")

        '''
        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 0),
                         (self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w),
                         rospy.Time.now(),
                         "Ego", "map")

if __name__ == '__main__':
    try:
        tl=Ego_listener()
    except rospy.ROSInternalException:
        pass
```

---

# 2. 정밀도로 지도

### 2-1-1 MGeo 데이터 확인

```py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo 는 정밀도로지도 데이터 인 MGeo(MORAI Geometry) 데이터를 읽어오는 예제입니다.
# Json 파일 형식으로 되어 있는 MGeo 데이터를 dictionary 형태로 읽어옵니다.

# 노드 실행 순서
# 1. Mgeo data 읽어온 후 데이터 확인

#TODO: (1) Mgeo data 읽어온 후 데이터 확인
'''
# Json 파일 형식으로 저장된 MGeo 데이터를 읽어오는 예제 입니다.
# VScode 의 debug 기능을 이용하여 MGeo 데이터를 확인 할 수 있습니다.
# MGeo 데이터는 인접 리스트 방식의 그래프 구조 입니다.
# 정밀도로지도의 도로 간의 연결 관계를 표현 합니다.
# MGeo 에는 도로의 형상을 나타내는 Node 와 Link 데이터가 있습니다.
# Node 와 Link 는 모두 Point 데이터 들의 집합입니다.
# Node 는 서로 다른 두개 이상의 Link 간의 연결 여부를 나타냅니다.
# Link 는 도로를 표현하며 도로 의 중심 선이 됩니다.
# Link 와 Node 정보가 모여 도로의 형상을 표현합니다.
# 각각의 Node Link 정보는 이름인 idx 정보를 가집니다 idx 는 중복 될 수 없습니다.
# to_links , from_links , to_node , from_node ... 등
# MGeo에 정의되어 있는 데이터를 활용해 각 Node 와 Link 간 연결 성을 나타낼 수 있습니다.
#

'''
load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
mgeo_planner_map = MGeo.create_instance_from_json(load_path)

node_set = mgeo_planner_map.node_set
link_set = mgeo_planner_map.link_set
nodes=node_set.nodes
links=link_set.lines

print('# of nodes: ', len(node_set.nodes))
print('# of links: ', len(link_set.lines))
```

### 2-1-2 MGeo 시각화

```py
# mgeo_pub.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point32
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo_pub 은 Mgeo 데이터를 읽어온 뒤 도로 정보를 Point Cloud Data 로 변환하는 예제입니다.
# Point Cloud 형식으로 변환 후 Rviz 를 이용해 정밀도로지도 데이터를 시각화 할 수 있습니다.

# 노드 실행 순서
# 1. Mgeo data 읽어온 후 데이터 확인
# 2. Link 정보 Point Cloud 데이터로 변환
# 3. Node 정보 Point Cloud 데이터로 변환
# 4. 변환한 Link, Node 정보 Publish

class get_mgeo :
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        self.link_pub = rospy.Publisher('link',PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('node',PointCloud, queue_size=1)

        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        '''
        # Json 파일 형식으로 저장된 MGeo 데이터를 읽어오는 예제 입니다.
        # VScode 의 debug 기능을 이용하여 MGeo 데이터를 확인 할 수 있습니다.
        # MGeo 데이터는 인접 리스트 방식의 그래프 구조 입니다.
        # 정밀도로지도의 도로 간의 연결 관계를 표현 합니다.
        # MGeo 에는 도로의 형상을 나타내는 Node 와 Link 데이터가 있습니다.
        # Node 와 Link 는 모두 Point 데이터 들의 집합입니다.
        # Node 는 서로 다른 두개 이상의 Link 간의 연결 여부를 나타냅니다.
        # Link 는 도로를 표현하며 도로 의 중심 선이 됩니다.
        # Link 와 Node 정보가 모여 도로의 형상을 표현합니다.
        # 각각의 Node Link 정보는 이름인 idx 정보를 가집니다 idx 는 중복 될 수 없습니다.
        # to_links , from_links , to_node , from_node ... 등
        # MGeo에 정의되어 있는 데이터를 활용해 각 Node 와 Link 간 연결 성을 나타낼 수 있습니다.

        '''
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes=node_set.nodes
        self.links=link_set.lines

        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()

        print('# of nodes: ', len(node_set.nodes))
        print('# of links: ', len(link_set.lines))

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            #TODO: (4) 변환한 Link, Node 정보 Publish
            '''
            # 변환한 Link, Node 정보 를 전송하는 publisher 를 만든다.
            self.link_pub.
            self.node_pub.

            '''
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)
            rate.sleep()


    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'

        #TODO: (2) Link 정보 Point Cloud 데이터로 변환
        '''
        # Point Cloud 형식으로 Link 의 좌표 정보를 변환합니다.
        # Link 의 개수 만큼 반복하는 반복 문을 이용해 Link 정보를 Point Cloud 형식 데이터에 넣습니다.

        for link_idx in self.links :
            for  in :


        '''
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:
                tmp_point=Point32()
                tmp_point.x=link_point[0]
                tmp_point.y=link_point[1]
                tmp_point.z=link_point[2]
                all_link.points.append(tmp_point)

        return all_link

    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'

        #TODO: (3) Node 정보 Point Cloud 데이터로 변환
        '''
        # Point Cloud 형식으로 Node 의 좌표 정보를 변환합니다.
        # Node 의 개수 만큼 반복하는 반복 문을 이용해 Node 정보를 Point Cloud 형식 데이터에 넣습니다.

        for node_idx in self.nodes :

        '''
        for node_idx in self.nodes :
            tmp_point=Point32()
            tmp_point.x=self.nodes[node_idx].point[0]
            tmp_point.y=self.nodes[node_idx].point[1]
            tmp_point.z=self.nodes[node_idx].point[2]
            all_node.points.append(tmp_point)

        return all_node


if __name__ == '__main__':

    test_track=get_mgeo()
```

# 3. 경로 계획

### 3-1 Odometry를 이용한 차량의 주행 경로 기록

```py
# path_maker.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry

# path_maker 는 차량의 위치 데이터를 받아 txt 파일로 저장하는 예제입니다.
# 저장한 txt 파일은 차량의 주행 경로가 되며 경로 계획에 이용 할 수 있습니다.

# 노드 실행 순서
# 1. 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
# 2. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 3. 콜백함수에서 이전 위치와 현재 위치의 거리 계산
# 4. 이전 위치보다 0.5m 이상일 때 위치를 저장

class pathMaker :
    def __init__(self, pkg_name = 'ssafy_2', path_name = 'make_path'):
        rospy.init_node('path_maker', anonymous=True)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 초기화
        self.prev_x = 0
        self.prev_y = 0
        self.is_odom=False

        #TODO: (1) 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
        '''
        # Path 데이터를 기록 하고 저장 할 경로와 txt 파일의 이름을 정한다.
        # 이후 쓰기 모드로 연다.
        # pkg_name 과 path_name 은 22 번 줄 참고한다.
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = 이곳에 txt 파일이 저장될 경로와 이름을 적는다
        self.f =

        '''
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = '{0}/{1}/path.txt'.format(pkg_path, path_name)
        print(full_path)
        self.f = open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_odom == True :
                # Ego 위치 기록
                self.path_make()
        self.f.close()

    def path_make(self):
        x = self.x
        y = self.y
        z = 0.0
        #TODO: (3) 콜백함수에서 이전 위치와 현재 위치의 거리 계산
        '''
        # 현재 차량의 위치와 이전에 지나온 위치의 좌표 데이터를 구한다.
        # 구해진 좌표 사이의 거리를 계산한다.
        # 이전 위치 좌표는 아래 #TODO: (4)에서 정의 한다.
        distance =

        '''
        distance = sqrt(((x-self.prev_x)**2)+((y-self.prev_y)**2))

        #TODO: (4) 이전 위치보다 0.5m 이상일 때 위치를 저장
        if distance >0.5:
            '''
            # distance 가 0.5 보다 커야지만 동작한다.
            # 현재 위치 좌표를 data 에 담은 뒤 txt 파일로 작성한다.
            # data 는 문자열 이며 x y z 사이는 \t 로 구분한다
            data ='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data 변수를 넣는 위치이다)
            self.prev_x =
            self.prev_y =
            self.prev_z =

            print(기록 된 위치 좌표를 출력한다)
            '''
            data ='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x = self.x
            self.prev_y = self.y
            self.prev_z = 0.0
            print(data)

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (2) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장

        '''
        # gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
        # Odometry 메세지 에 담긴 물체의 위치 데이터를 아래 변수에 넣어준다.
        self.x = 물체의 x 좌표
        self.y = 물체의 y 좌표

        '''
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

if __name__ == '__main__' :
    try:
        p_m=pathMaker()
    except rospy.ROSInternalException:
        pass
```

<br>

### 3-2 저장된 주행 경로를 읽어 차량의 global Path(전역경로) 생성

```py
# global_path_pub.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

# global_path_pub 은 txt 파일로 저장한 Path 데이터를 global Path (전역경로) 로 읽어오는 예제입니다.
# 만들어진 global Path(전역경로) 는 Local Path (지역경로) 를 만드는데 사용 된다.

# 노드 실행 순서
# 1. Global Path publisher 선언 및 Global Path 변수 생성
# 2. 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기
# 3. 읽어 온 경로 데이터를 Global Path 변수에 넣기
# 4. Global Path 정보 Publish


class global_path_pub :
    def __init__(self, pkg_name = 'ssafy_2', path_name = 'make_path'):
        rospy.init_node('global_path_pub', anonymous = True)

        #TODO: (1) Global Path publisher 선언 및 Global Path 변수 생성
        '''
        # Global Path 데이터를 Publish 하는 변수와 메세지를 담고있는 변수를 선언한다.
        # 이때 Global Path 는 map 좌표계를 기준으로 생성한다.
        self.global_path_pub =
        self.global_path_msg =
        self.global_path_msg.header.frame_id =

        '''
        self.global_path_pub = rospy.Publisher('global_path', Path, queue_size=1)
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id ='map'

        #TODO: (2) 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기
        '''
        # Path 데이터가 기록 된 txt 파일의 경로와 이름을 정한다.
        # 이후 읽기 모드로 연다.
        # pkg_name 과 path_name 은 21 번 줄 참고한다.
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path =
        self.f =
        lines = self.f.readlines()
        '''
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = '{0}/{1}/path.txt'.format(pkg_path, path_name)
        self.f = open(full_path,"r")
        lines = self.f.readlines()

        #TODO: (3) 읽어 온 경로 데이터를 Global Path 변수에 넣기
        '''
        # 읽어온 x y z 좌표 데이터를 self.global_path_msg 변수에 넣는다.
        # 넣어준 반복 문을 이용하여 작성한다.
        for line in lines :
            tmp = line.split()
            read_pose =
            read_pose.pose.position.x =
            read_pose.pose.position.y =
            read_pose.pose.orientation.w = 1
            self.global_path_msg.poses.append(read_pose)
        self.f.close()

        '''
        for line in lines:
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1
            self.global_path_msg.poses.append(read_pose)
        self.f.close()

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (4) Global Path 정보 Publish
            '''
            # Global Path 메세지 를 전송하는 publisher 를 만든다.
            self.global_path_pub.

            '''
            self.global_path_pub.publish(self.global_path_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        test_track = global_path_pub()
    except rospy.ROSInterruptException:
        pass
```

<br>

### 3-3 global Path(전역경로)를 이용한 local Path(지역경로) 생성

```py
# local_path_pub.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

# local_path_pub 은 global Path (전역경로) 데이터를 받아 Local Path (지역경로) 를 만드는 예제입니다.
# Local Path (지역경로) 는 global Path(전역경로) 에서 차량과 가장 가까운 포인트를 시작으로 만들어 집니다.

# 노드 실행 순서
# 1. Global Path 와 Odometry 데이터 subscriber 생성
# 2. Local Path publisher 선언
# 3. Local Path 의 Size 결정
# 4. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 5. Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
# 6. 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리
# 7. Local Path 메세지 Publish


class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        #TODO: (1) Global Path 와 Odometry 데이터 subscriber 생성
        '''
        # Global Path 와 Odometry 데이터 subscriber 를 생성한다.
        # 콜백 함수의 이름은 self.global_path_callback, self.odom_callback 로 한다.
        rospy.Subscriber( odometry 메세지 콜백 완성하기 )
        rospy.Subscriber( global path 메세지 콜백 완성하기 )

        '''
        self.odom_sub = rospy.Subscriber('/odom',Odometry, self.odom_callback)
        self.global_path_sub = rospy.Subscriber( 'global_path', Path, self.global_path_callback)

        #TODO: (2) Local Path publisher 선언
        '''
        # local Path 데이터를 Publish 하는 변수를 선언한다.
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)
        '''
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)

        # 초기화
        self.is_odom = False
        self.is_path = False

        #TODO: (3) Local Path 의 Size 결정
        '''
        # Local Path 의 크기를 지정한다.
        # 차량이 주행 시 Local Path 의 크기 만큼의 정보를 가지고 주행하게 된다
        # 너무 작지도 크기지도 않은 값을 사용한다 (50 ~ 200)
        self.local_path_size =
        '''
        self.local_path_size = 70

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():

            if self.is_odom == True and self.is_path == True:
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'

                x=self.x
                y=self.y

                #TODO: (5) Global Path 에서 차량 위치와 가장 가까운 포인트(current Waypoint) 탐색
                '''
                # global Path 에서 차량의 현재 위치를 찾습니다.
                # 현제 위치는 WayPoint 로 기록하며 현재 차량이 Path 에서 몇번 째 위치에 있는지 나타내는 값이 됩니다.
                # 차량의 현재 위치는 Local Path 를 만드는 시작 위치가 됩니다.
                # 차량의 현재 위치를 탐색하는 반복문은 작성해 current_waypoint 찾습니다.
                min_dis = float('inf')
                current_waypoint = -1
                for  in  :
                '''
                min_dis = float('inf')
                current_waypoint = -1
                for idx  in range(0, len(self.global_path_msg.poses)) :
                    now_dist = sqrt(((self.global_path_msg.poses[idx].pose.position.x - self.x)**2)+((self.global_path_msg.poses[idx].pose.position.y - self.y)**2))
                    if now_dist < min_dis :
                        min_dis = now_dist
                        current_waypoint = idx



                #TODO: (6) 가장 가까운 포인트(current Waypoint) 위치부터 Local Path 생성 및 예외 처리
                '''
                # 차량의 현재 위치 부터 local_path_size 로 지정한 Path 의 크기 만큼의 Path local_path 를 생성합니다.
                # 차량에 남은 Path 의 길이가 local_path_size 보다 작은 경우가 있음으로 조건 문을 이용하여 해당 조건을 예외 처리 합니다.
                if current_waypoint != -1 :
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):

                    else :
                '''
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
                #TODO: (7) Local Path 메세지 Publish
                '''
                # Local Path 메세지 를 전송하는 publisher 를 만든다.
                self.local_path_pub.
                '''
                self.local_path_pub.publish(local_path_msg)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장

        '''
        # gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
        # Odometry 메세지 에 담긴 물체의 위치 데이터를 아래 변수에 넣어준다.
        self.x = 물체의 x 좌표
        self.y = 물체의 y 좌표
        '''
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

```

<br>

### 3-4-1 MGeo 데이터에 Dijkstra 알고리즘 적용한 Global Path(전역경로) 생성[Dijstra-1]

```py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json

from math import cos,sin,sqrt,pow,atan2,pi
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo_dijkstra_path_1 은 Mgeo 데이터를 이용하여 시작 Node 와 목적지 Node 를 지정하여 Dijkstra 알고리즘을 적용하는 예제 입니다.
# 사용자가 직접 지정한 시작 Node 와 목적지 Node 사이 최단 경로 계산하여 global Path(전역경로) 를 생성 합니다.

# 노드 실행 순서
# 0. 필수 학습 지식
# 1. Mgeo data 읽어온 후 데이터 확인
# 2. 시작 Node 와 종료 Node 정의
# 3. weight 값 계산
# 4. Dijkstra Path 초기화 로직
# 5. Dijkstra 핵심 코드
# 6. node path 생성
# 7. link path 생성
# 8. Result 판별
# 9. point path 생성
# 10. dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
# 11. dijkstra 이용해 만든 Global Path 정보 Publish

#TODO: (0) 필수 학습 지식
'''
# dijkstra 알고리즘은 그래프 구조에서 노드 간 최단 경로를 찾는 알고리즘 입니다.
# 시작 노드부터 다른 모든 노드까지의 최단 경로를 탐색합니다.
# 다양한 서비스에서 실제로 사용 되며 인공 위성에도 사용되는 방식 입니다.
# 전체 동작 과정은 다음과 같습니다.
#
# 1. 시작 노드 지정
# 2. 시작 노드를 기준으로 다른 노드와의 비용을 저장(경로 탐색 알고리즘에서는 비용이란 경로의 크기를 의미)
# 3. 방문하지 않은 노드들 중 가장 적은 비용의 노드를 방문
# 4. 방문한 노드와 인접한 노드들을 조사해서 새로 조사된 최단 거리가 기존 발견된 최단거리 보다 작으면 정보를 갱신
#   [   새로 조사된 최단 거리 : 시작 노드에서 방문 노드 까지의 거리 비용 + 방문 노드에서 인접 노드까지의 거리 비용    ]
#   [   기존 발견된 최단 거리 : 시작 노드에서 인접 노드까지의 거리 비용                                       ]
# 5. 3 ~ 4 과정을 반복
#

'''
class dijkstra_path_pub :
    def __init__(self):
        rospy.init_node('dijkstra_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size = 1)

        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes=node_set.nodes
        self.links=link_set.lines

        self.global_planner=Dijkstra(self.nodes,self.links)

        #TODO: (2) 시작 Node 와 종료 Node 정의
        '''
        # Dijkstra Path 를 만들기 위한 출발 Node 와 도착 Node의 Idx 를 지정합니다.
        # MGeo 데이터는 Json 파일로 Idx 정보를 확인 할 수 있지만 시뮬레이터를 통해 직접 확인 가능합니다.
        # F8 키보드 입력 또는 시뮬레이터에서 화면 좌측 상단에 View --> MGeo Viewer 를 클릭합니다.
        # MGeo Viewer 기능을 이용하여 맵에있는 MGeo 정보를 확인 할 수 있으며 시각화 까지 가능합니다.
        # 해당 기능을 이용하여 원하는 시작 위치와 종료 위치의 Node 이름을 알아낸 뒤 아래 변수에 입력하세요.

        self.start_node = 'A119BS010184'
        self.end_node = 'A119BS010148'
        '''
        self.start_node = 'A119BS010184'
        self.end_node = 'A119BS010148'


        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.global_path_msg = self.calc_dijkstra_path_node(self.start_node, self.end_node)

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (11) dijkstra 이용해 만든 Global Path 정보 Publish
            '''
            # dijkstra 이용해 만든 Global Path 메세지 를 전송하는 publisher 를 만든다.
            self.global_path_pub.

            '''
            self.global_path_pub.publish(self.global_path_msg)

            rate.sleep()

    def calc_dijkstra_path_node(self, start_node, end_node):

        result, path = self.global_planner.find_shortest_path(start_node, end_node)

        #TODO: (10) dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'
        '''
        # dijkstra 경로 데이터 중 Point 정보를 이용하여 Path 데이터를 만들어 줍니다.

        '''
        for point in path['point_path'] :
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(point[0])
            read_pose.pose.position.y = float(point[1])
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        return out_path

class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        #TODO: (3) weight 값 계산
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    def get_weight_matrix(self):
        #TODO: (3) weight 값 계산
        '''
        # weight 값 계산은 각 Node 에서 인접 한 다른 Node 까지의 비용을 계산합니다.
        # 계산된 weight 값 은 각 노드간 이동시 발생하는 비용(거리)을 가지고 있기 때문에
        # Dijkstra 탐색에서 중요하게 사용 됩니다.
        # weight 값은 딕셔너리 형태로 사용 합니다.
        # 이중 중첩된 딕셔너리 형태로 사용하며
        # Key 값으로 Node의 Idx Value 값으로 다른 노드 까지의 비용을 가지도록 합니다.
        # 아래 코드 중 self.find_shortest_link_leading_to_node 를 완성하여
        # Dijkstra 알고리즘 계산을 위한 Node와 Node 사이의 최단 거리를 계산합니다.

        '''
        # 초기 설정
        weight = dict()
        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 다른 노드로 진행하는 모든 weight
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            # 전체 weight matrix에 추가
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 현재 노드로는 cost = 0
            weight[from_node_id][from_node_id] = 0

            for to_node in from_node.get_to_nodes():
                # 현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
                weight[from_node_id][to_node.idx] = min_cost

        return weight

    def find_shortest_link_leading_to_node(self, from_node, to_node):
        """현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다"""
        #TODO: (3) weight 값 계산
        '''
        # 최단거리 Link 인 shortest_link 변수와
        # shortest_link 의 min_cost 를 계산 합니다.

        '''
        to_links = []
        for link in from_node.get_to_links():
            if link.to_node is to_node:
                to_links.append(link)

        shortest_link = None
        min_cost = float('inf')

        for link in to_links:
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link

        return shortest_link, min_cost

    def find_nearest_node_idx(self, distance, s):
        idx_list = self.nodes.keys()
        min_value = float('inf')
        min_idx = idx_list[-1]

        for idx in idx_list:
            if distance[idx] < min_value and s[idx] == False :
                min_value = distance[idx]
                min_idx = idx
        return min_idx

    def find_shortest_path(self, start_node_idx, end_node_idx):
        #TODO: (4) Dijkstra Path 초기화 로직
        # s 초기화         >> s = [False] * len(self.nodes)
        # from_node 초기화 >> from_node = [start_node_idx] * len(self.nodes)
        '''
        # Dijkstra 경로 탐색을 위한 초기화 로직 입니다.
        # 변수 s와 from_node 는 딕셔너리 형태로 크기를 MGeo의 Node 의 개수로 설정합니다.
        # Dijkstra 알고리즘으로 탐색 한 Node 는 변수 s 에 True 로 탐색하지 않은 변수는 False 로 합니다.
        # from_node 의 Key 값은 Node 의 Idx로
        # from_node 의 Value 값은 Key 값의 Node Idx 에서 가장 비용이 작은(가장 가까운) Node Idx로 합니다.
        # from_node 통해 각 Node 에서 가장 가까운 Node 찾고
        # 이를 연결해 시작 노드부터 도착 노드 까지의 최단 경로를 탐색합니다.

        '''
        s = dict()
        from_node = dict()
        for node_id in self.nodes.keys():
            s[node_id] = False
            from_node[node_id] = start_node_idx

        s[start_node_idx] = True
        distance =copy.deepcopy(self.weight[start_node_idx])

        #TODO: (5) Dijkstra 핵심 코드
        for i in range(len(self.nodes.keys()) - 1):
            selected_node_idx = self.find_nearest_node_idx(distance, s)
            s[selected_node_idx] = True
            for j, to_node_idx in enumerate(self.nodes.keys()):
                if s[to_node_idx] == False:
                    distance_candidate = distance[selected_node_idx] + self.weight[selected_node_idx][to_node_idx]
                    if distance_candidate < distance[to_node_idx]:
                        distance[to_node_idx] = distance_candidate
                        from_node[to_node_idx] = selected_node_idx

        #TODO: (6) node path 생성
        tracking_idx = end_node_idx
        node_path = [end_node_idx]

        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)

        node_path.reverse()

        #TODO: (7) link path 생성
        link_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]

            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]

            shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
            link_path.append(shortest_link.idx)

        #TODO: (8) Result 판별
        if len(link_path) == 0:
            return False, {'node_path': node_path, 'link_path':link_path, 'point_path':[]}

        #TODO: (9) point path 생성
        point_path = []
        # print(link_path)
        for link_id in link_path:

            link = self.links[link_id]
            for point in link.points:
                point_path.append([point[0], point[1], 0])

        return True, {'node_path': node_path, 'link_path':link_path, 'point_path':point_path}

if __name__ == '__main__':

    dijkstra_path_pub = dijkstra_path_pub()
```

<br>

### 3-4-1 MGeo 데이터에 Dijkstra 알고리즘 적용한 Global Path(전역경로) 생성[Dijstra-2]

```py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json

from math import cos,sin,sqrt,pow,atan2,pi
from geometry_msgs.msg import Point32,PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo_dijkstra_path_1 은 Mgeo 데이터를 이용하여 시작 Node 와 목적지 Node 를 지정하여 Dijkstra 알고리즘을 적용하는 예제 입니다.
# 사용자가 직접 지정한 시작 Node 와 목적지 Node 사이 최단 경로 계산하여 global Path(전역경로) 를 생성 합니다.
# 시작 Node 와 목적지 Node 는 Rviz 의 goal pose / initial pose 두 기능을 이용하여 정의합니다.

# 노드 실행 순서
# 0. 필수 학습 지식
# 1. Mgeo data 읽어온 후 데이터 확인
# 2. 시작 Node 와 종료 Node 정의
# 3. weight 값 계산
# 4. Dijkstra Path 초기화 로직
# 5. Dijkstra 핵심 코드
# 6. node path 생성
# 7. link path 생성
# 8. Result 판별
# 9. point path 생성
# 10. dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
# 11. dijkstra 이용해 만든 Global Path 정보 Publish

#TODO: (0) 필수 학습 지식
'''
# dijkstra 알고리즘은 그래프 구조에서 노드 간 최단 경로를 찾는 알고리즘 입니다.
# 시작 노드부터 다른 모든 노드까지의 최단 경로를 탐색합니다.
# 다양한 서비스에서 실제로 사용 되며 인공 위성에도 사용되는 방식 입니다.
# 전체 동작 과정은 다음과 같습니다.
#
# 1. 시작 노드 지정
# 2. 시작 노드를 기준으로 다른 노드와의 비용을 저장(경로 탐색 알고리즘에서는 비용이란 경로의 크기를 의미)
# 3. 방문하지 않은 노드들 중 가장 적은 비용의 노드를 방문
# 4. 방문한 노드와 인접한 노드들을 조사해서 새로 조사된 최단 거리가 기존 발견된 최단거리 보다 작으면 정보를 갱신
#   [   새로 조사된 최단 거리 : 시작 노드에서 방문 노드 까지의 거리 비용 + 방문 노드에서 인접 노드까지의 거리 비용    ]
#   [   기존 발견된 최단 거리 : 시작 노드에서 인접 노드까지의 거리 비용                                       ]
# 5. 3 ~ 4 과정을 반복
#

'''
class dijkstra_path_pub :
    def __init__(self):
        rospy.init_node('dijkstra_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size = 1)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_callback)


        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes=node_set.nodes
        self.links=link_set.lines

        self.global_planner=Dijkstra(self.nodes,self.links)

        self.is_goal_pose = False
        self.is_init_pose = False

        while True:
            if self.is_goal_pose == True and self.is_init_pose == True:
                break
            else:
                rospy.loginfo('Waiting goal pose data')
                rospy.loginfo('Waiting init pose data')


        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.global_path_msg = self.calc_dijkstra_path_node(self.start_node, self.end_node)

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (11) dijkstra 이용해 만든 Global Path 정보 Publish
            '''
            # dijkstra 이용해 만든 Global Path 메세지 를 전송하는 publisher 를 만든다.
            self.global_path_pub.

            '''
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()

    def init_callback(self,msg):
        #TODO: (2) 시작 Node 와 종료 Node 정의
        # 시작 Node 는 Rviz 기능을 이용해 지정한 위치에서 가장 가까이 있는 Node 로 한다.
        '''
        # Rviz 의 2D Pose Estimate 기능을 이용해 시작 Node를 지정합니다.
        # Rviz 창에서 2D Pose Estimate 기능 클릭 후 마우스 좌 클릭을 통해 원하는 위치를 지정할 수 있습니다.
        # 출발 위치를 2D Pose Estimate 지정 하면 Rviz 에서
        # PoseWithCovarianceStamped 형식의 ROS 메세지를 Publish 합니다.
        # 해당 형식의 메세지를 Subscribe 해서  2D Pose Estimate 로 지정한 위치와 가장 가까운 노드를 탐색하는 합니다.
        # 가장 가까운 Node 가 탐색 된다면 이를 "self.start_node" 변수에 해당 Node Idx 를 지정합니다.

        self.start_node = node_idx

        '''
        min_dist = float('inf')
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        for node_idx in self.nodes:
            node_x = self.nodes[node_idx].point[0]
            node_y = self.nodes[node_idx].point[1]
            dist = sqrt(((x-node_x)**2)+((y-node_y)**2))
            if min_dist > dist:
                min_dist = dist
                self.start_node = node_idx

        self.is_init_pose = True

    def goal_callback(self,msg):
        #TODO: (2) 시작 Node 와 종료 Node 정의
        # 종료 Node 는 Rviz 기능을 이용해 지정한 위치에서 가장 가까이 있는 Node 로 한다.
        '''
        # Rviz 의 2D Nav Goal 기능을 이용해 도착 Node를 지정합니다.
        # Rviz 창에서 2D Nav Goal 기능 클릭 후 마우스 좌 클릭을 통해 원하는 위치를 지정할 수 있습니다.
        # 도착 위치를 2D Nav Goal 지정 하면 Rviz 에서
        # PoseStamped 형식의 ROS 메세지를 Publish 합니다.
        # 해당 형식의 메세지를 Subscribe 해서  2D Nav Goal 로 지정한 위치와 가장 가까운 노드를 탐색하는 합니다.
        # 가장 가까운 Node 가 탐색 된다면 이를 "self.start_node" 변수에 해당 Node Idx 를 지정합니다.

        self.end_node = node_idx

        '''
        min_dist = float('inf')
        x = msg.pose.position.x
        y = msg.pose.position.y
        for node_idx in self.nodes:
            node_x = self.nodes[node_idx].point[0]
            node_y = self.nodes[node_idx].point[1]
            dist = sqrt(((x-node_x)**2)+((y-node_y)**2))
            if min_dist > dist:
                min_dist = dist
                self.end_node = node_idx

        self.is_goal_pose = True

    def calc_dijkstra_path_node(self, start_node, end_node):

        result, path = self.global_planner.find_shortest_path(start_node, end_node)

        #TODO: (10) dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'
        '''
        # dijkstra 경로 데이터 중 Point 정보를 이용하여 Path 데이터를 만들어 줍니다.

        '''
        for waypoint in path['point_path']:
            read_pose = PoseStamped()
            read_pose.pose.position.x = waypoint[0]
            read_pose.pose.position.y = waypoint[1]
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)
        return out_path

class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    def get_weight_matrix(self):
        #TODO: (3) weight 값 계산
        '''
        # weight 값 계산은 각 Node 에서 인접 한 다른 Node 까지의 비용을 계산합니다.
        # 계산된 weight 값 은 각 노드간 이동시 발생하는 비용(거리)을 가지고 있기 때문에
        # Dijkstra 탐색에서 중요하게 사용 됩니다.
        # weight 값은 딕셔너리 형태로 사용 합니다.
        # 이중 중첩된 딕셔너리 형태로 사용하며
        # Key 값으로 Node의 Idx Value 값으로 다른 노드 까지의 비용을 가지도록 합니다.
        # 아래 코드 중 self.find_shortest_link_leading_to_node 를 완성하여
        # Dijkstra 알고리즘 계산을 위한 Node와 Node 사이의 최단 거리를 계산합니다.

        '''
        # 초기 설정
        weight = dict()
        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 다른 노드로 진행하는 모든 weight
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            # 전체 weight matrix에 추가
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 현재 노드로는 cost = 0
            weight[from_node_id][from_node_id] = 0

            for to_node in from_node.get_to_nodes():
                # 현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
                weight[from_node_id][to_node.idx] = min_cost

        return weight

    def find_shortest_link_leading_to_node(self, from_node,to_node):
        """현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다"""
        #TODO: (3) weight 값 계산
        '''
        # 최단거리 Link 인 shortest_link 변수와
        # shortest_link 의 min_cost 를 계산 합니다.

        '''
        to_links = []
        for link in from_node.get_to_links():
            if link.to_node is to_node:
                to_links.append(link)

        shortest_link = None
        min_cost = float('inf')

        for link in to_links:
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link

        return shortest_link, min_cost

    def find_nearest_node_idx(self, distance, s):
        idx_list = self.nodes.keys()
        min_value = float('inf')
        min_idx = idx_list[-1]

        for idx in idx_list:
            if distance[idx] < min_value and s[idx] == False :
                min_value = distance[idx]
                min_idx = idx
        return min_idx

    def find_shortest_path(self, start_node_idx, end_node_idx):
        #TODO: (4) Dijkstra Path 초기화 로직
        # s 초기화         >> s = [False] * len(self.nodes)
        # from_node 초기화 >> from_node = [start_node_idx] * len(self.nodes)
        '''
        # Dijkstra 경로 탐색을 위한 초기화 로직 입니다.
        # 변수 s와 from_node 는 딕셔너리 형태로 크기를 MGeo의 Node 의 개수로 설정합니다.
        # Dijkstra 알고리즘으로 탐색 한 Node 는 변수 s 에 True 로 탐색하지 않은 변수는 False 로 합니다.
        # from_node 의 Key 값은 Node 의 Idx로
        # from_node 의 Value 값은 Key 값의 Node Idx 에서 가장 비용이 작은(가장 가까운) Node Idx로 합니다.
        # from_node 통해 각 Node 에서 가장 가까운 Node 찾고
        # 이를 연결해 시작 노드부터 도착 노드 까지의 최단 경로를 탐색합니다.

        '''
        s = dict()
        from_node = dict()
        for node_id in self.nodes.keys():
            s[node_id] = False
            from_node[node_id] = start_node_idx

        s[start_node_idx] = True
        distance =copy.deepcopy(self.weight[start_node_idx])

        #TODO: (5) Dijkstra 핵심 코드
        for i in range(len(self.nodes.keys()) - 1):
            selected_node_idx = self.find_nearest_node_idx(distance, s)
            s[selected_node_idx] = True
            for j, to_node_idx in enumerate(self.nodes.keys()):
                if s[to_node_idx] == False:
                    distance_candidate = distance[selected_node_idx] + self.weight[selected_node_idx][to_node_idx]
                    if distance_candidate < distance[to_node_idx]:
                        distance[to_node_idx] = distance_candidate
                        from_node[to_node_idx] = selected_node_idx

        #TODO: (6) node path 생성
        tracking_idx = end_node_idx
        node_path = [end_node_idx]

        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)

        node_path.reverse()

        #TODO: (7) link path 생성
        link_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]

            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]

            shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
            link_path.append(shortest_link.idx)

        #TODO: (8) Result 판별
        if len(link_path) == 0:
            return False, {'node_path': node_path, 'link_path':link_path, 'point_path':[]}

        #TODO: (9) point path 생성
        point_path = []
        for link_id in link_path:
            link = self.links[link_id]
            for point in link.points:
                point_path.append([point[0], point[1], 0])

        return True, {'node_path': node_path, 'link_path':link_path, 'point_path':point_path}

if __name__ == '__main__':

    dijkstra_path_pub = dijkstra_path_pub()
```

---

# 4. 판단 제어

### 4-1. Pure pursuit 알고리즘을 적용한 횡 방향 제어

```py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
import numpy as np
import tf

# pure_pursuit 은 차량의 차량의 횡 방향 제어 예제입니다.
# 차량이 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 차량의 제어 입력은 CtrlCmd 메세지를 이용 하며 종방향 제어 입력은 longlCmdType 2(Velocity control) 이용하여 등속 운동 하도록 합니다.

# 노드 실행 순서
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 좌표 변환 행렬 생성
# 3. Steering 각도 계산
# 4. 제어입력 메세지 Publish

#TODO: (0) 필수 학습 지식
'''
# Pure Pursuit 은 차량의 Kinematic Model Based 경로 추종 알고리즘 입니다.
# 현재 차량의 Heading 각도와 실제 Path 의 각도 오차를 계산하여 차량의 조향 각도를 계산합니다.
# 전방주시거리(Look Forward Distance) 라는 변수를 가지고 있습니다.
# 전방주시거리는 Pure Pursuit 알고리즘에서 기준으로 하는 추종 거리로 조향 각도를 계산하는데 이용 합니다.
# 전방주시거리(Look Forward Distance)는 해당 예제에서 "self.lfd" 라는 변수로 사용합니다.
# 직접 "self.lfd" 변수의 값에 적절한 값을 넣어 제어 알고리즘의 성능을 올릴 수 있습니다.
# 아래 정의 된 변수 중 "self.vehicle_length" 와 "self.lfd" 를 변경 하여서 직접 제어기 성능을 튜닝 해보세요.

'''
class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        '''
        # Local Path 와 Odometry 데이터를 수신 할 Subscriber 를 만들고
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        rospy.Subscriber("local_path" )
        rospy.Subscriber("odom" )
        self.ctrl_cmd_pub =

        '''

        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)


        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2

        self.is_path=False
        self.is_odom=False

        self.is_look_forward_point=False

        self.forward_point=Point()
        self.current_postion=Point()

        self.vehicle_length = 1
        self.lfd = 5

        rate = rospy.Rate(50) # 30hz
        while not rospy.is_shutdown():

            if self.is_path ==True and self.is_odom==True  :

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                    self.ctrl_cmd_msg.velocity = 20.0
                    print(self.ctrl_cmd_msg.steering)
                else :
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.velocity = 0.0

                #TODO: (4) 제어입력 메세지 Publish
                '''
                # 제어입력 메세지 를 전송하는 publisher 를 만든다.
                self.ctrl_cmd_pub.
                '''
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=tf.transformations.euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def calc_pure_pursuit(self,):
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (2) 좌표 변환 행렬 생성
        '''
        # Pure Pursuit 알고리즘을 실행 하기 위해서 차량 기준의 좌표계가 필요합니다.
        # Path 데이터를 현재 차량 기준 좌표계로 좌표 변환이 필요합니다.
        # 좌표 변환을 위한 좌표 변환 행렬을 작성합니다.
        # Path 데이터를 차량 기준 좌표 계로 변환 후 Pure Pursuit 알고리즘 중 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 찾습니다.
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 이용하여 조향 각도를 계산하게 됩니다.
        # 좌표 변환 행렬을 이용해 Path 데이터를 차량 기준 좌표 계로 바꾸는 반복 문을 작성 한 뒤
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 계산하는 로직을 작성 하세요.

        trans_matrix = np.array([   [                       ,                       ,               ],
                                    [                       ,                       ,               ],
                                    [0                      ,0                      ,1              ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(self.path.poses) :
            path_point =

            global_path_point = [ , , 1]
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0]>0 :
                dis =
                if dis >= self.lfd :
                    self.forward_point =
                    self.is_look_forward_point = True
                    break
        '''
        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), 0],
                                 [sin(self.vehicle_yaw),  cos(self.vehicle_yaw), 0],
                                 [0                    ,0                      ,1 ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(self.path.poses) :
            path_point = i.pose.position

            global_path_point = [path_point.x - translation[0], path_point.y - translation[1], 1]
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0]>0 :
                dis = (local_path_point[0]**2 + local_path_point[1]**2)**0.5
                if dis >= self.lfd :
                    self.forward_point = local_path_point
                    self.is_look_forward_point = True
                    break


        #TODO: (3) Steering 각도 계산
        '''
        # 제어 입력을 위한 Steering 각도를 계산 합니다.
        # theta 는 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 좌표의 각도를 계산 합니다.
        # Steering 각도는 Pure Pursuit 알고리즘의 각도 계산 수식을 적용하여 조향 각도를 계산합니다.
        theta =
        steering =

        '''
        try:
            theta = atan2(self.forward_point[1], self.forward_point[0])
        except:
            theta = 0

        steering = atan2(2 * self.vehicle_length * sin(theta), self.lfd)

        return steering


if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
```

<br>

### 4-2. PID 제어를 적용한 종 방향 제어

<br>

### 4-3. 도로의 곡률을 고려한 차량의 주행 속도 계획

<br>

### 4-4. Pure pursuit 알고리즘을 강화한 Advanced Pure pursuit 알고리즘

<br>

### 4-5. Adaptive Cruise Control
