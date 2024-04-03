#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import json
import rospy
from morai_msgs.msg import GPSMessage
from std_msgs.msg import Float32MultiArray, Int64
from pyproj import Transformer
import paho.mqtt.client as mqtt

state_file_path = "/tmp/loc_publisher_state.json"

# MQTT 클라이언트 인스턴스 생성
mqtt_client = mqtt.Client()

# 최근의 GPS 데이터를 저장할 변수
last_gps_data = None

def check_running_state():
    try:
        with open(state_file_path, 'r') as state_file:
            state = json.load(state_file)
            print(state.get("running", False))
            return state.get("running", False)
    except FileNotFoundError:
        return False

def gps_callback(data):
    global last_gps_data
    # GPS 데이터 저장
    last_gps_data = data

def publish_gps_data_BE(event):
    global last_gps_data
    if last_gps_data is not None:
        # GPS 데이터를 문자열로 변환
        gps_data = f"{last_gps_data.latitude} {last_gps_data.longitude}"
        # MQTT 브로커에 GPS 데이터 발행
        mqtt_client.publish("location/BE", gps_data)

def publish_gps_data_FE(event):
    global last_gps_data
    if last_gps_data is not None:
        # GPS 데이터를 문자열로 변환
        gps_data = f"{last_gps_data.latitude} {last_gps_data.longitude}"
        # MQTT 브로커에 GPS 데이터 발행
        mqtt_client.publish("location/FE", gps_data)

def convert_to_utm(latitude, longitude):
    # gps -> utm 좌표 변환
    transformer = Transformer.from_crs("epsg:4326", "epsg:32652", always_xy=True)
    utm_x, utm_y = transformer.transform(longitude, latitude)
    utm_x -= 302459.942
    utm_y -= 4122635.537

    return utm_x, utm_y

def main():
    print("locPublisher running=========")

    # ROS 노드, 퍼블리셔 초기화
    route_pub = rospy.Publisher("/route", Float32MultiArray, queue_size=10)
    rsvId_pub = rospy.Publisher("/rsvId", Int64, queue_size=1)
    rospy.init_node('gps_mqtt_publisher', anonymous=True)
    rate = rospy.Rate(2)

    if len(sys.argv) > 1:
        json_data = sys.argv[1]
        data = json.loads(json_data)

        # rsvId 발행
        rsvId_msg = Int64()
        rsvId_msg.data = data["rsvId"]
        print(rsvId_msg)
        rsvId_pub.publish(rsvId_msg)
        rate.sleep()

        # UTM 좌표로 변환 후 경로 리스트 발행
        route_msg = Float32MultiArray()
        for location in data["locations"]:
            utm_x, utm_y = convert_to_utm(location["latitude"], location["longitude"])
            route_msg.data.extend([utm_x, utm_y])

        print(route_msg)
        route_pub.publish(route_msg)
        rate.sleep()

        print("ROS publish completed")

    else:
        print("Error: No data provided!")

    # MQTT 브로커에 연결
    mqtt_client.connect("j10d212.p.ssafy.io", 1883, 60)
    mqtt_client.loop_start()

    # GPS 데이터를 구독
    rospy.Subscriber("/gps", GPSMessage, gps_callback)

    # running state일 때만 publish 실행
    while not rospy.is_shutdown():
        if check_running_state():
            # 10초마다 publish_gps_data_BE 함수 호출
            rospy.Timer(rospy.Duration(10), publish_gps_data_BE)

            # 1초마다 publish_gps_data_FE 함수 호출
            rospy.Timer(rospy.Duration(1), publish_gps_data_FE)
        else:
            # ROS 종료 후 MQTT 연결 종료
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
            rospy.signal_shutdown("Stopping locPublisher")
        rate.sleep()

if __name__ == '__main__':
    main()
