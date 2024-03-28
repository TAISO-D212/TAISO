#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import GPSMessage
import paho.mqtt.client as mqtt
    
# MQTT 클라이언트 인스턴스 생성
mqtt_client = mqtt.Client()

# 최근의 GPS 데이터를 저장할 변수
last_gps_data = None

def gps_callback(data):
    global last_gps_data
    # GPS 데이터 저장
    last_gps_data = data

def publish_gps_data(event):
    global last_gps_data
    if last_gps_data is not None:
        # GPS 데이터를 문자열로 변환
        gps_data = "%s %s" % (last_gps_data.latitude, last_gps_data.longitude)
        # MQTT 브로커에 GPS 데이터 발행
        mqtt_client.publish("location/BE", gps_data)

def main():
    # MQTT 브로커에 연결
    mqtt_client.connect("j10d212.p.ssafy.io", 1883, 60)
    mqtt_client.loop_start()

    # ROS 노드 초기화
    rospy.init_node('gps_mqtt_publisher', anonymous=True)

    # GPS 데이터를 구독
    rospy.Subscriber("/gps", GPSMessage, gps_callback)

    # 10초마다 publish_gps_data 함수 호출
    rospy.Timer(rospy.Duration(10), publish_gps_data)

    # ROS 스핀, 콜백 함수를 계속해서 호출
    rospy.spin()

    # ROS 종료 후 MQTT 연결 종료
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

if __name__ == '__main__':
    main()