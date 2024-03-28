#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import threading
import subprocess
import paho.mqtt.client as mqtt

topicList = [("connect/BE/start", 0), ("distance/BE", 0)]

# 콜백 함수 정의
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(topicList)

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    if msg.topic == "distance/BE":
        thread = threading.Thread(target=run_calc_distance, args=(msg.payload,))
        thread.start()
    elif msg.topic == "connect/BE/start":
        thread = threading.Thread(target=run_loc_publisher, args=(msg.payload,))
        thread.start()

def run_calc_distance(payload):
    subprocess.call(["python", "/home/ssafy/catkin_ws/src/mqtt_connection/scripts/calcDistance.py", payload])

def run_loc_publisher(payload):
    subprocess.call(["python", "/home/ssafy/catkin_ws/src/mqtt_connection/scripts/locPublisher.py", payload])

# MQTT 클라이언트 인스턴스 생성
client = mqtt.Client()

# 콜백 함수 할당
client.on_connect = on_connect
client.on_message = on_message

# MQTT 브로커에 연결
client.connect("j10d212.p.ssafy.io", 1883, 60)

# 블로킹 호출로 메시지를 기다림
client.loop_forever()
