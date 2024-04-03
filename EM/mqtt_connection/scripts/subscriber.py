#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import threading
import subprocess
import paho.mqtt.client as mqtt

topicList = [("connect/BE/start", 0), ("distance/BE", 0), ("connect/BE/end", 0)]
state_file_path = "/tmp/loc_publisher_state.json"

# 콜백 함수 정의
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(topicList)

def on_message(client, userdata, msg):
    print(f"{msg.topic} {msg.payload.decode()}")
    if msg.topic == "distance/BE":
        thread = threading.Thread(target=run_calc_distance, args=(msg.payload.decode(),))
        thread.start()
    elif msg.topic == "connect/BE/start":
        update_state(True)  # locPublisher 실행 시작
        thread = threading.Thread(target=run_loc_publisher, args=(msg.payload.decode(),))
        thread.start()
    elif msg.topic == "connect/BE/end":
        update_state(False)  # locPublisher 실행 종료

def run_calc_distance(payload):
    current_dir = os.path.dirname(__file__)
    calc_distance_path = os.path.join(current_dir, "calcDistance.py")
    subprocess.call(["python3", calc_distance_path, payload])

def run_loc_publisher(payload):
    current_dir = os.path.dirname(__file__)
    loc_publisher_path = os.path.join(current_dir, "locPublisher.py")
    subprocess.call(["python3", loc_publisher_path, payload])

def update_state(running):
    with open(state_file_path, 'w') as state_file:
        json.dump({"running": running}, state_file)

# MQTT 클라이언트 인스턴스 생성
client = mqtt.Client()

# 콜백 함수 할당
client.on_connect = on_connect
client.on_message = on_message

# MQTT 브로커에 연결
client.connect("j10d212.p.ssafy.io", 1883, 60)

# 블로킹 호출로 메시지를 기다림
client.loop_forever()
