#!/usr/bin/env python
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt

# 콜백 함수 정의
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("/testTopic")

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

# MQTT 클라이언트 인스턴스 생성
client = mqtt.Client()

# 콜백 함수 할당
client.on_connect = on_connect
client.on_message = on_message

# MQTT 브로커에 연결
client.connect("192.168.56.1", 1883, 60)

# 블로킹 호출로 메시지를 기다림
client.loop_forever()
