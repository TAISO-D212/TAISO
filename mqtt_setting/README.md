# MQTT 설정 순서
- [EC2 서버 내 mosquitto 설치](https://changun516.tistory.com/201)
- windows : conf 파일 수정 : 외부 ROS 서버에서 접속 가능하도록 설정 변경
    - ``` listener  -> listener 1883 ```
    - ``` allow_anonymous  -> allow_anonymous true ```
- linux
    - conf 파일 새로 생성 : 외부 ROS 서버에서 접속 가능하도록 설정 변경
        - ```cd /etc/mosquitto/conf.d/``` : 디렉토리 이동
        - ```sudo nano custom_config.conf``` : 새 설정 파일 생성
        - ``` listener 1883``` : 기본 포트 번호 리스닝 활성화
        - ``` llow_anonymous true ``` : 익명 사용자 접근 허용(보안은 과제 사항)
        - ``` listener 9001``` : FE 연결용 websocket 포트 번호 리스닝 활성화
        - ``` protocol websockets``` : FE 연결용 websocket 프로토콜 접속 활성화
        - ```cafile /path/to/ca.crt``` : 인증서(SSL/TLS) 파일 경로
        - ```certfile /path/to/server.crt``` : 서버의 공개 키 포함하는 인증서 파일 경로
        - ```keyfile /path/to/server.key``` : 서버의 비공개 키 포함하는 키 파일 경로
    - mosquitto 서비스 시작으로 설정 적용
        - ```sudo systemctl start mosquitto```
    - EC2 서버 시스템 부팅 시 자동으로 mosquitto 시작
        - ```sudo systemctl enable mosquitto```
    - 서비스 상태 확인
        - ```sudo systemctl status mosquitto```
- ROS
    - mqtt-client 설치
        - ```pip install paho-mqtt```
        - ```sudo apt-get install mosquitto-clients```

