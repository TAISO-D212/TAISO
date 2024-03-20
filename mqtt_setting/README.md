# MQTT 설정 순서
- [EC2 서버 내 mosquitto 설치](https://changun516.tistory.com/201)
- conf 파일 수정 : 외부 ROS 서버에서 접속 가능하도록 설정 변경
    - ``` listener  -> listener 1883 ```
    - ``` allow_anonymous  -> allow_anonymous true ```
