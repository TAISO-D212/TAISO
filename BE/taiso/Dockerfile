# base 이미지 설정
FROM amazoncorretto:17

COPY .env /.env
COPY src/main/resources/taiso-18ea8-firebase-adminsdk-m9qcd-e458ff02c9.json src/main/resources/taiso-18ea8-firebase-adminsdk-m9qcd-e458ff02c9.json

# jar 파일을 컨테이너 내부에 복사
COPY build/libs/taiso-0.0.1-SNAPSHOT.jar /TAISO-app.jar

# 외부 호스트 3000 포트로 노출(외부에 호출될 포트)
EXPOSE 3000

# 실행 명령어
ENTRYPOINT ["java","-jar","/TAISO-app.jar"]


## base 이미지 설정
#FROM amazoncorretto:17
#
## jar 파일 위치를 변수로 설정
#ARG JAR_FILE=build/libs/*.jar
#
## jar 파일을 컨테이너 내부에 복사
#COPY ${JAR_FILE} TAISO-app.jar
#
## 외부 호스트 3000 포트로 노출(외부에 호출될 포트)
#EXPOSE 3000
#
## 실행 명령어
#ENTRYPOINT ["java","-jar","/TAISO-app.jar"]