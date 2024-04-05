
# TAISO: 시골 친화형 자율주행 택시 서비스

![](https://i.imgur.com/rYL1KGC.png)


# I. 개요
타이소는 교통 소외지역 주민을 위한 자율주행 택시 서비스입니다.

    - 삼성 청년 SW 아카데미 SSAFY 10기 2학기 특화 프로젝트 구미 2반 12팀
    - 2023.2.19 ~ 2023.4.5

### 1. 프로젝트 개요
---
#### 앱으로 호출하는 편리한 시골 택시
    →  터치 한번으로 예약하는 편리함!  
    →  교통 소외지역에 이동의 자유를!

#### 합승 서비스로 경제적인 주행 제공

    →  마을 사람들의 예약 목록을 확인하여 합승 신청!
    →  효율적인 주행으로 불필요한 이동 및 대기 시작 감소!
	

#### 실시간 이동경로 확인

    →  현재 이동 중인 위치를 실시간으로 확인!
    →  예상 도착시간 확인 가능!




### 2. 프로젝트 목표 및 배경
---


    [목표]
    1. 교통 소외지역 주민의 이동권 보장을 위한 자율주행 서비스 제공
        - 도심지에 비해 부족한 시골 지역의 교통 인프라를 자율 주행 버스 서비스를 이용해 개선
        - 부족한 인력과 수익 구조를 무인 자율화를 통해 보완
    2. 지역소멸 위기에 처한 시골 지역에 인구 유치 및 균형 발전 도모
    3. 이동이 불편한 노령 인구를 직접 이송함으로써 노인의 이동 편의성 증대 및 교류 확대

    [배경]
	1. 디지털 소외계층에 대한 사용자 경험 기반 서비스 제공
	    * 예전부터 존재했던 버스와 자율주행을 결합한 새로운 형태의 서비스
	    * 농촌지역에 도입되는 자율주행기반 기술 체험과 인식 개선
	2. 농촌 주민의 소외된 삶, 사회적 배려
	3. 도심지에 집중된 인프라 보완
	4. 자율주행 기반으로 교통복지 예산 낭비 방지


### 3. 기대 효과
---
    [기대효과]
    1. 교통 소외지역 주민의 사회 참여 증가
        * 시골 지역의 교통 인프라 개선
        * 자율주행 버스가 교통 소외지역 주민에게 편의성 제공
        * 교통 소외지역 주민의 교류와 이동 증가로 교통 소외 지역 주민의 사회 참여 증가
    2. 지역 경제 활성화
        * 지역에 새로운 이동 수단이 생기면서 소비 활동이 증가하고, 지역 상권의 활성
        * 관광객들의 접근성이 증대되면서 지역 관광 수입 증가 기대
        * 자율주행 버스를 통해 도심 이외의 지역에도 효율적인 교통 수단을 제공함으로써, 도시와 농촌 간의 격차 해소
        * 자율주행 버스 도입을 통한 노인의 경제활동 참여 증가

### 4. 팀원 소개
---
    ❤️ 김태용: FE
    🧡 전근렬: BE/FE
    💛 배성연: BE/MQTT 통신
    💚 정경리: BE/DevOps
    💙 전인구: 자율주행 (ROS/Morai Simulator)
    💜 양원석: 자율주행 (ROS/Morai Simulator)




# II. TAISO 서비스 화면 
### 1. 앱 화면
#### 1️⃣ 로그인

<p align="center">
 <img src = "https://i.imgur.com/N0FKOap.gif">
</p>

#### 2️⃣ 합승예약

<p align="center">
 <img src = "https://i.imgur.com/wdwRg6X.gif">
</p>

#### 3️⃣ 즐겨찾기 추가/삭제

<p align="center">
 <img src = "https://github.com/TAISO-D212/TAISO/assets/50294908/5af2951a-2aa3-4e7e-b73a-7473ed6df59b">
</p>

#### 4️⃣ 현 위치 마커

<p align="center">
 <img src = "https://i.imgur.com/bmIOWI8.gif">
</p>

#### 5️⃣ 자율주행

<div style="text-align:center;">
  <img src="/uploads/33986547e90babcd64467a2d7ebb8731/자율주행.gif" style="width: 20%;">
</div>



### 2. 시뮬레이터 화면

#### 1️⃣ 인지 - 카메라 센서
<p align="center">
 <img src = "/uploads/585b1828d2cfa3c54b4fb5187b622fa1/센서_활용.gif">
</p>

#### 2️⃣ 인지 - 라이다 센서

<p align="center">
 <img src = "https://i.imgur.com/NiituoU.png">
</p>
<p align="center">
 <img src = "https://i.imgur.com/SHpprM9.png">
</p>
<p align="center">
 <img src = "/uploads/6408145d1cb0f80c02a3d50eee1e1682/라이다.gif">
</p>

#### 3️⃣ 판단 - dijkstra 경로 계획 

<p align="center">
 <img src = "https://i.imgur.com/DsuFdg2.png">
</p>
<p align="center">
 <img src = "/uploads/a4966c9ebc1bad84b96e2c313fc04920/다익스트라.gif">
</p>


#### 4️⃣ 판단 - V2X 

<p align="center">
 <img src = "https://i.imgur.com/9CVwDrz.png">
</p>


#### 5️⃣ 제어 - acc & 정지선 속도 제어

* 정지선/신호 인식
<p align="center">
 <img src = "https://github.com/TAISO-D212/TAISO/assets/50294908/a9b07f11-0976-4041-9f98-d8b8142a007e">
</p>

* 앞 차 인식
<p align="center">
 <img src = "/uploads/ab8c5c3e6a58b3e207fa2b1fd59fe256/앞차인식.gif">
</p>


# III. 기술 & 개발 환경 / 빌드 및 배포

#### 📔 [매뉴얼 바로가기](https://github.com/TAISO-D212/TAISO/blob/master/docs/TAISO%20%ED%8F%AC%ED%8C%85%20%EB%A7%A4%EB%89%B4%EC%96%BC.md)

### 🏗️ 아키텍처
#### 🔧 기술
![](https://i.imgur.com/zigXAr3.png)
![](https://i.imgur.com/ajU7dcz.png)

#### ☁️ 배포
![](https://i.imgur.com/TSfkqtI.png)


# IV. 프로젝트 산출물  

🧾 [기능 명세서](https://kimtaeyong.notion.site/dbf6a8fdc14d494e8172ca040bb36038)  

📃 [요구사항 정의서](https://kimtaeyong.notion.site/86156328c3734bf6b6bcb4360d5c81a6)  

📜 [API 명세서](https://kimtaeyong.notion.site/API-23b59c75b0584712b4d625b71ff70efe)  

💾 [ERD](https://kimtaeyong.notion.site/ERD-4e00456b5ad64f2586e0b24d4ff7eb4b)  



#### 프로젝트 파일 구조
📱 [FE 구조](https://github.com/TAISO-D212/TAISO/blob/master/FE/%ED%8F%B4%EB%8D%94%EA%B5%AC%EC%A1%B0_FE.txt) 

⚙️ [BE 구조](https://github.com/TAISO-D212/TAISO/blob/master/BE/%ED%8F%B4%EB%8D%94%EA%B5%AC%EC%A1%B0_BE.txt)  


# V. 프로젝트 결과물

📔 [포팅 매뉴얼](https://github.com/TAISO-D212/TAISO/blob/master/docs/TAISO%20%ED%8F%AC%ED%8C%85%20%EB%A7%A4%EB%89%B4%EC%96%BC.md)  

🗣️ [중간 발표 자료](https://www.canva.com/design/DAF_XhUAR14/W7leyn3d0c11jM4fW0YpjA/view?utm_content=DAF_XhUAR14&utm_campaign=designshare&utm_medium=link&utm_source=editor)  

🗣️ [최종 발표 자료](https://www.canva.com/design/DAGBUDIsiLA/UGu0VwczUgmRmfLWdVq59g/view?utm_content=DAGBUDIsiLA&utm_campaign=share_your_design&utm_medium=link&utm_source=shareyourdesignpanel)  

🎬 [UCC 영상](https://www.youtube.com/watch?v=rpGTvFzOC_0)  



