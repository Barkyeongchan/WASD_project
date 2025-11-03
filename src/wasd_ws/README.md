# 🤖 WASD Warehouse Robot (TurtleBot3 Burger)

소규모 물류 창고용 **자율주행 로봇 시스템**  
입출고 재고 관리 관제 앱과 연동되어,  
로봇이 물건을 자동으로 적치 위치까지 운반하고 확인 데이터를 DB로 전송합니다.

---

## 📦 프로젝트 개요

**전체 흐름**
1. 물건 입고  
2. 직원이 로봇에 물건 종류 & 수량 입력 → "이동" 버튼  
3. 로봇이 SLAM & Nav2를 이용해 지정된 랙으로 이동  
4. 직원이 물건을 선반에 적치하고 → "확인" 버튼  
5. 로봇이 DB 서버(Spring Boot + MySQL)에 입고 정보 전송  
6. 관제 클라이언트는 WebSocket으로 실시간 모니터링  
7. 로봇은 다시 대기 위치로 복귀  

---

## 🧱 개발 환경

| 구성 | 내용 |
|------|------|
| **로봇** | TurtleBot3 Burger + Raspberry Pi 4 |
| **ROS2** | Humble Hawksbill |
| **OS** | Ubuntu 22.04 (PC & Raspberry Pi) |
| **언어** | Python / C++ |
| **DB/서버** | AWS RDS (MySQL) + FastAPI |
| **통신** | REST API / WebSocket |
| **맵핑 & 내비게이션** | SLAM Toolbox + Nav2 |

---

## 🗂️ 폴더 구조

```
wasd_ws/
├── src/
│   ├── wasd_bringup/           # 전체 실행을 담당하는 launch 및 config
│   ├── wasd_robot_description/ # 로봇 URDF, TF, 센서 모델
│   ├── wasd_navigation/        # SLAM, Nav2, costmap, map 파일
│   ├── wasd_warehouse/         # 창고 업무 로직 (이동, 복귀 등)
│   ├── wasd_interfaces/        # msg/srv/action 정의
│   ├── wasd_bridge/            # 서버 통신 (REST/MQTT/WebSocket)
│   └── wasd_utils/             # 공용 유틸리티
├── build/                      # colcon build 결과물 (Git에 올리지 않음)
├── install/
└── log/
⚠️ build/, install/, log/ 폴더는 자동 생성되며 Git에 포함시키지 않습니다.
```

## 🧩 주요 패키지 설명
|패키지	|역할|
|------|------|
|wasd_bringup|	모든 노드 및 설정을 한 번에 실행하는 launch 패키지|
|wasd_navigation|	Nav2, SLAM, costmap, BT 설정 및 맵 데이터|
|wasd_warehouse|	로봇 이동, 랙 도착, 복귀 등의 창고 업무 로직|
|wasd_interfaces|	ROS2 메시지/서비스/액션 정의 (PutawayTask 등)|
|wasd_bridge|	서버와의 데이터 연동 (REST/MQTT/WebSocket)|
|wasd_robot_description|	URDF, TF, 센서 모델 정의|
|wasd_utils|	로깅, 파라미터 로더 등 공용 유틸리티|

## ⚙️ Git 관리 (.gitignore)
Git에는 소스 코드만 올리고 빌드 산출물은 제외합니다.

.gitignore (워크스페이스 루트에 생성):

```
# ROS2 build artifacts
/build/
/install/
/log/
/.colcon*
```

##🧑‍💻 개발 & 배포 워크플로우
##🖥️ 개발 PC (Ubuntu)
```
# 1. 워크스페이스 생성
mkdir -p ~/wasd_ws/src
cd ~/wasd_ws

# 2. Git clone
git clone git@github.com:<yourname>/wasd_robot.git src/wasd_robot

# 3. 빌드
colcon build
source install/setup.bash

# 4. 테스트 실행
ros2 launch wasd_bringup warehouse_robot.launch.py

# 5. 커밋 & 푸시 (소스 코드만)
git add .
git commit -m "feat: update nav2 params"
git push
```

##  🤖 Raspberry Pi (TurtleBot3 본체)
```
# 1. 코드 가져오기
cd ~/wasd_ws/src/wasd_robot
git pull

# 2. 빌드 (ARM 환경에서 다시 빌드)
cd ~/wasd_ws
colcon build --symlink-install
source install/setup.bash

# 3. 실행
ros2 launch wasd_bringup warehouse_robot.launch.py
```
### 💡 빌드는 PC와 Raspberry Pi 각각 별도로 수행해야 합니다.
(x86 ↔ ARM 환경이 달라서 빌드 결과물 공유 불가)
---

## 🌍 네트워크 설정
|역할|	설명|
|------|------|
|PC|	개발, RViz 시각화, 시뮬레이션|
|라즈베리파이|	실제 로봇 실행|
|ROS_DOMAIN_ID|	두 장치가 동일해야 통신 가능(30으로 설정되어있음)|
|시간 동기화|	chrony로 NTP 동기화 권장|
---

## 🚀 실행 시나리오 예시
```
# 1️⃣ 맵 기반 네비게이션 실행
ros2 launch wasd_bringup warehouse_robot.launch.py

# 2️⃣ 목적지 클릭 or 명령 발행
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}"

# 3️⃣ 로봇 이동
# → SLAM / Nav2로 경로 계획 및 이동
# → 도착 후 UI에서 "확인" 버튼

# 4️⃣ DB 서버에 정보 전송 (Spring Boot API)
# → REST POST /api/putaway/confirm
# → 관제 클라이언트에 WebSocket 푸시
```
## 📘 참고 명령어
```
# 빌드
colcon build --symlink-install

# 환경 세팅
source install/setup.bash

# 토픽 리스트
ros2 topic list

# 액션 상태 확인
ros2 action list
```

## 🧰 TODO (로봇 측)

 - wasd_navigation/config/nav2_params.yaml 튜닝 (footprint, inflation 등)

 - wasd_warehouse/config/shelves.yaml 작성 (선반 좌표, marker ID)

 - wasd_bringup/launch/warehouse_robot.launch.py 작성

 - 라즈베리파이에서 안정 실행 테스트

 - 서버 연동 (wasd_bridge) 구현

## 🧑‍🔧 Maintainer

WASD_Project Team