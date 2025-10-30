# 🚗 connectwo_line_tracing

`connectwo_line_tracing`는 **Connectwo 로봇 기반 라인트레이싱(Line Tracing) 시스템**을 위한 코드 모듈입니다.  
라즈베리파이(Raspberry Pi)를 사용하며, 카메라, IR 센서로 듀얼 센싱을 통해 흑백 라인을 인식하고 모터드라이버를 제어하여 주행합니다.



## 폴더 구조
```
connectwo_line_tracing/
├── firmware/
│ ├── raspberrypi/
│ │ ├── run_line_tracing.py # 메인 실행 스크립트
│ │ ├── sensors.py # 라인 센서 입력 처리
│ │ ├── motor_driver.py # 모터 제어 로직
│ │ └── requirements.txt # 의존성 패키지 목록
│ └── mcu/
│ ├── line_sensor_read.ino # (선택) MCU에서 센서 읽기 전용 코드
│ └── README.md
├── controller/
│ ├── line_follower.py # 기본 라인트레이싱 알고리즘
│ ├── junction_handler.py # 교차로, 직각 라인 처리
│ └── pid.py # PID 제어기 모듈
├── robot/
│ ├── hardware.yaml # 센서/모터 하드웨어 설정
│ ├── thresholds.yaml # 검정/흰 바닥 인식 임계값
│ └── kinematics.yaml # 바퀴 간격, 속도 제한 등 로봇 파라미터
├── tests/
│ ├── print_sensors.py # 센서값 확인용 테스트 스크립트
│ ├── motor_slow.py # 저속 주행 테스트
│ └── simulate_pattern.py # 시뮬레이션용 코드
├── utils/
│ ├── logger.py # 로깅 유틸리티
│ └── serial_helper.py # 시리얼 통신 헬퍼
├── scripts/
│ ├── pull_and_run.sh # 라즈베리파이 자동 업데이트 & 실행 스크립트
│ └── setup_pi.sh # 초기 환경 설정 스크립트
└── README.md
```

---

## 개발 환경
```
- **Main Development:** PC (Mac / Windows / Ubuntu)
- **Execution Target:** Raspberry Pi (Ubuntu 20.04)
- **Version Control:** GitHub (Team Repository)
- **IDE:** VS Code

> 개발은 PC에서 진행하고, 라즈베리파이에서는 `git pull`을 통해 실행만 하는 것을 권장합니다.
```
---

## 설치 및 실행

### 1. 라즈베리파이 초기 설정
```
sudo apt update
sudo apt install -y python3-pip git
git clone https://github.com/<team_repo>/WASD_project.git
cd WASD_project/src/connectwo_line_tracing/firmware/raspberrypi
pip3 install -r requirements.txt
```

### 2. 코드 업데이트 & 실행
```bash
cd ~/WASD_project/src/connectwo_line_tracing
git pull
python3 firmware/raspberrypi/run_line_tracing.py
```

### 코드 구성 개요

| 폴더	| 역할 |
| ------ | ------|
|firmware/ | 실제 하드웨어 제어 코드 (센서, 모터) |
|controller/|	라인트레이싱 알고리즘, PID, 보정 로직|
|robot/|	로봇 설정 및 보정값(YAML 포맷)|
|tests/| 주행 전 테스트 및 디버깅용 스크립트|
|utils/|	로깅, 시리얼 통신 등 공통 유틸|
|scripts/|	라즈베리파이 자동화 및 초기 설정|

### 개발 워크플로우

1. PC에서 개발
- 새로운 기능 개발 / 버그 수정
- 커밋 & 푸시

```bash
코드 복사
git add .
git commit -m "feat: add PID tuning logic"
git push origin feature/pid-tuning
```

2. 라즈베리파이에서 실행
- GitHub에서 최신 코드 가져오기
- 테스트 주행

```bash

git pull
python3 firmware/raspberrypi/run_line_tracing.py
```
3. 필드 테스트 중 변경 발생 시

- 현장에서 수정한 값은 .local.yaml 등으로 따로 저장하고 .gitignore 처리
- PC로 가져와 정식 커밋 후 푸시

### 향후 확장 계획
 - 직각 라인 인식 및 회전 알고리즘 개선
 - 센서 보정 자동화
 - 시뮬레이션 환경 추가 (OpenCV 기반)
 - 라인트레이싱 데이터 로깅 및 시각화

### Contributors
| 이름	| 역할	|비고|
|------|------|------|
|홍찬호|	ROS / 시뮬레이션(SLAM, NAV2) 메인 개발| |
|김동현|	ROS / 하드웨어(Connectwo)| connectwo 라인센서 / 모터 제어|
|박영찬| Web Front/backend	| web 관제 시스템|
|조호윤|	Web Front/backend| web 관제 시스템|