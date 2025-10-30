#!/usr/bin/env python3
"""
sensors.py
- Raspberry Pi에서 카메라 + Arduino(IR 5개) 센서를 동시에 다루는 모듈
- 하드웨어 설정은 robot/hardware.yaml 에서 읽어온다.
"""

import os
import time
import yaml
import threading

# 카메라
import cv2

# 아두이노 시리얼
try:
    import serial
except ImportError:
    serial = None
    print("[sensors] pyserial이 설치되어 있지 않습니다. `pip install pyserial` 해주세요.")


CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "robot",
    "hardware.yaml"
)


class Sensors:
    def __init__(self, config_path: str = CONFIG_PATH):
        # 1) 설정 불러오기
        self.config = self._load_config(config_path)

        # 2) 카메라 설정
        cam_cfg = self.config.get("camera", {})
        self.use_camera = cam_cfg.get("use", True)
        self.camera_device = cam_cfg.get("device", "/dev/video0")
        self.camera_resolution = cam_cfg.get("resolution", [640, 480])
        self.camera_fps = cam_cfg.get("framerate", 30)

        # 3) IR(아두이노) 설정
        ir_cfg = self.config.get("ir_sensors", {})
        self.use_ir = ir_cfg.get("use", True)
        self.ir_port = ir_cfg.get("serial_port", "/dev/ttyACM0")
        self.ir_baud = ir_cfg.get("baudrate", 115200)
        self.ir_count = ir_cfg.get("count", 5)
        self.ir_last_values = [0] * self.ir_count
        self.ir_lock = threading.Lock()

        # 4) 시리얼 연결 (아두이노)
        self.arduino_ser = None
        if self.use_ir and serial is not None:
            self._open_arduino_serial()

        # 5) 카메라 열기
        self.cap = None
        if self.use_camera:
            self._open_camera()

        # 6) 백그라운드로 아두이노 읽기 스레드
        if self.arduino_ser is not None:
            t = threading.Thread(target=self._ir_reader_loop, daemon=True)
            t.start()

    # ----------------------------------------------------------------------------------
    # 설정 불러오기
    # ----------------------------------------------------------------------------------
    def _load_config(self, path: str):
        if not os.path.exists(path):
            raise FileNotFoundError(f"[sensors] 하드웨어 설정 파일을 찾을 수 없습니다: {path}")
        with open(path, "r") as f:
            return yaml.safe_load(f)

    # ----------------------------------------------------------------------------------
    # 아두이노 시리얼
    # ----------------------------------------------------------------------------------
    def _open_arduino_serial(self):
        try:
            self.arduino_ser = serial.Serial(self.ir_port, self.ir_baud, timeout=1)
            time.sleep(2)  # 아두이노 리셋 대기
            print(f"[sensors] Arduino 시리얼 연결됨: {self.ir_port} @ {self.ir_baud}")
        except Exception as e:
            print(f"[sensors] Arduino 시리얼 열기 실패: {e}")
            self.arduino_ser = None

    def _ir_reader_loop(self):
        """아두이노에서 오는 줄단위 데이터를 계속 읽어서 self.ir_last_values 에 저장"""
        while True:
            try:
                line = self.arduino_ser.readline().decode("utf-8").strip()
                if not line:
                    continue
                # 예: "512,480,300,450,600"
                parts = line.split(",")
                if len(parts) == self.ir_count:
                    vals = [int(p) for p in parts]
                    with self.ir_lock:
                        self.ir_last_values = vals
                # 필요하면 여기서 바로 threshold 비교해서 상태 만들 수도 있음
            except Exception as e:
                # 시리얼이 잠깐 끊겨도 전체 센서가 죽지 않게
                # print(f"[sensors] IR 읽기 오류: {e}")
                time.sleep(0.1)

    def get_ir_values(self):
        """가장 최근에 읽은 IR 값(리스트)을 반환"""
        if not self.use_ir:
            return []
        with self.ir_lock:
            return list(self.ir_last_values)

    # ----------------------------------------------------------------------------------
    # 카메라
    # ----------------------------------------------------------------------------------
    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            print(f"[sensors] 카메라를 열 수 없습니다: {self.camera_device}")
            self.cap = None
            return

        w, h = self.camera_resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
        print(f"[sensors] 카메라 열림: {self.camera_device} ({w}x{h} @ {self.camera_fps}fps)")

    def read_camera_frame(self):
        """카메라 프레임 한 장을 읽어 반환 (실패 시 None)"""
        if not self.use_camera or self.cap is None:
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    # ----------------------------------------------------------------------------------
    # 통합 읽기
    # ----------------------------------------------------------------------------------
    def read_all(self):
        """
        라인트레이싱 컨트롤러에서 바로 쓸 수 있게
        카메라 + IR 를 통합해서 dict로 반환
        """
        frame = self.read_camera_frame()
        ir_vals = self.get_ir_values()

        data = {
            "camera": {
                "frame": frame,   # OpenCV BGR 이미지 (None일 수도 있음)
                "width": self.camera_resolution[0],
                "height": self.camera_resolution[1],
            },
            "ir": {
                "values": ir_vals,
                "count": self.ir_count,
            },
            "timestamp": time.time(),
        }
        return data

    # ----------------------------------------------------------------------------------
    # 리소스 정리
    # ----------------------------------------------------------------------------------
    def close(self):
        if self.cap is not None:
            self.cap.release()
        if self.arduino_ser is not None:
            self.arduino_ser.close()


# 테스트 실행용
if __name__ == "__main__":
    s = Sensors()
    try:
        while True:
            data = s.read_all()
            ir_vals = data["ir"]["values"]
            print("IR:", ir_vals, " cam:", "OK" if data["camera"]["frame"] is not None else "NONE")
            time.sleep(0.2)
    except KeyboardInterrupt:
        s.close()
        print("\n[sensors] 종료")
