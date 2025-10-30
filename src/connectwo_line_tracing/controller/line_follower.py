from firmware.raspberrypi.cam_sensors import get_line_offset, open_camera

cap = open_camera()
while True:
    ret, frame = cap.read()
    if not ret:
        break
    offset = get_line_offset(frame)
    ang = 0.0
    if offset is not None:
        ang = -offset * 0.003   # 대충 비례제어 (나중에 PID로)
    lin = 0.15                  # 항상 조금은 전진
    # 여기서 lin, ang을 STM32로 보내면 끝