#!/usr/bin/env python3
import cv2
import time
import glob

def try_open(dev, width=640, height=480, fps=30):
    cap = cv2.VideoCapture(dev)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap

def open_camera_auto(width=640, height=480, fps=30):
    # 우선순위: /dev/video0 -> /dev/video1 -> 나머지
    candidates = ["/dev/video0", "/dev/video1"]
    others = [d for d in glob.glob("/dev/video*") if d not in candidates]
    candidates.extend(sorted(others))

    print("[cam] 찾은 비디오 장치:", candidates)

    for dev in candidates:
        cap = try_open(dev, width, height, fps)
        if cap is not None:
            print(f"[cam] 이 장치를 사용합니다: {dev}")
            return cap
        else:
            print(f"[cam] 열기 실패: {dev}")

    raise RuntimeError("사용 가능한 카메라를 찾지 못했습니다.")


def get_line_offset(frame, show=False):
    """
    아주 단순 버전:
    - 아래쪽 일부만 잘라서(ROI)
    - 그레이스케일 → 블러 → 이진화
    - 흰색(또는 검정) 픽셀의 중심 x를 찾는다
    반환: offset (화면 중앙 기준, 픽셀 단위. 왼쪽이면 -, 오른쪽이면 +)
    """
    h, w, _ = frame.shape

    # 아래쪽 1/3만 사용
    roi = frame[int(h*0.6):h, 0:w]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    # 바닥/라인에 따라 바꾸자. 흰색 라인 = THRESH_BINARY, 검정 라인 = THRESH_BINARY_INV
    _, thresh = cv2.threshold(blur, 150, 255, cv2.THRESH_BINARY_INV)

    # x 좌표 평균
    M = cv2.moments(thresh)
    cx = None
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        # 시각화
        if show:
            cv2.circle(roi, (cx, roi.shape[0]//2), 5, (0,0,255), -1)

    offset = None
    if cx is not None:
        center = w // 2
        offset = cx - center  # 왼쪽이면 -, 오른쪽이면 +

    if show:
        cv2.imshow("roi", roi)
        cv2.imshow("thresh", thresh)

    return offset

if __name__ == "__main__":
    cap = open_camera_auto(640, 480, 30)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 읽지 못했습니다.")
                break

            offset = get_line_offset(frame, show=True)
            print("line offset:", offset)

            # q 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.01)
    finally:
        cap.release()
        cv2.destroyAllWindows()
