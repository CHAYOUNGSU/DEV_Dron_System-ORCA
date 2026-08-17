import sys
import io

# 윈도우 콘솔 UTF-8 설정
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

import airsim
import time
import cv2
import numpy as np

def main():
    print("[INFO] AirSim 시뮬레이터 연결 중...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("[SUCCESS] 연결 성공!")

    client.enableApiControl(True)
    client.armDisarm(True)

    # 1. 10미터 높이로 이륙
    print("[1/6] 🚀 고도 10m로 힘차게 이륙 중...")
    client.takeoffAsync().join()
    client.moveToPositionAsync(0, 0, -10, velocity=3).join()
    time.sleep(1)

    # 2. 사각형(Square) 비행 궤적 그리기 (가로세로 15m)
    print("[2/6] 🔄 15미터 사각형 궤적 비행 시작...")
    positions = [
        (15, 0, -10),   # 앞으로 15m
        (15, 15, -10),  # 우측으로 15m
        (0, 15, -10),   # 뒤로 15m
        (0, 0, -10)     # 다시 제자리
    ]

    for idx, (x, y, z) in enumerate(positions, 1):
        print(f"   -> 지점 {idx}/4 로 이동 중... (X:{x}m, Y:{y}m)")
        client.moveToPositionAsync(x, y, z, velocity=5).join()
        time.sleep(0.5)

    # 3. 360도 제자리 제자리 회전 (Yaw 제어)
    print("[3/6] 🔄 360도 제자리 회전 제어...")
    for angle in [90, 180, 270, 360]:
        client.rotateToYawAsync(angle, margin=5).join()
        time.sleep(0.5)

    # 4. 고도 20m로 수직 상승 후 공중 뷰 카메라 캡처
    print("[4/6] 🚀 20미터 고도로 고속 상승...")
    client.moveToPositionAsync(0, 0, -20, velocity=5).join()
    time.sleep(1)

    print("[5/6] 📸 공중 뷰 카메라 캡처...")
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    if responses and len(responses) > 0:
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        if len(img1d) > 0:
            img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
            cv2.imwrite("high_altitude_view.png", img_rgb)
            print("[SUCCESS] 'high_altitude_view.png' 파일이 저장되었습니다.")

    # 5. 안전 착륙
    print("[6/6] 🛬 출발지점으로 돌아와 착륙 중...")
    client.moveToPositionAsync(0, 0, -5, velocity=4).join()
    client.landAsync().join()

    client.armDisarm(False)
    client.enableApiControl(False)
    print("[SUCCESS] ✈️ 모든 화려한 비행 미션이 완료되었습니다!")

if __name__ == "__main__":
    main()
