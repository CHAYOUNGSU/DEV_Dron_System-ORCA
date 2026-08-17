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
    print("[INFO] LandscapeMountains 3D 시뮬레이터에 연결 중...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("[SUCCESS] 연결 성공! 산악 지형 드론 제어를 시작합니다.")

    # 제어 권한 획득
    client.enableApiControl(True)
    client.armDisarm(True)

    # 1. 산악 지형 상공으로 고고도 이륙 (30m 상승)
    print("[1/5] 🏔️ 산악 상공 30미터 고도로 고속 상승 중...")
    client.takeoffAsync().join()
    client.moveToPositionAsync(0, 0, -30, velocity=5).join()
    time.sleep(1)

    # 2. 산맥을 가로지르는 장거리 탐색 비행 (전방 80m, 우측 40m)
    print("[2/5] 🚁 산맥 장거리 비행 탐색 중 (전방 80m, 우측 40m)...")
    client.moveToPositionAsync(80, 0, -35, velocity=8).join()
    client.moveToPositionAsync(80, 40, -35, velocity=8).join()
    time.sleep(1)

    # 3. 산악 전경 360도 스캔 회전
    print("[3/5] 🔄 웅장한 산악 풍경 360도 스캔 회전...")
    for angle in [90, 180, 270, 360]:
        client.rotateToYawAsync(angle, margin=5).join()
        time.sleep(0.5)

    # 4. 고화질 산악 전경 카메라 캡처
    print("[4/5] 📸 드론 정면 고화질 산악 카메라 이미지 캡처...")
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    if responses and len(responses) > 0:
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        if len(img1d) > 0:
            img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
            cv2.imwrite("mountain_drone_view.png", img_rgb)
            print("[SUCCESS] 아름다운 산악 풍경 사진이 'mountain_drone_view.png'로 저장되었습니다.")

    # 5. 출발 지점으로 복귀 및 착륙
    print("[5/5] 🛬 출발지 지점으로 복귀 및 안전 착륙...")
    client.moveToPositionAsync(0, 0, -10, velocity=6).join()
    client.landAsync().join()

    client.armDisarm(False)
    client.enableApiControl(False)
    print("[SUCCESS] 🏔️ LandscapeMountains 드론 산악 탐색 비행이 완료되었습니다!")

if __name__ == "__main__":
    main()
