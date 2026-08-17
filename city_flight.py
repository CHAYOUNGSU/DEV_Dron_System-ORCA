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
    print("[INFO] CityEnviron 3D 대도시 시뮬레이터에 연결 중...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("[SUCCESS] 연결 성공! 도심 항공 드론 비행을 시작합니다.")

    # 제어 권한 획득
    client.enableApiControl(True)
    client.armDisarm(True)

    # 1. 빌딩 숲 상공 40m 고도로 고속 상승
    print("[1/5] 🏙️ 빌딩 숲 상공 40미터 고도로 이륙 중...")
    client.takeoffAsync().join()
    client.moveToPositionAsync(0, 0, -40, velocity=5).join()
    time.sleep(1)

    # 2. 고층 빌딩 사이 대도시 도로망 순항 비행 (전방 100m, 우측 50m)
    print("[2/5] 🚁 대도시 빌딩 사이 100m 순항 비행 중...")
    client.moveToPositionAsync(100, 0, -40, velocity=8).join()
    client.moveToPositionAsync(100, 50, -40, velocity=8).join()
    time.sleep(1)

    # 3. 도심 파노라마 스카이라인 360도 스캔
    print("[3/5] 🔄 도심 파노라마 스카이라인 360도 스캔 회전...")
    for angle in [90, 180, 270, 360]:
        client.rotateToYawAsync(angle, margin=5).join()
        time.sleep(0.5)

    # 4. 고층 빌딩 시티 스카이라인 고화질 캡처
    print("[4/5] 📸 고층 빌딩 시티 스카이라인 고화질 촬영 중...")
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    if responses and len(responses) > 0:
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        if len(img1d) > 0:
            img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
            cv2.imwrite("city_skyline_view.png", img_rgb)
            print("[SUCCESS] 화려한 도시 스카이라인 사진이 'city_skyline_view.png'로 저장되었습니다.")

    # 5. 안전한 지점 복귀 및 착륙
    print("[5/5] 🛬 출발 지점으로 복귀 및 착륙 중...")
    client.moveToPositionAsync(0, 0, -10, velocity=6).join()
    client.landAsync().join()

    client.armDisarm(False)
    client.enableApiControl(False)
    print("[SUCCESS] 🏙️ CityEnviron 도심 드론 비행 미션이 성공적으로 완료되었습니다!")

if __name__ == "__main__":
    main()
