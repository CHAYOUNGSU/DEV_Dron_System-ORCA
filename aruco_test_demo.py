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
    print("[INFO] AirSim 연결 시도 중...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("[SUCCESS] 시뮬레이터 연결 성공!")

    client.enableApiControl(True)
    client.armDisarm(True)

    # 1. 드론 이륙
    print("[1] 이륙 중...")
    client.takeoffAsync().join()

    # 2. 질문 1: 특정 좌표 (X=10m, Y=5m, Z=-3m 고도 3m)로 지정 이동
    target_x = 10.0
    target_y = 5.0
    target_z = -3.0 # AirSim에서 Z축은 음수가 상공(위) 방향입니다
    speed = 3.0     # 초당 3m 속도

    print(f"[2] 특정 좌표 (X:{target_x}m, Y:{target_y}m, 고도:{abs(target_z)}m)로 이동 중...")
    client.moveToPositionAsync(target_x, target_y, target_z, speed).join()
    time.sleep(1)

    # 3. 질문 2: 그 지점에서 드론 정면 카메라 이미지 촬영 후 내 PC에 저장
    print("[3] 해당 좌표에서 사진 촬영 중...")
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])

    if responses and len(responses) > 0:
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        if len(img1d) > 0:
            img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
            
            # 내 PC의 현재 워크스페이스 폴더에 저장
            save_filename = "target_position_photo.png"
            cv2.imwrite(save_filename, img_rgb)
            print(f"[SUCCESS] ✅ 사진이 내 PC의 '{save_filename}' 파일로 저장되었습니다!")

    # 4. 착륙 및 종료
    print("[4] 착륙 중...")
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("[SUCCESS] 미션 완수!")

if __name__ == "__main__":
    main()
