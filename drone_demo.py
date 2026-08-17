import sys
import io

# 윈도우 콘솔 UnicodeEncodeError (cp949) 방지
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

import airsim
import time
import cv2
import numpy as np

def main():
    print("AirSim 시뮬레이터에 연결 시도 중...")
    
    # 1. AirSim Multirotor(드론) 클라이언트 생성 및 연결
    client = airsim.MultirotorClient()
    
    try:
        client.confirmConnection()
        print("[SUCCESS] 시뮬레이터 연결 성공!")
    except Exception as e:
        print("[FAIL] 시뮬레이터 연결 실패. AirSim 환경(Blocks.exe 등)이 실행 중인지 확인하세요.")
        print(f"에러 내용: {e}")
        return

    # 2. 드론 제어 권한 획득 및 시뮬레이션 설정
    client.enableApiControl(True)
    client.armDisarm(True)

    print("[1/5] 이륙 (Takeoff)...")
    client.takeoffAsync().join()

    # 3. 목표 지점으로 이동 (x=0, y=0, z=-5 -> 고도 5m 비행)
    print("[2/5] 고도 5m 상승 및 제자리 비행...")
    client.moveToPositionAsync(0, 0, -5, 3).join()
    time.sleep(2)

    # 4. 카메라 시점 데이터(카메라 렌더링) 가져오기
    print("[3/5] 카메라 이미지 캡처 중...")
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False) # 전방 정면 RGB 카메라
    ])

    if responses and len(responses) > 0:
        response = responses[0]
        # 바이트 배열을 OpenCV 이미지로 변환
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        if len(img1d) > 0:
            img_rgb = img1d.reshape(response.height, response.width, 3)
            # 이미지 파일 저장
            cv2.imwrite("drone_camera.png", img_rgb)
            print("[SUCCESS] 카메라 이미지가 'drone_camera.png'로 저장되었습니다.")

    # 5. 전방 5m 이동 예시
    print("[4/5] 전방으로 5m 이동...")
    client.moveToPositionAsync(5, 0, -5, 2).join()
    time.sleep(2)

    # 6. 착륙 및 시뮬레이션 종료
    print("[5/5] 착륙 (Landing)...")
    client.landAsync().join()

    # 7. 제어 권한 해제
    client.armDisarm(False)
    client.enableApiControl(False)
    print("[SUCCESS] 드론 미션 완료!")

if __name__ == "__main__":
    main()
