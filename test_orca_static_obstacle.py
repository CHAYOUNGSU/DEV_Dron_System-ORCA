#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
[ORCA 정적 장애물(건물/구조물) 관통 경로 회피 실환경 실측 테스트]

검증 시나리오:
  1. AbandonedPark 시뮬레이터 실행 및 4대 편대 이륙.
  2. 대상 정적 장애물: SM_CarouselA_2 (월드 좌표 x=-0.07m, y=17.92m, z=3.64m)
  3. Bravo(Drone2)를 회전목마 너머 월드 (X=0.0m, Y=30.0m, Z=-4.0m) 위치로 전진 배치.
  4. Bravo 대상 /api/rth 호출 -> 홈 월드 (X=0.0m, Y=3.5m)로의 복귀 직선 경로가 회전목마를 정면 관통!
  5. ORCA 솔버(서버)가 정적 장애물 반경(2.2m)에 의해 횡방향 회피 속도(vx != 0)를 생성하여
     회전목마를 무충돌 안전 우회 통과하는지 20Hz 고빈도 독립 샘플러로 정량 실측.
  6. 대조군(Control Group) 직선 경로 모델과의 횡방향 편차(Delta X), 최소 거리, 충돌 0회, 홈 착륙 정합성 검증.
"""

import math
import time
import json
import socket
import urllib.request
import threading
import airsim

SERVER_BASE_URL = "http://127.0.0.1:8000"
REPORT_PATH = "orca_static_obstacle_report.json"
SIM_MAP_ID = "park"
REQUIRED_MIN_OBSTACLE_DIST_M = 2.2
MIN_AVOIDANCE_LATERAL_DEV_M = 1.0  # 횡방향 회피 최소 편차 기준 (1.0m 이상 횡방향 우회 기동 입증)

VEHICLES = {
    "Drone1": "SimpleFlight",
    "Drone2": "Drone2",
    "Drone3": "Drone3",
    "Drone4": "Drone4"
}
SPAWN_OFFSETS = {
    "Drone1": (0.0, 0.0, 0.0),
    "Drone2": (0.0, 3.5, 0.0),
    "Drone3": (0.0, 7.0, 0.0),
    "Drone4": (0.0, 10.5, 0.0)
}


def api_post(endpoint: str, payload: dict = None) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    data = json.dumps(payload or {}).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=120) as res:
        return json.loads(res.read().decode('utf-8'))


def api_get(endpoint: str) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    req = urllib.request.Request(url, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=10) as res:
        return json.loads(res.read().decode('utf-8'))


def check_port_open(port: int = 41451) -> bool:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        res = s.connect_ex(('127.0.0.1', port))
        s.close()
        return res == 0
    except Exception:
        return False


def wait_for_port(port: int = 41451, timeout: float = 60.0) -> bool:
    t_start = time.time()
    while time.time() - t_start < timeout:
        if check_port_open(port):
            return True
        time.sleep(0.3)
    return False


def dist2d(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def run_test():
    print("=" * 80)
    print("[ORCA 정적 장애물(회전목마) 관통 경로 회피 실환경 실측 테스트 (AbandonedPark)]")
    print("=" * 80)

    # 1. Launch AbandonedPark simulator
    print("\n[1] AbandonedPark 시뮬레이터 실행...")
    launch_res = api_post("/api/simulators/launch", {"id": SIM_MAP_ID, "resolution": "1280x720"})
    print(f"  - launch: {launch_res.get('message', launch_res)}")
    assert wait_for_port(41451, 60.0), "AbandonedPark 포트 오픈 타임아웃"
    time.sleep(3.0)

    # 2. Bulk Takeoff via server
    print("\n[2] 4대 편대 전체 동시 이륙...")
    takeoff_res = api_post("/api/fleet/takeoff")
    print(f"  - takeoff: {takeoff_res.get('message', takeoff_res)}")
    time.sleep(7.0)

    # Connect client
    client = airsim.MultirotorClient(timeout_value=10)
    client.confirmConnection()
    v_list = client.listVehicles()
    print(f"  - 현재 활성화된 편대 기체 목록: {v_list}")

    # 3. Target static obstacle: SM_CarouselA_2
    target_obstacle_name = "SM_CarouselA_2"
    obs_pose = client.simGetObjectPose(target_obstacle_name)
    obs_x = obs_pose.position.x_val
    obs_y = obs_pose.position.y_val
    obs_z = obs_pose.position.z_val
    print(f"\n[3] 대상 정적 장애물 확인: {target_obstacle_name}")
    print(f"  - 월드 좌표: (x={obs_x:.2f}m, y={obs_y:.2f}m, z={obs_z:.2f}m)")
    print(f"  - 요구 안전 이격 거리: >= {REQUIRED_MIN_OBSTACLE_DIST_M}m")

    # 4. Advance Bravo(Drone2) safely over Carousel to Y=+30.0m
    # Bravo home: (0.0, 3.5, 0.0), Spawn offset: (0.0, 3.5, 0.0)
    # Fly high (Z=-15.0m) over obstacles during setup, then descend to cruise altitude Z=-4.0m
    bravo_vname = "Drone2"
    bravo_off_y = 3.5
    target_adv_world = (0.0, 30.0, -4.0)
    print(f"\n[4] Bravo(Drone2)를 회전목마 너머(Y=+30m)로 안전 상공 경유 전진 배치...")
    # Step A: Climb to safe transit altitude Z=-15.0m
    client.moveToPositionAsync(0.0, 0.0, -15.0, 3.0, vehicle_name=bravo_vname).join()
    # Step B: Fly forward to Y=26.5m (World Y=30.0m)
    client.moveToPositionAsync(0.0, 26.5, -15.0, 4.0, vehicle_name=bravo_vname).join()
    # Step C: Descend to target cruise altitude Z=-4.0m
    client.moveToPositionAsync(0.0, 26.5, -4.0, 2.0, vehicle_name=bravo_vname).join()
    time.sleep(2.0)

    bravo_state_init = client.getMultirotorState(vehicle_name=bravo_vname)
    bp_init = bravo_state_init.kinematics_estimated.position
    bravo_init_world = (bp_init.x_val, bp_init.y_val + bravo_off_y, bp_init.z_val)
    print(f"  - Bravo 전진 배치 완료: 월드 ({bravo_init_world[0]:.2f}, {bravo_init_world[1]:.2f}, {bravo_init_world[2]:.2f})")

    # =========================================================================
    # SCENARIO: Bravo RTH Directly Piercing Carousel Obstacle
    # Path: from Y=30.0 to Y=3.5 (Straight line X=0.0 passes right through Carousel at X=-0.07, Y=17.92)
    # =========================================================================
    print("\n" + "=" * 80)
    print("[시나리오] Bravo(Drone2) RTH 회전목마 정면 관통 복귀 & ORCA 자율 회피 실측")
    print(f"  - 직선 복귀 경로: 월드 (0.0, 30.0) -> (0.0, 3.5)")
    print(f"  - 관통 장애물 위치: {target_obstacle_name} (X={obs_x:.2f}m, Y={obs_y:.2f}m)")
    print("=" * 80)

    # 20Hz High Frequency Sampler
    is_sampling = True
    samples = []
    min_obs_dist = 999.0
    max_lateral_dev = 0.0
    collision_count = 0
    collision_details = []
    last_col_timestamp = 0

    def sampler_loop():
        nonlocal is_sampling, min_obs_dist, max_lateral_dev, collision_count, last_col_timestamp
        sampler_client = airsim.MultirotorClient(timeout_value=2)
        sampler_client.confirmConnection()

        while is_sampling:
            t = time.time()
            try:
                st = sampler_client.getMultirotorState(vehicle_name=bravo_vname)
                pos = st.kinematics_estimated.position
                vel = st.kinematics_estimated.linear_velocity
                col = sampler_client.simGetCollisionInfo(vehicle_name=bravo_vname)

                wx = pos.x_val
                wy = pos.y_val + bravo_off_y
                wz = pos.z_val

                is_airborne = (st.landed_state != airsim.LandedState.Landed) and (wz < -1.0)
                if is_airborne and col.has_collided and col.time_stamp > last_col_timestamp:
                    last_col_timestamp = col.time_stamp
                    collision_count += 1
                    collision_details.append({
                        "time": round(t, 2),
                        "object_name": col.object_name,
                        "impact_point": [round(col.impact_point.x_val, 2), round(col.impact_point.y_val, 2), round(col.impact_point.z_val, 2)]
                    })

                # Distance to carousel
                d_obs = dist2d(wx, wy, obs_x, obs_y)
                if is_airborne and d_obs < min_obs_dist:
                    min_obs_dist = d_obs

                # Lateral deviation from straight line (X=0.0)
                # Straight line connecting (0, 30) and (0, 3.5) has x = 0.0
                lateral_dev = abs(wx - 0.0)
                if is_airborne and (3.5 <= wy <= 30.0):
                    if lateral_dev > max_lateral_dev:
                        max_lateral_dev = lateral_dev

                samples.append({
                    "time": round(t, 2),
                    "pos_world": [round(wx, 3), round(wy, 3), round(wz, 3)],
                    "vel": [round(vel.x_val, 2), round(vel.y_val, 2), round(vel.z_val, 2)],
                    "dist_to_obstacle": round(d_obs, 2),
                    "lateral_deviation": round(lateral_dev, 2)
                })
            except Exception:
                pass
            time.sleep(0.05)

    sampler_thread = threading.Thread(target=sampler_loop, daemon=True)
    sampler_thread.start()

    # Trigger Bravo RTH
    t_rth_start = time.time()
    print("  - Bravo(Drone2) RTH 복귀 명령 전송...")
    rth_res = api_post("/api/rth", {"drone_id": "Drone2"})
    rth_elapsed = time.time() - t_rth_start
    print(f"  - RTH 응답 완료 (소요시간: {rth_elapsed:.2f}s) | {rth_res.get('status')}: {rth_res.get('message')}")

    time.sleep(3.0)
    is_sampling = False
    sampler_thread.join(timeout=3.0)

    # Final Bravo landing position check
    bravo_state_final = client.getMultirotorState(vehicle_name=bravo_vname)
    bp_final = bravo_state_final.kinematics_estimated.position
    bravo_final_world = (bp_final.x_val, bp_final.y_val + bravo_off_y, bp_final.z_val)
    home_dist = dist2d(bravo_final_world[0], bravo_final_world[1], 0.0, 3.5)

    print("\n" + "=" * 80)
    print("[실환경 정적 장애물 회피 실측 종합 결과]")
    print(f"  - 총 수집 샘플 수: {len(samples)} frames (20Hz 샘플링)")
    print(f"  - 직선 선호 경로 대비 횡방향 최대 회피 편차(Max Lateral Deviation): {max_lateral_dev:.2f}m (기준: >= {MIN_AVOIDANCE_LATERAL_DEV_M}m)")
    print(f"  - 비행 중 회전목마와의 최소 실측 이격 거리(Min Obstacle Distance): {min_obs_dist:.2f}m (요구 기준: >= {REQUIRED_MIN_OBSTACLE_DIST_M}m)")
    print(f"  - 충돌 발생 횟수: {collision_count}회")
    print(f"  - Bravo 최종 착륙 위치(월드): ({bravo_final_world[0]:.2f}, {bravo_final_world[1]:.2f}, {bravo_final_world[2]:.2f}) | 홈 오차: {home_dist:.2f}m (기준: <= 1.5m)")

    # Criteria
    pass_dev = max_lateral_dev >= MIN_AVOIDANCE_LATERAL_DEV_M
    pass_dist = min_obs_dist >= REQUIRED_MIN_OBSTACLE_DIST_M
    pass_col = collision_count == 0
    pass_home = home_dist <= 1.5

    print(f"\n[항목별 판정]")
    print(f"  1. ORCA 횡방향 자율 회피 기동 입증 (max lateral dev >= {MIN_AVOIDANCE_LATERAL_DEV_M}m): {'PASS' if pass_dev else 'FAIL'} ({max_lateral_dev:.2f}m)")
    print(f"  2. 정적 장애물 최소 안전 이격 유지 (min dist >= {REQUIRED_MIN_OBSTACLE_DIST_M}m): {'PASS' if pass_dist else 'FAIL'} ({min_obs_dist:.2f}m)")
    print(f"  3. 편대 무충돌 달성 (collision == 0): {'PASS' if pass_col else 'FAIL'} ({collision_count} collisions)")
    print(f"  4. RTH 홈 원점 복귀 및 착륙 정합성 (home error <= 1.5m): {'PASS' if pass_home else 'FAIL'} ({home_dist:.2f}m)")

    final_pass = pass_dev and pass_dist and pass_col and pass_home
    print(f"\n=> 최종 테스트 판정: {'[PASS] ALL PASSED (정적 장애물 ORCA 회피 완전 검증 성공)' if final_pass else '[FAIL] FAILED'}")
    print("=" * 80)

    # Control Group Comparison Data
    # In a straight line without obstacle avoidance, lateral deviation = 0.0m, and collision occurs at Y ~ 17.92m (Carousel center X=-0.07m).
    control_group_analysis = {
        "without_static_orca_prediction": {
            "expected_trajectory": "Straight line along X=0.0m from Y=30.0m to Y=3.5m",
            "expected_lateral_deviation_m": 0.0,
            "expected_min_distance_to_carousel_m": 0.07,
            "expected_collision": True,
            "collision_point_estimate": [0.0, 17.92, -4.0]
        },
        "with_static_orca_measured": {
            "max_lateral_deviation_m": round(max_lateral_dev, 2),
            "min_measured_distance_m": round(min_obs_dist, 2),
            "actual_collision_count": collision_count,
            "home_landing_error_m": round(home_dist, 2)
        }
    }

    # Save report
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "test_passed": final_pass,
        "map_id": SIM_MAP_ID,
        "target_obstacle": {
            "name": target_obstacle_name,
            "position_world": [obs_x, obs_y, obs_z]
        },
        "metrics": {
            "required_min_distance_m": REQUIRED_MIN_OBSTACLE_DIST_M,
            "min_measured_distance_m": round(min_obs_dist, 2),
            "max_lateral_deviation_m": round(max_lateral_dev, 2),
            "total_collisions": collision_count,
            "home_landing_error_m": round(home_dist, 2),
            "rth_duration_sec": round(rth_elapsed, 2)
        },
        "control_group_comparison": control_group_analysis,
        "collision_details": collision_details,
        "samples_count": len(samples),
        "trajectory_samples": samples
    }

    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"  - 실측 리포트 저장 완료: {REPORT_PATH}\n")

    assert final_pass, f"Test failed! dev={max_lateral_dev:.2f}m, min_dist={min_obs_dist:.2f}m, collisions={collision_count}, home_err={home_dist:.2f}m"


if __name__ == "__main__":
    run_test()
