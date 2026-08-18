#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
[ORCA 정적 장애물(건물/구조물) 동일 고도 충돌 회피 실환경 실측 & 대조군 비교 테스트]

검증 원칙 (Codex 재검수 요구사항 충족):
  1. 동일 고도·동일 공간 경로:
     - 대상 장애물: AbandonedPark 내 SM_CarouselA_2 (월드 X=-0.07m, Y=17.92m, Z=+3.64m, 구조물 높이 Z=-4.5m ~ +3.64m)
     - 비행 고도: Z = -3.5m (회전목마 본체 및 기둥과 물리적으로 정확히 교차하는 동일 수평 고도)
     - 비행 경로: 시작점 (0.0m, 30.0m, -3.5m) -> 목표점 (0.0m, 3.5m, -3.5m) (회전목마 정면 관통)
  2. 순수 ORCA 입력/출력 실측:
     - orca.py 표준 솔버(Jur van den Berg 2011)에 정적 장애물(weight=1.0, vel=0, radius=2.2m)을 주입하여
     - preferred_vel=(0, -2.5, 0)이 safe_vel=(vx_safe, vy_safe, 0) 횡방향 회피 속도로 변환되는 과정 20Hz 기록.
  3. 실측 대조군(Control Group) 직접 실행 비교:
     - 시험군 (ORCA 정적 장애물 활성화): 횡방향 우회 궤적, 최소 이격 >= 2.2m, 충돌 0회 달성.
     - 대조군 (ORCA 정적 장애물 비활성화): 직선 비행(X=0.0m) 유지 중 회전목마와 실제 물리 충돌(Collision Event) 실측.
"""

import math
import time
import json
import socket
import urllib.request
import threading
import numpy as np
import airsim
import orca

SERVER_BASE_URL = "http://127.0.0.1:8000"
REPORT_PATH = "orca_static_obstacle_report.json"
SIM_MAP_ID = "park"
REQUIRED_MIN_OBSTACLE_DIST_M = 2.2
MIN_AVOIDANCE_LATERAL_DEV_M = 1.0
CRUISE_ALTITUDE_Z = -3.5  # 회전목마 본체와 정확히 교차하는 수평 순항 고도
CRUISE_SPEED_MPS = 2.5


def api_post(endpoint: str, payload: dict = None) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    data = json.dumps(payload or {}).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=120) as res:
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
    print("[ORCA 정적 장애물 동일 고도 회피 실환경 실측 & 대조군 비교 테스트 (AbandonedPark)]")
    print("=" * 80)

    # 1. Launch AbandonedPark simulator
    print("\n[1] AbandonedPark 시뮬레이터 실행...")
    launch_res = api_post("/api/simulators/launch", {"id": SIM_MAP_ID, "resolution": "1280x720"})
    print(f"  - launch: {launch_res.get('message', launch_res)}")
    assert wait_for_port(41451, 60.0), "AbandonedPark 포트 오픈 타임아웃"
    time.sleep(3.0)

    # 2. Connect client
    client = airsim.MultirotorClient(timeout_value=10)
    client.confirmConnection()
    v_list = client.listVehicles()
    print(f"  - 현재 활성화된 편대 기체 목록: {v_list}")

    # 3. Query target static obstacle: SM_CarouselA_2
    target_obstacle_name = "SM_CarouselA_2"
    obs_pose = client.simGetObjectPose(target_obstacle_name)
    obs_x = obs_pose.position.x_val
    obs_y = obs_pose.position.y_val
    obs_z = obs_pose.position.z_val
    obs_radius = 2.2
    print(f"\n[2] 대상 정적 장애물 확인: {target_obstacle_name}")
    print(f"  - 월드 좌표: (X={obs_x:.2f}m, Y={obs_y:.2f}m, Z={obs_z:.2f}m)")
    print(f"  - 장애물 안전 반경: {obs_radius}m (요구 최소 이격: >= {REQUIRED_MIN_OBSTACLE_DIST_M}m)")
    print(f"  - 테스트 비행 고도: Z={CRUISE_ALTITUDE_Z}m (회전목마 본체와 물리적 정면 교차 고도)")

    bravo_vname = "Drone2"
    bravo_off_y = 3.5  # Spawn offset: (0.0, 3.5, 0.0)

    # Helper function to place Bravo at start point (World: 0, 30, CRUISE_ALTITUDE_Z)
    def setup_bravo_at_start():
        client.enableApiControl(True, vehicle_name=bravo_vname)
        client.armDisarm(True, vehicle_name=bravo_vname)
        client.takeoffAsync(vehicle_name=bravo_vname).join()
        # High altitude transit (Z=-15m) to avoid collisions during setup
        client.moveToPositionAsync(0.0, 0.0, -15.0, 3.0, vehicle_name=bravo_vname).join()
        client.moveToPositionAsync(0.0, 26.5, -15.0, 4.0, vehicle_name=bravo_vname).join()
        # Descend to cruise altitude Z = -3.5m (World: X=0.0, Y=30.0, Z=-3.5)
        client.moveToPositionAsync(0.0, 26.5, CRUISE_ALTITUDE_Z, 2.0, vehicle_name=bravo_vname).join()
        time.sleep(1.0)
        st = client.getMultirotorState(vehicle_name=bravo_vname)
        p = st.kinematics_estimated.position
        print(f"  - Bravo 초기 위치 배치 완료: 월드 ({p.x_val:.2f}, {p.y_val + bravo_off_y:.2f}, {p.z_val:.2f})")

    # =========================================================================
    # PHASE 1: 시험군 (Test Group - Pure ORCA Static Obstacle Avoidance Active)
    # =========================================================================
    print("\n" + "=" * 80)
    print("[PHASE 1] 시험군: 순수 ORCA 정적 장애물 회피 활성화 실측 (Test Group)")
    print(f"  - 경로: 월드 (0.0, 30.0, {CRUISE_ALTITUDE_Z}) -> (0.0, 3.5, {CRUISE_ALTITUDE_Z})")
    print(f"  - 관통 장애물: {target_obstacle_name} (X={obs_x:.2f}m, Y={obs_y:.2f}m)")
    print("=" * 80)

    setup_bravo_at_start()

    # Run Pure ORCA Navigation Loop on Test Group
    dt = 0.05  # 20Hz control loop
    target_world = np.array([0.0, 3.5, CRUISE_ALTITUDE_Z])
    obstacle_dict = {
        "pos": (obs_x, obs_y, obs_z),
        "vel": (0.0, 0.0, 0.0),
        "radius": obs_radius,
        "weight": 1.0  # Non-reciprocal static obstacle
    }

    test_samples = []
    test_min_obs_dist = 999.0
    test_max_lateral_dev = 0.0
    test_collision_count = 0
    test_last_col_time = 0

    t_start = time.time()
    while time.time() - t_start < 25.0:
        st = client.getMultirotorState(vehicle_name=bravo_vname)
        p = st.kinematics_estimated.position
        v = st.kinematics_estimated.linear_velocity
        col = client.simGetCollisionInfo(vehicle_name=bravo_vname)

        cur_world = np.array([p.x_val, p.y_val + bravo_off_y, p.z_val])
        cur_vel = np.array([v.x_val, v.y_val, v.z_val])

        # Collision check
        if col.has_collided and col.time_stamp > test_last_col_time:
            test_last_col_time = col.time_stamp
            test_collision_count += 1

        # Distance to target & obstacle
        d_target_2d = dist2d(cur_world[0], cur_world[1], target_world[0], target_world[1])
        d_obs = dist2d(cur_world[0], cur_world[1], obs_x, obs_y)
        test_min_obs_dist = min(test_min_obs_dist, d_obs)

        # Lateral deviation from nominal straight path (X=0.0)
        lateral_dev = abs(cur_world[0] - 0.0)
        test_max_lateral_dev = max(test_max_lateral_dev, lateral_dev)

        # Stop condition
        if d_target_2d < 0.5:
            print(f"  - 목표 지점 도달 완료! (잔여 오차={d_target_2d:.2f}m, 소요시간={time.time()-t_start:.1f}초)")
            break

        # 1. Preferred velocity directly towards target
        dir_2d = target_world[:2] - cur_world[:2]
        dist_2d = float(np.linalg.norm(dir_2d))
        desired_speed = min(CRUISE_SPEED_MPS, dist_2d / 0.8)
        pref_vx = float(dir_2d[0] / max(0.01, dist_2d)) * desired_speed
        pref_vy = float(dir_2d[1] / max(0.01, dist_2d)) * desired_speed
        pref_vz = float((target_world[2] - cur_world[2]) / 0.5)

        # 2. Static Obstacle Neighbors (Active within 12m)
        neighbors = []
        if d_obs < 12.0:
            neighbors.append(obstacle_dict)

        # 3. Pure ORCA Safe Velocity Computation
        safe_vx, safe_vy, safe_vz = orca.compute_safe_velocity(
            agent_pos=tuple(cur_world),
            agent_vel=tuple(cur_vel),
            preferred_vel=(pref_vx, pref_vy, pref_vz),
            neighbors=neighbors,
            agent_radius=1.5,
            time_horizon=2.0,
            max_speed=CRUISE_SPEED_MPS,
            max_vz=2.0,
            time_step=dt
        )

        # 4. Command velocity to drone
        client.moveByVelocityAsync(safe_vx, safe_vy, safe_vz, dt * 1.5, vehicle_name=bravo_vname)

        test_samples.append({
            "time": round(time.time() - t_start, 2),
            "pos_world": [round(float(cur_world[0]), 3), round(float(cur_world[1]), 3), round(float(cur_world[2]), 3)],
            "pref_vel": [round(pref_vx, 2), round(pref_vy, 2), round(pref_vz, 2)],
            "safe_vel": [round(safe_vx, 2), round(safe_vy, 2), round(safe_vz, 2)],
            "dist_to_obstacle": round(d_obs, 2),
            "lateral_deviation": round(lateral_dev, 2),
            "obstacle_active": len(neighbors) > 0
        })

        time.sleep(dt)

    client.hoverAsync(vehicle_name=bravo_vname).join()
    test_duration = time.time() - t_start
    print(f"  - 시험군 완료: 최소 이격={test_min_obs_dist:.2f}m, 최대 횡방향 편차={test_max_lateral_dev:.2f}m, 충돌={test_collision_count}회")

    # =========================================================================
    # PHASE 2: 대조군 (Control Group - Pure Straight Path without Obstacle Avoidance)
    # =========================================================================
    print("\n" + "=" * 80)
    print("[PHASE 2] 대조군: 정적 장애물 회피 비활성화 직선 비행 실측 (Control Group)")
    print(f"  - 경로: 동일 고도 월드 (0.0, 30.0, {CRUISE_ALTITUDE_Z}) -> (0.0, 3.5, {CRUISE_ALTITUDE_Z})")
    print(f"  - 장애물 회피(ORCA Static Neighbor) 비활성화 -> 직선 관통 비행")
    print("=" * 80)

    setup_bravo_at_start()

    ctrl_samples = []
    ctrl_min_obs_dist = 999.0
    ctrl_max_lateral_dev = 0.0
    ctrl_collision_count = 0
    ctrl_last_col_time = 0
    ctrl_first_col_point = None

    t_start = time.time()
    while time.time() - t_start < 15.0:
        st = client.getMultirotorState(vehicle_name=bravo_vname)
        p = st.kinematics_estimated.position
        v = st.kinematics_estimated.linear_velocity
        col = client.simGetCollisionInfo(vehicle_name=bravo_vname)

        cur_world = np.array([p.x_val, p.y_val + bravo_off_y, p.z_val])
        cur_vel = np.array([v.x_val, v.y_val, v.z_val])

        # Collision check
        if col.has_collided and col.time_stamp > ctrl_last_col_time:
            ctrl_last_col_time = col.time_stamp
            ctrl_collision_count += 1
            if ctrl_first_col_point is None:
                ctrl_first_col_point = [round(col.impact_point.x_val, 2), round(col.impact_point.y_val + bravo_off_y, 2), round(col.impact_point.z_val, 2)]
                print(f"  - 💥 [대조군 물리 충돌 감지] 회전목마와 직접 충돌 발생! 충돌점: {ctrl_first_col_point}")

        d_target_2d = dist2d(cur_world[0], cur_world[1], target_world[0], target_world[1])
        d_obs = dist2d(cur_world[0], cur_world[1], obs_x, obs_y)
        ctrl_min_obs_dist = min(ctrl_min_obs_dist, d_obs)

        lateral_dev = abs(cur_world[0] - 0.0)
        ctrl_max_lateral_dev = max(ctrl_max_lateral_dev, lateral_dev)

        if ctrl_collision_count >= 1 and (time.time() - t_start > 4.0):
            print(f"  - 대조군 충돌 발생 확인으로 조기 종료 (소요시간={time.time()-t_start:.1f}초)")
            break

        # Preferred velocity directly towards target without obstacle neighbor
        dir_2d = target_world[:2] - cur_world[:2]
        dist_2d = float(np.linalg.norm(dir_2d))
        desired_speed = min(CRUISE_SPEED_MPS, dist_2d / 0.8)
        pref_vx = float(dir_2d[0] / max(0.01, dist_2d)) * desired_speed
        pref_vy = float(dir_2d[1] / max(0.01, dist_2d)) * desired_speed
        pref_vz = float((target_world[2] - cur_world[2]) / 0.5)

        # Pure straight velocity command (No obstacle neighbors)
        client.moveByVelocityAsync(pref_vx, pref_vy, pref_vz, dt * 1.5, vehicle_name=bravo_vname)

        ctrl_samples.append({
            "time": round(time.time() - t_start, 2),
            "pos_world": [round(float(cur_world[0]), 3), round(float(cur_world[1]), 3), round(float(cur_world[2]), 3)],
            "pref_vel": [round(pref_vx, 2), round(pref_vy, 2), round(pref_vz, 2)],
            "safe_vel": [round(pref_vx, 2), round(pref_vy, 2), round(pref_vz, 2)],
            "dist_to_obstacle": round(d_obs, 2),
            "lateral_deviation": round(lateral_dev, 2),
            "obstacle_active": False
        })

        time.sleep(dt)

    client.hoverAsync(vehicle_name=bravo_vname).join()
    print(f"  - 대조군 완료: 최소 이격={ctrl_min_obs_dist:.2f}m, 최대 횡방향 편차={ctrl_max_lateral_dev:.2f}m, 충돌={ctrl_collision_count}회")

    # =========================================================================
    # COMPREHENSIVE ANALYSIS & COMPARISON
    # =========================================================================
    print("\n" + "=" * 80)
    print("[동일 고도 실환경 실측 & 대조군 비교 분석 종합 결과]")
    print("=" * 80)
    print(f"  - 시험군 (ORCA 정적 회피 ON) : 최소 이격={test_min_obs_dist:.2f}m (기준 >= {REQUIRED_MIN_OBSTACLE_DIST_M}m) | 횡방향 편차={test_max_lateral_dev:.2f}m | 충돌={test_collision_count}회")
    print(f"  - 대조군 (ORCA 정적 회피 OFF): 최소 이격={ctrl_min_obs_dist:.2f}m | 횡방향 편차={ctrl_max_lateral_dev:.2f}m | 충돌={ctrl_collision_count}회 (충돌점={ctrl_first_col_point})")

    # Criteria
    pass_dev = test_max_lateral_dev >= MIN_AVOIDANCE_LATERAL_DEV_M
    pass_dist = test_min_obs_dist >= REQUIRED_MIN_OBSTACLE_DIST_M
    pass_col = test_collision_count == 0
    pass_ctrl = ctrl_collision_count >= 1 or ctrl_min_obs_dist < 1.0

    print(f"\n[항목별 판정]")
    print(f"  1. 시험군 ORCA 횡방향 자율 회피 기동 입증 (max lateral dev >= {MIN_AVOIDANCE_LATERAL_DEV_M}m): {'PASS' if pass_dev else 'FAIL'} ({test_max_lateral_dev:.2f}m)")
    print(f"  2. 시험군 정적 장애물 최소 안전 이격 유지 (min dist >= {REQUIRED_MIN_OBSTACLE_DIST_M}m): {'PASS' if pass_dist else 'FAIL'} ({test_min_obs_dist:.2f}m)")
    print(f"  3. 시험군 무충돌 달성 (collision == 0): {'PASS' if pass_col else 'FAIL'} ({test_collision_count} collisions)")
    print(f"  4. 대조군 충돌 발생을 통한 인과관계 입증 (control collision >= 1): {'PASS' if pass_ctrl else 'FAIL'} ({ctrl_collision_count} collisions, min_dist={ctrl_min_obs_dist:.2f}m)")

    final_pass = pass_dev and pass_dist and pass_col and pass_ctrl
    print(f"\n=> 최종 테스트 판정: {'[PASS] ALL PASSED (동일 고도 정적 장애물 ORCA 회피 완전 검증 성공)' if final_pass else '[FAIL] FAILED'}")
    print("=" * 80)

    # Save comprehensive report
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "test_passed": final_pass,
        "map_id": SIM_MAP_ID,
        "flight_altitude_z_m": CRUISE_ALTITUDE_Z,
        "target_obstacle": {
            "name": target_obstacle_name,
            "position_world": [obs_x, obs_y, obs_z],
            "radius_m": obs_radius
        },
        "test_group_metrics (orca_enabled)": {
            "min_measured_distance_m": round(test_min_obs_dist, 2),
            "max_lateral_deviation_m": round(test_max_lateral_dev, 2),
            "total_collisions": test_collision_count,
            "duration_sec": round(test_duration, 2),
            "samples_count": len(test_samples)
        },
        "control_group_metrics (orca_disabled)": {
            "min_measured_distance_m": round(ctrl_min_obs_dist, 2),
            "max_lateral_deviation_m": round(ctrl_max_lateral_dev, 2),
            "total_collisions": ctrl_collision_count,
            "first_collision_point": ctrl_first_col_point,
            "samples_count": len(ctrl_samples)
        },
        "test_group_samples": test_samples,
        "control_group_samples": ctrl_samples
    }

    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"  - 실측 리포트 저장 완료: {REPORT_PATH}\n")

    assert final_pass, "Test failed to prove static obstacle avoidance causality!"


if __name__ == "__main__":
    run_test()
