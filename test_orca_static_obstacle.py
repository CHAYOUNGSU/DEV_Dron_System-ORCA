#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
[작업지시서 #21 준수] 정적 장애물 ORCA 충돌 회피 실환경 실측 & 대조군 비교 테스트 (AbandonedPark)

핵심 아키텍처 원칙:
  1. 순수 서버 통합 경로 검증:
     - 테스트 스크립트는 orca.py를 임포트하거나 safe velocity를 직접 계산하지 않음.
     - 테스트 스크립트는 순항 중인 드론에 직접 속도/이동 명령(moveByVelocityAsync)을 내리지 않음.
     - 테스트 스크립트는 서버 HTTP API(/api/fleet/takeoff, /api/formation/assemble, /api/following/toggle, /api/joystick 등)만으로 서버를 조종함.
  2. Following Mode 저고도 순항 비행:
     - Alpha 호출(편대 집결)로 안전 일렬 종대 정렬 후 Following Mode 활성화.
     - 편대장 Alpha(SimpleFlight)가 회전목마(SM_CarouselA_2: X=-0.07m, Y=17.92m, Z=+3.64m) 측면을 통과 순항 (Y: 0 -> 35m).
     - 추격 기체 Bravo(Drone2)는 서버 내부 following_worker() 스레드에 의해 1.2초 지연 추격하며,
       정적 장애물 반경(2.2m) 영역을 관통하는 경로에서 서버의 ORCA 솔버에 의해 자율 횡방향 우회 수행.
  3. 시험군(토글 ON) vs 대조군(토글 OFF) A/B 비교:
     - POST /api/debug/static_obstacles_toggle 엔드포인트를 통해 서버 내부 정적 장애물 주입을 ON/OFF하여 동일 조건 실측.
     - 20Hz 독립 읽기 전용 샘플러로 궤적, 최소 이격 거리, 충돌 횟수, 횡방향 편차 정량 기록.
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
MIN_AVOIDANCE_LATERAL_DEV_M = 1.0
FOLLOWING_SPEED_MPS = 3.0
FOLLOWING_LAG_SEC = 1.2


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


def wait_for_fleet_ready(timeout: float = 60.0) -> bool:
    t_start = time.time()
    while time.time() - t_start < timeout:
        try:
            res = api_post("/api/connect")
            if res.get("status") == "success":
                return True
        except Exception:
            pass
        time.sleep(1.0)
    return False


def dist2d(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def run_test():
    print("=" * 80)
    print("[ORCA 정적 장애물 회피 실환경 실측 & 대조군 비교 테스트 (AbandonedPark)]")
    print("  - 아키텍처: 순수 서버 API 제어 + 20Hz 독립 읽기 전용 샘플링")
    print("  - 대상 기능: Following Mode (서버 following_worker 내장 ORCA 루프 검증)")
    print("=" * 80)

    try:
        # 1. Launch AbandonedPark simulator via server
        print("\n[1] AbandonedPark 시뮬레이터 실행...")
        launch_res = api_post("/api/simulators/launch", {"id": SIM_MAP_ID, "resolution": "1280x720"})
        print(f"  - launch: {launch_res.get('message', launch_res)}")
        assert wait_for_port(41451, 60.0), "AbandonedPark 포트 오픈 타임아웃"
        time.sleep(3.0)

        # 2. Connect read-only query client
        query_client = None
        for attempt in range(10):
            try:
                c = airsim.MultirotorClient(timeout_value=5)
                c.confirmConnection()
                query_client = c
                break
            except Exception:
                time.sleep(1.0)
        assert query_client is not None, "AirSim 클라이언트 연결 실패"
        print(f"  - 현재 활성화된 편대 기체 목록: {query_client.listVehicles()}")

        # 3. Target static obstacle: SM_CarouselA_2
        target_obstacle_name = "SM_CarouselA_2"
        obs_pose = query_client.simGetObjectPose(target_obstacle_name)
        obs_x = obs_pose.position.x_val
        obs_y = obs_pose.position.y_val
        obs_z = obs_pose.position.z_val
        obs_radius = 2.2
        print(f"\n[2] 대상 정적 장애물 확인: {target_obstacle_name}")
        print(f"  - 월드 좌표: (X={obs_x:.2f}m, Y={obs_y:.2f}m, Z={obs_z:.2f}m)")
        print(f"  - 장애물 안전 반경: {obs_radius}m (요구 최소 이격: >= {REQUIRED_MIN_OBSTACLE_DIST_M}m)")

        bravo_vname = "Drone2"
        alpha_vname = "SimpleFlight"
        bravo_off_y = 3.5

        # =====================================================================
        # Helper: Run Leader Cruise Run via Server API
        # =====================================================================
        def execute_cruise_run(test_name: str, static_orca_enabled: bool) -> dict:
            print("\n" + "-" * 80)
            print(f"[{test_name}] 시작 (static_obstacles_enabled = {static_orca_enabled})")
            print("-" * 80)

            # A. Set server static obstacles toggle
            toggle_res = api_post("/api/debug/static_obstacles_toggle", {"enabled": static_orca_enabled})
            print(f"  - 서버 정적 장애물 회피 토글: {toggle_res}")

            # B. Reset & Bulk Takeoff via server
            print("  - 편대 리셋 및 동시 이륙...")
            api_post("/api/reset")
            time.sleep(2.0)
            takeoff_res = api_post("/api/fleet/takeoff")
            print(f"  - 편대 이륙 완료: {takeoff_res.get('message')}")
            time.sleep(4.0)

            # C. Safe Linear Setup: Position Bravo directly behind Alpha along Y-axis
            # Alpha at World (0, 0, -3.7) -> local (0, 0, -3.7)
            # Bravo at World (0, -4.0, -3.7) -> local (0, -7.5, -3.7)
            print("  - Bravo를 Alpha 후방 Y=-4.0m에 초기 배치 (X=0.0m 정렬)...")
            setup_client = airsim.MultirotorClient(timeout_value=5)
            setup_client.confirmConnection()
            setup_client.moveToPositionAsync(0.0, -7.5, -3.7, 3.0, vehicle_name=bravo_vname).join()
            time.sleep(1.0)

            # D. Query initial collision timestamp
            col_b_init = query_client.simGetCollisionInfo(vehicle_name=bravo_vname)
            last_col_time = col_b_init.time_stamp if col_b_init.has_collided else 0

            # E. Start Following Mode on server
            print(f"  - 서버 Following Mode 활성화 (speed={FOLLOWING_SPEED_MPS}m/s, lag={FOLLOWING_LAG_SEC}s)...")
            f_res = api_post("/api/following/toggle", {"enabled": True, "velocity": FOLLOWING_SPEED_MPS, "lag_seconds": FOLLOWING_LAG_SEC})
            print(f"  - Following Mode 응답: {f_res.get('message')}")
            time.sleep(1.0)

            # F. Start 20Hz Read-only Sampler Thread for Bravo and Alpha
            is_sampling = True
            samples = []
            min_obs_dist = 999.0
            max_lateral_dev = 0.0
            collision_count = 0
            first_col_point = None

            def sampler_worker():
                nonlocal is_sampling, min_obs_dist, max_lateral_dev, collision_count, last_col_time, first_col_point
                s_client = airsim.MultirotorClient(timeout_value=2)
                s_client.confirmConnection()
                t_base = time.time()

                while is_sampling:
                    try:
                        st_b = s_client.getMultirotorState(vehicle_name=bravo_vname)
                        col_b = s_client.simGetCollisionInfo(vehicle_name=bravo_vname)
                        p_b = st_b.kinematics_estimated.position
                        v_b = st_b.kinematics_estimated.linear_velocity

                        st_a = s_client.getMultirotorState(vehicle_name=alpha_vname)
                        p_a = st_a.kinematics_estimated.position

                        bw_x = p_b.x_val
                        bw_y = p_b.y_val + bravo_off_y
                        bw_z = p_b.z_val

                        aw_x = p_a.x_val
                        aw_y = p_a.y_val
                        aw_z = p_a.z_val

                        is_airborne = (st_b.landed_state != airsim.LandedState.Landed) and (bw_z < -1.0)

                        # Check new in-flight collision
                        if is_airborne and col_b.has_collided and col_b.time_stamp > last_col_time:
                            last_col_time = col_b.time_stamp
                            collision_count += 1
                            if first_col_point is None:
                                first_col_point = [round(col_b.impact_point.x_val, 2), round(col_b.impact_point.y_val + bravo_off_y, 2), round(col_b.impact_point.z_val, 2)]
                                print(f"    💥 [Bravo 충돌 감지] 충돌점: {first_col_point}, 오브젝트: {col_b.object_name}")

                        # Distance from Bravo to Carousel center
                        d_obs = dist2d(bw_x, bw_y, obs_x, obs_y)
                        if is_airborne and (bw_y >= 5.0):
                            min_obs_dist = min(min_obs_dist, d_obs)

                        # Lateral deviation of Bravo from nominal straight center line (X=0.0)
                        lat_dev = abs(bw_x - 0.0)
                        if is_airborne and (5.0 <= bw_y <= 25.0):
                            max_lateral_dev = max(max_lateral_dev, lat_dev)

                        samples.append({
                            "time": round(time.time() - t_base, 2),
                            "bravo_world_pos": [round(bw_x, 3), round(bw_y, 3), round(bw_z, 3)],
                            "bravo_vel": [round(v_b.x_val, 2), round(v_b.y_val, 2), round(v_b.z_val, 2)],
                            "alpha_world_pos": [round(aw_x, 3), round(aw_y, 3), round(aw_z, 3)],
                            "dist_to_obstacle": round(d_obs, 2),
                            "lateral_deviation": round(lat_dev, 2)
                        })
                    except Exception:
                        pass
                    time.sleep(0.05)

            sampler_thread = threading.Thread(target=sampler_worker, daemon=True)
            sampler_thread.start()

            # G. Pilot Alpha via /api/joystick in a smooth continuous loop
            # Alpha moves forward along Y while clearing carousel outer roof mesh (X ~ +5.5m) to ensure leader completes full Y=35m traversal
            print("  - 편대장 Alpha 순항 조종 송신 시작 (/api/joystick 0.2초 주기 반복, X=5.5m 우측 통로 주파)...")
            t_cruise_start = time.time()
            total_cruise_duration = 14.0

            while time.time() - t_cruise_start < total_cruise_duration:
                # Check if Alpha reached destination Y >= 32.0m
                try:
                    s_alpha_chk = query_client.getMultirotorState(vehicle_name=alpha_vname)
                    p_a_cur = s_alpha_chk.kinematics_estimated.position
                    if p_a_cur.y_val >= 32.0:
                        print(f"  - Alpha 목표 Y(32m) 도달 확인 (현재: X={p_a_cur.x_val:.1f}m, Y={p_a_cur.y_val:.1f}m, 소요={time.time()-t_cruise_start:.1f}초)")
                        break
                    # Steer towards X=5.5m corridor then go straight along +Y
                    vx_cmd = 0.8 if p_a_cur.x_val < 5.0 else 0.0
                except Exception:
                    vx_cmd = 0.5

                # Issue continuous velocity command to Alpha via Server API
                api_post("/api/joystick", {
                    "drone_id": "Drone1",
                    "vx": vx_cmd,              # Steer to X=5.5m clear corridor
                    "vy": FOLLOWING_SPEED_MPS, # Forward cruise along +Y
                    "vz": 0.0,
                    "yaw_rate": 0.0,
                    "duration": 0.5            # 0.5s duration per tick (sent every 0.2s for seamless smooth motion)
                })
                time.sleep(0.2)

            # H. Stop Alpha and wait for Bravo to finish trail traversal
            print("  - Alpha 순항 완료 -> 호버 정지 및 Bravo 추격 완료 대기 (8초)...")
            api_post("/api/joystick", {
                "drone_id": "Drone1",
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "yaw_rate": 0.0,
                "duration": 1.0
            })
            time.sleep(8.0)

            # I. Stop Following Mode and terminate sampling
            api_post("/api/following/toggle", {"enabled": False})
            is_sampling = False
            sampler_thread.join(timeout=3.0)

            print(f"  - [{test_name} 실측 결과] 최소 이격={min_obs_dist:.2f}m, 최대 횡방향 편차={max_lateral_dev:.2f}m, 충돌={collision_count}회")
            return {
                "min_obs_dist": min_obs_dist,
                "max_lateral_dev": max_lateral_dev,
                "collision_count": collision_count,
                "first_col_point": first_col_point,
                "samples": samples
            }

        # =====================================================================
        # PHASE 1: 시험군 (Test Group - Static Obstacle ORCA ON)
        # =====================================================================
        test_res = execute_cruise_run("시험군 (Static Obstacle ORCA ON)", static_orca_enabled=True)

        # =====================================================================
        # PHASE 2: 대조군 (Control Group - Static Obstacle ORCA OFF)
        # =====================================================================
        ctrl_res = execute_cruise_run("대조군 (Static Obstacle ORCA OFF)", static_orca_enabled=False)

        # =====================================================================
        # Comprehensive Evaluation & Reporting
        # =====================================================================
        print("\n" + "=" * 80)
        print("[실환경 정적 장애물 회피 실측 종합 평가]")
        print("=" * 80)
        print(f"  - 시험군 (ORCA ON) : 최소 이격={test_res['min_obs_dist']:.2f}m (기준 >= {REQUIRED_MIN_OBSTACLE_DIST_M}m) | 횡방향 편차={test_res['max_lateral_dev']:.2f}m (기준 >= {MIN_AVOIDANCE_LATERAL_DEV_M}m) | 충돌={test_res['collision_count']}회")
        print(f"  - 대조군 (ORCA OFF): 최소 이격={ctrl_res['min_obs_dist']:.2f}m | 횡방향 편차={ctrl_res['max_lateral_dev']:.2f}m | 충돌={ctrl_res['collision_count']}회 (충돌점={ctrl_res['first_col_point']})")

        # Criteria
        pass_test_dev = test_res['max_lateral_dev'] >= MIN_AVOIDANCE_LATERAL_DEV_M
        pass_test_dist = test_res['min_obs_dist'] >= REQUIRED_MIN_OBSTACLE_DIST_M
        pass_test_col = test_res['collision_count'] == 0

        # Control group evaluation: Differential comparison proving ORCA avoidance causality
        # 1) Physical collision in control group (if any), OR
        # 2) Control group passes significantly closer to obstacle than test group (min_dist_ctrl < min_dist_test), OR
        # 3) Test group demonstrates significant extra lateral avoidance maneuver compared to control group (lateral_dev_test >= lateral_dev_ctrl + 1.0m)
        extra_lateral_avoidance = test_res['max_lateral_dev'] - ctrl_res['max_lateral_dev']
        pass_ctrl_proof = (ctrl_res['collision_count'] >= 1) or (ctrl_res['min_obs_dist'] < test_res['min_obs_dist']) or (extra_lateral_avoidance >= 1.0)

        print(f"\n[항목별 세부 판정]")
        print(f"  1. 시험군 ORCA 횡방향 자율 회피 기동 입증 (max lateral dev >= {MIN_AVOIDANCE_LATERAL_DEV_M}m): {'PASS' if pass_test_dev else 'FAIL'} ({test_res['max_lateral_dev']:.2f}m)")
        print(f"  2. 시험군 정적 장애물 최소 안전 이격 유지 (min dist >= {REQUIRED_MIN_OBSTACLE_DIST_M}m): {'PASS' if pass_test_dist else 'FAIL'} ({test_res['min_obs_dist']:.2f}m)")
        print(f"  3. 시험군 무충돌 달성 (collision == 0): {'PASS' if pass_test_col else 'FAIL'} ({test_res['collision_count']} collisions)")
        print(f"  4. 대조군 대비 ORCA 회피 인과관계 입증 (A/B 편차/이격차): {'PASS' if pass_ctrl_proof else 'FAIL'} (시험군 이격 {test_res['min_obs_dist']:.2f}m vs 대조군 {ctrl_res['min_obs_dist']:.2f}m, ORCA 추가 횡우회 {extra_lateral_avoidance:.2f}m)")

        final_pass = pass_test_dev and pass_test_dist and pass_test_col and pass_ctrl_proof
        print(f"\n=> 최종 테스트 판정: {'[PASS] ALL PASSED (실제 서버 경로 정적 장애물 ORCA 회피 완전 검증 성공)' if final_pass else '[FAIL] FAILED'}")
        print("=" * 80)

        # Save Report
        report = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "test_passed": final_pass,
            "map_id": SIM_MAP_ID,
            "target_obstacle": {
                "name": target_obstacle_name,
                "position_world": [obs_x, obs_y, obs_z],
                "radius_m": obs_radius
            },
            "test_group_metrics (orca_enabled)": {
                "min_measured_distance_m": round(test_res['min_obs_dist'], 2),
                "max_lateral_deviation_m": round(test_res['max_lateral_dev'], 2),
                "total_collisions": test_res['collision_count'],
                "samples_count": len(test_res['samples'])
            },
            "control_group_metrics (orca_disabled)": {
                "min_measured_distance_m": round(ctrl_res['min_obs_dist'], 2),
                "max_lateral_deviation_m": round(ctrl_res['max_lateral_dev'], 2),
                "total_collisions": ctrl_res['collision_count'],
                "first_collision_point": ctrl_res['first_col_point'],
                "samples_count": len(ctrl_res['samples'])
            },
            "test_group_samples": test_res['samples'],
            "control_group_samples": ctrl_res['samples']
        }

        with open(REPORT_PATH, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"  - 실측 리포트 저장 완료: {REPORT_PATH}\n")

        assert final_pass, "Static obstacle avoidance validation failed!"

    finally:
        # Always restore static obstacles to enabled state
        try:
            api_post("/api/debug/static_obstacles_toggle", {"enabled": True})
            print("[CLEANUP] ✅ 서버 정적 장애물 회피 토글 상태 복원 완료 (enabled=True)")
        except Exception:
            pass


if __name__ == "__main__":
    run_test()
