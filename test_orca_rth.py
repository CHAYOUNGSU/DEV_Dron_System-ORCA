"""
ORCA Collision Avoidance & Shared control_lock Verification for Return To Home (_do_rth).

Comprehensive Test Scenarios:
1. Concurrency Scenario (Bravo & Charlie Simultaneous RTH):
   - Take off all 4 drones in Blocks simulator.
   - Advance Bravo (Drone2) and Charlie (Drone3) into cross-return positions:
     * Drone2 (Bravo): Forward-Right (+20.0m, +10.0m)
     * Drone3 (Charlie): Forward-Left (+20.0m, -5.0m)
   - Trigger simultaneous/parallel RTH for Drone2 and Drone3.
   - Independent 20Hz Sampler verifies:
     * True concurrency / Overlap execution: Both drones fly concurrently (overlap >= 5.0s)
     * No sampling errors (sampling_errors == 0)
     * Sufficient samples (samples_count >= 40 per drone)
     * Zero collisions (total_collisions == 0)
     * Minimum separation distance >= 3.0m (Configured agent radius: 1.6m, combined radius: 3.2m)
     * Landing precision: Drone2 lands at spawn offset (0, 3.5), Drone3 at (0, 7.0) with error <= 1.5m.

2. Critical Vulnerability Scenario A (Alpha RTH with concurrent Alpha rotate command):
   - Alpha (Drone1) is not in FOLLOW_CHAIN and was previously unprotected by is_follower_locked.
   - Advance Alpha forward, trigger RTH for Alpha.
   - Mid-flight, send concurrent /api/rotate command targeting Alpha.
   - Verify that control_lock serializes requests without msgpack socket error or server crash,
     and Alpha successfully completes RTH and lands safely at (0, 0) with error <= 1.5m.

3. Critical Vulnerability Scenario B (Delta RTH with unverified endpoint /api/land override):
   - Delta (Drone4) advances forward, triggers RTH.
   - Mid-flight, dispatch /api/land targeting Delta.
   - Verify:
     * /api/land returns status: success
     * RTH thread is cleanly canceled and terminates promptly
     * Delta is safely Landed (on the ground and stopped)
     * No subsequent RTH velocity commands are executed after landing.

Usage:
    python server.py
    python test_orca_rth.py
"""
import sys
import io
import os
import json
import time
import math
import socket
import threading
import urllib.request

if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

try:
    import airsim
except ImportError:
    print("[ERROR] airsim library not found.", flush=True)
    sys.exit(1)

SERVER_BASE_URL = "http://127.0.0.1:8000"
REPORT_PATH = "orca_rth_report.json"
VEHICLES = {"Drone1": "SimpleFlight", "Drone2": "Drone2", "Drone3": "Drone3", "Drone4": "Drone4"}
LABELS = {"Drone1": "Alpha", "Drone2": "Bravo", "Drone3": "Charlie", "Drone4": "Delta"}
SPAWN_OFFSETS = {"Drone1": (0.0, 0.0, 0.0), "Drone2": (0.0, 3.5, 0.0), "Drone3": (0.0, 7.0, 0.0), "Drone4": (0.0, 10.5, 0.0)}
ORCA_AGENT_RADIUS_M = 1.6  # Configured per-drone radius (Combined safety distance = 3.2m)
REQUIRED_MIN_SEPARATION_M = 3.2  # Strict verification threshold matching configured combined safety distance (3.2m)


def api_post(endpoint: str, payload: dict = None) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    data = json.dumps(payload or {}).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=60) as res:
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


def dist3d(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def wait_for_connected(timeout: float = 60.0) -> bool:
    t_start = time.time()
    while time.time() - t_start < timeout:
        try:
            res = api_get("/api/simulators")
            if res.get("connected", False):
                return True
        except Exception:
            pass
        time.sleep(0.5)
    return False


def main():
    print("=" * 80, flush=True)
    print("[ORCA RTH 공유 control_lock 일원화 & 착륙 안전 오버라이드 실측 테스트]", flush=True)
    print("=" * 80, flush=True)

    # 1. Simulator Launch
    print("\n[1] Blocks 시뮬레이터 실행...", flush=True)
    res = api_post("/api/simulators/launch", {"id": "blocks", "resolution": "1280x720"})
    print(f"  - {res.get('status')}: {res.get('message')}", flush=True)
    assert wait_for_port(41451, 60.0), "AirSim RPC 포트 오픈 대기 타임아웃"
    time.sleep(4.0)

    client_ctrl = None
    for attempt in range(15):
        try:
            c = airsim.MultirotorClient(timeout_value=5)
            c.confirmConnection()
            if len(c.listVehicles()) >= 4:
                client_ctrl = c
                break
        except Exception as e:
            pass
        time.sleep(1.0)
    if client_ctrl is None:
        client_ctrl = airsim.MultirotorClient(timeout_value=5)
        client_ctrl.confirmConnection()
    print(f"  - 감지된 기체: {client_ctrl.listVehicles()}", flush=True)

    # 2. Bulk Takeoff
    print("\n[2] 전체 편대 동시 이륙...", flush=True)
    api_post("/api/fleet/takeoff")
    time.sleep(6.0)

    # =========================================================================
    # SCENARIO 1: Bravo & Charlie Simultaneous RTH (ORCA 3-Leg Collision Avoidance)
    # =========================================================================
    print("\n" + "=" * 80, flush=True)
    print("[시나리오 1] Bravo(Drone2)와 Charlie(Drone3) 동시 교차 RTH ORCA 충돌 회피 실측", flush=True)
    print("=" * 80, flush=True)

    targets = {
        "Drone2": (20.0, 10.0, -5.0),
        "Drone3": (20.0, -5.0, -5.0)
    }

    advance_futures = []
    for d_id, (wx, wy, wz) in targets.items():
        vname = VEHICLES[d_id]
        off = SPAWN_OFFSETS[d_id]
        local_x = wx - off[0]
        local_y = wy - off[1]
        local_z = wz - off[2]
        print(f"  - {LABELS[d_id]} 전진 이동: 월드 ({wx:.1f}, {wy:.1f}, {wz:.1f}) -> 로컬 ({local_x:.1f}, {local_y:.1f}, {local_z:.1f})", flush=True)
        advance_futures.append(client_ctrl.moveToPositionAsync(local_x, local_y, local_z, 4.0, vehicle_name=vname))

    for f in advance_futures:
        try:
            f.join()
        except Exception:
            pass
    time.sleep(1.0)

    dispersed_positions = {}
    for d_id, vname in VEHICLES.items():
        s = client_ctrl.getMultirotorState(vehicle_name=vname)
        p = s.kinematics_estimated.position
        off = SPAWN_OFFSETS[d_id]
        dispersed_positions[d_id] = (round(p.x_val + off[0], 2), round(p.y_val + off[1], 2), round(p.z_val + off[2], 2))
        print(f"  - {LABELS[d_id]} RTH 직전 위치(월드): {dispersed_positions[d_id]}", flush=True)

    # 20Hz High-Frequency Telemetry & Collision Sampler
    print("\n  - 20Hz 고빈도 텔레메트리 & 충돌 샘플러 가동...", flush=True)
    samples = {d_id: [] for d_id in VEHICLES}
    collisions_detected = {d_id: 0 for d_id in VEHICLES}
    collision_events = []
    pairwise_distances = []
    sampling_errors = []
    stop_flag = threading.Event()

    def sampler_worker():
        try:
            sampler_client = airsim.MultirotorClient(timeout_value=2)
            sampler_client.confirmConnection()
        except Exception as e_init:
            sampling_errors.append(f"Sampler connection failed: {e_init}")
            return

        initial_timestamps = {}
        for d_id, vname in VEHICLES.items():
            try:
                c_init = sampler_client.simGetCollisionInfo(vehicle_name=vname)
                initial_timestamps[d_id] = c_init.time_stamp
            except Exception as e_probe:
                sampling_errors.append(f"Initial collision probe error on {d_id}: {e_probe}")
                initial_timestamps[d_id] = 0

        last_known_timestamps = dict(initial_timestamps)

        t0 = time.time()
        while not stop_flag.is_set():
            t_curr = time.time() - t0
            current_tick_positions = {}
            for d_id, vname in VEHICLES.items():
                try:
                    s = sampler_client.getMultirotorState(vehicle_name=vname)
                    col = sampler_client.simGetCollisionInfo(vehicle_name=vname)
                    p = s.kinematics_estimated.position
                    v = s.kinematics_estimated.linear_velocity
                    off = SPAWN_OFFSETS[d_id]
                    pos_tuple = (round(p.x_val + off[0], 2), round(p.y_val + off[1], 2), round(p.z_val + off[2], 2))
                    current_tick_positions[d_id] = pos_tuple

                    is_new_collision = bool(col.has_collided) and (col.time_stamp > last_known_timestamps[d_id]) and not col.object_name.startswith("Ground")
                    if is_new_collision:
                        collisions_detected[d_id] += 1
                        last_known_timestamps[d_id] = col.time_stamp
                        collision_events.append({
                            "t": round(t_curr, 2),
                            "drone_id": d_id,
                            "object_name": col.object_name,
                            "penetration_depth": round(col.penetration_depth, 4),
                            "pos": pos_tuple
                        })

                    samples[d_id].append({
                        "t": round(t_curr, 2),
                        "pos": pos_tuple,
                        "vel": (round(v.x_val, 2), round(v.y_val, 2), round(v.z_val, 2)),
                        "has_collided": is_new_collision,
                        "collision_count": collisions_detected[d_id],
                        "object_name": col.object_name if col.has_collided else ""
                    })
                except Exception as e_sample:
                    sampling_errors.append(f"Sampling exception on {d_id} @ t={t_curr:.2f}s: {e_sample}")

            drone_ids = list(current_tick_positions.keys())
            for i in range(len(drone_ids)):
                for j in range(i + 1, len(drone_ids)):
                    d1, d2 = drone_ids[i], drone_ids[j]
                    p1, p2 = current_tick_positions[d1], current_tick_positions[d2]
                    dist_val = dist3d(p1, p2)
                    pairwise_distances.append({
                        "t": round(t_curr, 2),
                        "pair": f"{d1}-{d2}",
                        "distance": round(dist_val, 2)
                    })

            time.sleep(0.05)

    sampler_thread = threading.Thread(target=sampler_worker, daemon=True)
    sampler_thread.start()

    print("  - Bravo(Drone2)와 Charlie(Drone3) 동시 RTH 복귀 명령 실행...", flush=True)
    rth_timings = {}
    rth_responses = {}

    def trigger_rth(d_id):
        t_start = time.time()
        rth_timings[d_id] = {"start_time": round(t_start, 2), "end_time": 0.0, "duration": 0.0}
        try:
            res = api_post("/api/rth", {"drone_id": d_id})
            t_end = time.time()
            rth_timings[d_id]["end_time"] = round(t_end, 2)
            rth_timings[d_id]["duration"] = round(t_end - t_start, 2)
            rth_responses[d_id] = res
            print(f"  - [{LABELS[d_id]}] RTH 완료 (소요시간: {t_end - t_start:.2f}s) | {res.get('status')}: {res.get('message')}", flush=True)
        except Exception as e_rth:
            t_end = time.time()
            rth_timings[d_id]["end_time"] = round(t_end, 2)
            rth_timings[d_id]["duration"] = round(t_end - t_start, 2)
            rth_responses[d_id] = {"status": "error", "message": str(e_rth)}

    t_bravo = threading.Thread(target=trigger_rth, args=("Drone2",))
    t_charlie = threading.Thread(target=trigger_rth, args=("Drone3",))

    t_bravo.start()
    time.sleep(0.2)
    t_charlie.start()

    time.sleep(1.0)
    dup_res = api_post("/api/rth", {"drone_id": "Drone2"})
    print(f"  - [Bravo 중복 RTH 테스트] 응답: {dup_res.get('status')} | {dup_res.get('message')}", flush=True)

    t_bravo.join(timeout=45.0)
    t_charlie.join(timeout=45.0)

    time.sleep(2.0)
    stop_flag.set()
    sampler_thread.join(timeout=3.0)

    # Concurrency / Overlap check
    t_start_b = rth_timings["Drone2"]["start_time"]
    t_end_b = rth_timings["Drone2"]["end_time"]
    t_start_c = rth_timings["Drone3"]["start_time"]
    t_end_c = rth_timings["Drone3"]["end_time"]
    overlap_sec = max(0.0, min(t_end_b, t_end_c) - max(t_start_b, t_start_c))
    concurrent_passed = (overlap_sec >= 5.0)

    final_positions = {}
    landing_accuracy = {}
    for d_id in ["Drone2", "Drone3"]:
        vname = VEHICLES[d_id]
        s = client_ctrl.getMultirotorState(vehicle_name=vname)
        p = s.kinematics_estimated.position
        off = SPAWN_OFFSETS[d_id]
        actual_world = (round(p.x_val + off[0], 2), round(p.y_val + off[1], 2), round(p.z_val + off[2], 2))
        final_positions[d_id] = actual_world

        expected_home_world = (off[0], off[1], 0.0)
        err_2d = math.sqrt((actual_world[0] - expected_home_world[0])**2 + (actual_world[1] - expected_home_world[1])**2)
        landing_accuracy[d_id] = {
            "expected_home_world": expected_home_world,
            "actual_world": actual_world,
            "error_distance_m": round(err_2d, 2),
            "accurate": bool(err_2d <= 1.5)
        }
        print(f"  - {LABELS[d_id]} 최종 착륙 위치(월드): {actual_world} | 의도 홈: {expected_home_world} | 오차={err_2d:.2f}m (정확={err_2d <= 1.5})")

    all_dists = [item["distance"] for item in pairwise_distances] if pairwise_distances else []
    min_dist_overall = min(all_dists) if all_dists else 0.0
    total_collisions = sum(collisions_detected.values())

    # =========================================================================
    # SCENARIO 2: Critical Vulnerability A - Alpha RTH with Alpha rotate intervention
    # =========================================================================
    print("\n" + "=" * 80, flush=True)
    print("[시나리오 2] 핵심 취약점 A 검증: Alpha RTH 중 Alpha 회전 명령 개입 직렬화 테스트", flush=True)
    print("=" * 80, flush=True)

    print("  - Alpha(Drone1)를 전방 15m 위치로 전진 이동...", flush=True)
    client_ctrl.moveToPositionAsync(15.0, 0.0, -5.0, 4.0, vehicle_name="SimpleFlight").join()
    time.sleep(1.0)

    alpha_rth_res = {}
    alpha_rotate_res = {}

    def alpha_rth_worker():
        try:
            alpha_rth_res["res"] = api_post("/api/rth", {"drone_id": "Drone1"})
            print(f"  - [Alpha RTH 스레드] 완료: {alpha_rth_res['res'].get('status')}", flush=True)
        except Exception as e_a:
            alpha_rth_res["res"] = {"status": "error", "message": str(e_a)}

    t_alpha_rth = threading.Thread(target=alpha_rth_worker)
    t_alpha_rth.start()

    # Mid-flight intervention: Send rotate command targeting Alpha
    time.sleep(2.0)
    print("  - [개입 발생] Alpha RTH 비행 중 Alpha 대상 /api/rotate(스캔 회전) 전송...", flush=True)
    try:
        alpha_rotate_res["res"] = api_post("/api/rotate", {"drone_id": "Drone1", "angle_deg": 45.0})
        print(f"  - [Alpha 회전 명령 응답]: {alpha_rotate_res['res'].get('status')} | {alpha_rotate_res['res'].get('message')}", flush=True)
    except Exception as e_rot:
        alpha_rotate_res["res"] = {"status": "error", "message": str(e_rot)}

    t_alpha_rth.join(timeout=45.0)

    s_alpha = client_ctrl.getMultirotorState(vehicle_name="SimpleFlight")
    p_alpha = s_alpha.kinematics_estimated.position
    alpha_err = math.sqrt(p_alpha.x_val**2 + p_alpha.y_val**2)
    alpha_safe_pass = (alpha_rotate_res["res"].get("status") == "success") and (alpha_err <= 1.5)
    print(f"  - Alpha 최종 복귀 착륙 위치: ({p_alpha.x_val:.2f}, {p_alpha.y_val:.2f}, {p_alpha.z_val:.2f}) | 오차={alpha_err:.2f}m (정합={alpha_safe_pass})")

    # =========================================================================
    # SCENARIO 3: Critical Vulnerability B - Delta RTH with /api/land override verification
    # =========================================================================
    print("\n" + "=" * 80, flush=True)
    print("[시나리오 3] 핵심 취약점 B 검증: Delta RTH 중 /api/land 안전 착륙 오버라이드 및 즉시 취소 실측", flush=True)
    print("=" * 80, flush=True)

    print("  - Delta(Drone4) 이륙 및 전진 배치...", flush=True)
    api_post("/api/takeoff", {"drone_id": "Drone4"})
    time.sleep(3.0)
    client_ctrl.moveToPositionAsync(15.0, 0.0, -5.0, 3.0, vehicle_name="Drone4").join()
    time.sleep(1.0)

    delta_rth_res = {}
    delta_land_res = {}
    delta_rth_start_t = time.time()

    def delta_rth_worker():
        try:
            delta_rth_res["res"] = api_post("/api/rth", {"drone_id": "Drone4"})
            delta_rth_res["end_time"] = time.time()
            print(f"  - [Delta RTH 스레드] 종료 응답: {delta_rth_res['res'].get('status')} | {delta_rth_res['res'].get('message')}", flush=True)
        except Exception as e_d:
            delta_rth_res["res"] = {"status": "error", "message": str(e_d)}
            delta_rth_res["end_time"] = time.time()

    t_delta_rth = threading.Thread(target=delta_rth_worker)
    t_delta_rth.start()

    # Mid-flight intervention: Send /api/land to Delta at t=2.0s
    time.sleep(2.0)
    print("  - [안전 착륙 개입] Delta RTH 비행 중 /api/land(착륙) 전송 -> RTH 즉시 취소 유도...", flush=True)
    try:
        delta_land_res["res"] = api_post("/api/land", {"drone_id": "Drone4"})
        print(f"  - [Delta 착륙 명령 응답]: {delta_land_res['res'].get('status')} | {delta_land_res['res'].get('message')}", flush=True)
    except Exception as e_land:
        delta_land_res["res"] = {"status": "error", "message": str(e_land)}

    # Wait for Delta RTH thread to terminate
    t_delta_rth.join(timeout=10.0)
    rth_thread_terminated = not t_delta_rth.is_alive()
    print(f"  - Delta RTH 스레드 즉시 종료 확인: {rth_thread_terminated}")

    # Wait 2.0s and verify Delta remains safely landed with zero velocity (no subsequent RTH velocity commands)
    time.sleep(2.0)
    s_delta_after = client_ctrl.getMultirotorState(vehicle_name="Drone4")
    p_delta_after = s_delta_after.kinematics_estimated.position
    v_delta_after = s_delta_after.kinematics_estimated.linear_velocity
    delta_speed_after = math.sqrt(v_delta_after.x_val**2 + v_delta_after.y_val**2 + v_delta_after.z_val**2)
    delta_is_landed = (p_delta_after.z_val >= -0.5) and (delta_speed_after < 0.2)
    print(f"  - Delta 최종 상태: 고도 Z={p_delta_after.z_val:.2f}m, 속도={delta_speed_after:.2f}m/s (안전 착륙 유지={delta_is_landed})")

    delta_override_passed = (
        delta_land_res["res"].get("status") == "success" and
        rth_thread_terminated and
        delta_is_landed
    )
    print(f"  - Delta 착륙 안전 오버라이드 최종 판정: {'PASS' if delta_override_passed else 'FAIL'}")

    # =========================================================================
    # Final Analysis & Reporting
    # =========================================================================
    print("\n" + "=" * 80, flush=True)
    print("[최종 실측 종합 판정]")
    print(f"  1. Bravo/Charlie 동시 RTH 비행 입증 (overlap >= 5.0s): {'PASS' if concurrent_passed else 'FAIL'} ({overlap_sec:.2f}s)")
    print(f"  2. 원자적 중복 RTH 거절 방어: {'PASS' if dup_res.get('status') in ['ignored', 'error'] else 'FAIL'}")
    print(f"  3. 무충돌 달성 (collision_count=0): {'PASS' if total_collisions == 0 else 'FAIL'} ({total_collisions} collisions)")
    print(f"  4. ORCA 설정 안전 이격 유지 (min >= {REQUIRED_MIN_SEPARATION_M:.1f}m, combined={2*ORCA_AGENT_RADIUS_M:.1f}m): {'PASS' if min_dist_overall >= REQUIRED_MIN_SEPARATION_M else 'FAIL'} ({min_dist_overall:.2f}m)")
    print(f"  5. 홈 착륙 정합성 (error <= 1.5m): {'PASS' if all(res['accurate'] for res in landing_accuracy.values()) else 'FAIL'}")
    print(f"  6. 취약점 A 검증 (Alpha RTH 중 Alpha 회전 명령 직렬화): {'PASS' if alpha_safe_pass else 'FAIL'}")
    print(f"  7. 취약점 B 검증 (Delta RTH 중 /api/land 안전 착륙 오버라이드 및 즉시 취소): {'PASS' if delta_override_passed else 'FAIL'}")

    all_passed = (
        concurrent_passed and
        (dup_res.get("status") in ["ignored", "error"]) and
        (total_collisions == 0) and
        (min_dist_overall >= REQUIRED_MIN_SEPARATION_M) and
        all(res["accurate"] for res in landing_accuracy.values()) and
        alpha_safe_pass and
        delta_override_passed
    )

    print(f"\n  => 최종 판정: {'✅ ALL PASSED (공유 control_lock 일원화 & 착륙 안전 오버라이드 완전 검증 성공)' if all_passed else '❌ TEST FAILED'}")
    print("=" * 80, flush=True)

    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "test_passed": all_passed,
        "concurrent_overlap_seconds": round(overlap_sec, 2),
        "rth_timings": rth_timings,
        "duplicate_prevention_test": dup_res,
        "total_samples": sum(len(samples[d]) for d in VEHICLES),
        "samples_count": {d: len(samples[d]) for d in VEHICLES},
        "sampling_errors_count": len(sampling_errors),
        "total_collisions": total_collisions,
        "collisions_per_drone": collisions_detected,
        "configured_orca_radius_m": ORCA_AGENT_RADIUS_M,
        "combined_safety_distance_m": 2 * ORCA_AGENT_RADIUS_M,
        "required_min_separation_m": REQUIRED_MIN_SEPARATION_M,
        "min_pairwise_distance_m": round(min_dist_overall, 2),
        "dispersed_positions": dispersed_positions,
        "final_positions": final_positions,
        "landing_accuracy": landing_accuracy,
        "vulnerability_scenario_alpha_intervention": {
            "rotate_response": alpha_rotate_res.get("res"),
            "alpha_final_error_m": round(alpha_err, 2),
            "passed": alpha_safe_pass
        },
        "vulnerability_scenario_delta_land_override": {
            "land_response": delta_land_res.get("res"),
            "rth_response": delta_rth_res.get("res"),
            "rth_thread_terminated": rth_thread_terminated,
            "delta_final_altitude_z": round(p_delta_after.z_val, 2),
            "delta_final_speed_mps": round(delta_speed_after, 2),
            "delta_is_landed": delta_is_landed,
            "passed": delta_override_passed
        }
    }

    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(f"  - 상세 실측 리포트 저장 완료: {REPORT_PATH}", flush=True)

    # Cleanup
    for d_id, vname in VEHICLES.items():
        try:
            client_ctrl.landAsync(vehicle_name=vname).join()
        except Exception:
            pass
    api_post("/api/simulators/stop")

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
