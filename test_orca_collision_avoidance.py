"""
ORCA Collision Avoidance & Following Mode Stress Verification Test.

Enhancements for independent verification:
1. Dedicated thread-safe airsim.MultirotorClient for telemetry & collision sampler.
2. Explicit sampling error tracking (no silent swallow); enforces sampling_errors == 0.
3. Minimum sample count threshold (samples_count >= 40 per drone).
4. Real-time pairwise minimum distance calculation across all 4 UAVs.
5. Leader-follower directional alignment verification (dot product with Alpha displacement > 0).
6. Comprehensive report generation with time-series kinematics and collision counters.

Usage:
    python server.py
    python test_orca_collision_avoidance.py
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
REPORT_PATH = "orca_collision_avoidance_report.json"
VEHICLES = {"Drone1": "SimpleFlight", "Drone2": "Drone2", "Drone3": "Drone3", "Drone4": "Drone4"}
LABELS = {"Drone1": "Alpha", "Drone2": "Bravo", "Drone3": "Charlie", "Drone4": "Delta"}
SPAWN_OFFSETS = {"Drone1": (0.0, 0.0, 0.0), "Drone2": (0.0, 3.5, 0.0), "Drone3": (0.0, 7.0, 0.0), "Drone4": (0.0, 10.5, 0.0)}
ORCA_AGENT_RADIUS_M = 1.5  # Combined safety separation requirement = 2 * 1.5 = 3.0m


def api_post(endpoint: str, payload: dict = None) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    data = json.dumps(payload or {}).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=30) as res:
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


def dist2d(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def dist3d(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def main():
    print("=" * 80, flush=True)
    print("[ORCA 충돌 회피 & Following Mode 정밀 스트레스 실측 테스트]", flush=True)
    print("=" * 80, flush=True)

    # 1. Simulator Launch
    print("\n[1] Blocks 시뮬레이터 실행...", flush=True)
    res = api_post("/api/simulators/launch", {"id": "blocks", "resolution": "1280x720"})
    print(f"  - {res.get('status')}: {res.get('message')}", flush=True)
    assert wait_for_port(41451, 60.0), "AirSim RPC 포트 오픈 대기 타임아웃"
    time.sleep(2.5)

    # Main control client
    client_ctrl = None
    for attempt in range(10):
        try:
            c = airsim.MultirotorClient(timeout_value=5)
            c.confirmConnection()
            client_ctrl = c
            break
        except Exception as e:
            print(f"  - 메인 클라이언트 연결 재시도 {attempt + 1}/10: {e}", flush=True)
            time.sleep(1.0)
    assert client_ctrl is not None, "AirSim 메인 제어 클라이언트 연결 실패"
    print(f"  - 감지된 기체: {client_ctrl.listVehicles()}", flush=True)

    # 2. Bulk Takeoff
    print("\n[2] 전체 편대 동시 이륙...", flush=True)
    api_post("/api/fleet/takeoff")
    time.sleep(4.0)

    # 3. Formation Assemble
    print("\n[3] 편대 집결 (알파 호출) - 4대 편대 정렬...", flush=True)
    res = api_post("/api/formation/assemble", {"spacing": 8.0, "velocity": 4.0})
    print(f"  - {res.get('status')}: {res.get('message')}", flush=True)
    time.sleep(6.0)

    positions_before = {}
    for d_id, vname in VEHICLES.items():
        s = client_ctrl.getMultirotorState(vehicle_name=vname)
        p = s.kinematics_estimated.position
        off = SPAWN_OFFSETS[d_id]
        positions_before[d_id] = (round(p.x_val + off[0], 2), round(p.y_val + off[1], 2), round(p.z_val + off[2], 2))
        print(f"  - {LABELS[d_id]}({vname}) 집결 후 위치: ({positions_before[d_id][0]:.1f}, {positions_before[d_id][1]:.1f}, {positions_before[d_id][2]:.1f})", flush=True)

    # 4. Activate Following Mode
    print("\n[4] Following Mode 활성화 (스트레스 조건: lag=1.2s 단축, velocity=3.5m/s)...", flush=True)
    res = api_post("/api/following/toggle", {"enabled": True, "lag_seconds": 1.2, "velocity": 3.5})
    print(f"  - {res.get('status')}: {res.get('message')}", flush=True)
    time.sleep(1.0)

    # 5. Dedicated Sampler Thread with Independent Client
    print("\n[5] 전용 독립 RPC 클라이언트를 통한 고빈도(20Hz) 샘플러 스레드 가동...", flush=True)
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

        # Record initial ground collision timestamps before following flight
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

                    # Detect new in-flight collision event
                    is_new_collision = bool(col.has_collided) and (col.time_stamp > last_known_timestamps[d_id])
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

            # Measure pairwise minimum distances at this tick
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

            time.sleep(0.05)  # 20Hz sampling rate

    sampler_thread = threading.Thread(target=sampler_worker, daemon=True)
    sampler_thread.start()

    # Dynamic Maneuvers
    # Phase A: Forward dash + right turn
    print("  -> Phase A: 전진 + 우측 급선회 기동 (4초)...", flush=True)
    api_post("/api/joystick", {"drone_id": "Drone1", "vx": 3.0, "vy": 1.5, "vz": 0, "yaw_rate": 30.0, "duration": 4.0})
    time.sleep(4.2)

    # Phase B: Rapid counter-turn (left S-curve)
    print("  -> Phase B: 급격한 반대 방향 좌측 S-Curve 선회 기동 (4초)...", flush=True)
    api_post("/api/joystick", {"drone_id": "Drone1", "vx": 3.0, "vy": -2.0, "vz": 0, "yaw_rate": -45.0, "duration": 4.0})
    time.sleep(4.2)

    # Phase C: Sudden reverse brake & climb
    print("  -> Phase C: 급정지 및 감속 호버링 (3초)...", flush=True)
    api_post("/api/joystick", {"drone_id": "Drone1", "vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0, "duration": 3.0})
    time.sleep(3.5)

    stop_flag.set()
    sampler_thread.join(timeout=3.0)

    # 6. Post-Flight Measurements
    positions_after = {}
    for d_id, vname in VEHICLES.items():
        s = client_ctrl.getMultirotorState(vehicle_name=vname)
        p = s.kinematics_estimated.position
        off = SPAWN_OFFSETS[d_id]
        positions_after[d_id] = (round(p.x_val + off[0], 2), round(p.y_val + off[1], 2), round(p.z_val + off[2], 2))

    print("\n" + "=" * 80, flush=True)
    print("[실측 결과 데이터 분석]", flush=True)

    # Sampling statistics
    total_samples = sum(len(samples[d]) for d in VEHICLES)
    print(f"  - 총 수집 샘플 수: {total_samples} 개")
    for d_id in VEHICLES:
        print(f"    * {LABELS[d_id]}({d_id}): {len(samples[d_id])} 개 샘플 수집")
    print(f"  - 샘플링 예외 발생 수: {len(sampling_errors)} 건")

    # Collision analysis
    total_collisions = sum(collisions_detected.values())
    for d_id in VEHICLES:
        print(f"  - {LABELS[d_id]} 충돌 횟수: {collisions_detected[d_id]} 회")

    # Separation analysis
    all_dists = [item["distance"] for item in pairwise_distances] if pairwise_distances else []
    min_dist_overall = min(all_dists) if all_dists else 0.0
    print(f"  - 비행 중 기체 간 최소 이격 거리: {min_dist_overall:.2f}m")

    # Displacement and direction alignment
    alpha_dx = positions_after["Drone1"][0] - positions_before["Drone1"][0]
    alpha_dy = positions_after["Drone1"][1] - positions_before["Drone1"][1]
    alpha_disp = math.sqrt(alpha_dx**2 + alpha_dy**2)
    print(f"  - Alpha 이동 거리: {alpha_disp:.2f}m (dx={alpha_dx:.2f}, dy={alpha_dy:.2f})")

    follower_results = {}
    for d_id in ["Drone2", "Drone3", "Drone4"]:
        dx = positions_after[d_id][0] - positions_before[d_id][0]
        dy = positions_after[d_id][1] - positions_before[d_id][1]
        disp = math.sqrt(dx**2 + dy**2)
        # Dot product with alpha's displacement direction
        dir_alignment = (dx * alpha_dx + dy * alpha_dy) / max(0.01, alpha_disp) if alpha_disp > 0.5 else 0.0
        same_dir = dir_alignment > 0
        follower_results[d_id] = {
            "displacement": round(disp, 2),
            "dx": round(dx, 2),
            "dy": round(dy, 2),
            "same_direction": same_dir,
            "alignment_score": round(dir_alignment, 2)
        }
        print(f"  - {LABELS[d_id]} 이동 거리: {disp:.2f}m (dx={dx:.2f}, dy={dy:.2f}) | 방향 정합성={same_dir}")

    # Robust Success Criteria:
    # 1. Samples collected >= 40 per drone
    sufficient_samples = all(len(samples[d]) >= 40 for d in VEHICLES)
    # 2. No sampling errors
    no_sampling_errors = (len(sampling_errors) == 0)
    # 3. No collisions
    no_collisions = (total_collisions == 0)
    # 4. Minimum separation distance maintained above configured ORCA safe distance (2 * radius = 3.0m)
    required_separation = 2 * ORCA_AGENT_RADIUS_M
    safe_separation = (min_dist_overall >= required_separation)
    # 5. All followers moved in the same general direction with displacement > 2.0m
    followers_tracked = all(res["displacement"] > 2.0 and res["same_direction"] for res in follower_results.values())

    test_passed = (
        sufficient_samples and
        no_sampling_errors and
        no_collisions and
        safe_separation and
        followers_tracked
    )

    print("\n" + "=" * 80, flush=True)
    print("[최종 판정 지표]")
    print(f"  1. 샘플 수집 충분성 (>=40): {'PASS' if sufficient_samples else 'FAIL'} ({[len(samples[d]) for d in VEHICLES]})")
    print(f"  2. 샘플링 에러 0건: {'PASS' if no_sampling_errors else 'FAIL'} ({len(sampling_errors)} errors)")
    print(f"  3. 무충돌 달성 (collision_count=0): {'PASS' if no_collisions else 'FAIL'} ({total_collisions} collisions)")
    print(f"  4. ORCA 설정 안전 이격 유지 (min >= {required_separation:.1f}m): {'PASS' if safe_separation else 'FAIL'} ({min_dist_overall:.2f}m)")
    print(f"  5. 팔로워 순차 추격 정합성: {'PASS' if followers_tracked else 'FAIL'}")
    print(f"\n  => 최종 테스트 판정: {'✅ ALL PASSED (ORCA 충돌 회피 실측 성공)' if test_passed else '❌ TEST FAILED'}")
    print("=" * 80, flush=True)

    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "test_passed": test_passed,
        "total_samples": total_samples,
        "samples_count": {d: len(samples[d]) for d in VEHICLES},
        "sampling_errors_count": len(sampling_errors),
        "sampling_errors": sampling_errors[:10],
        "total_collisions": total_collisions,
        "collisions_per_drone": collisions_detected,
        "collision_events": collision_events,
        "configured_orca_radius_m": ORCA_AGENT_RADIUS_M,
        "required_separation_m": required_separation,
        "min_pairwise_distance_m": round(min_dist_overall, 2),
        "alpha_displacement_m": round(alpha_disp, 2),
        "follower_results": follower_results,
        "positions_before": positions_before,
        "positions_after": positions_after,
        "sample_series_summary": {
            d: {
                "first_sample": samples[d][0] if samples[d] else None,
                "mid_sample": samples[d][len(samples[d]) // 2] if samples[d] else None,
                "last_sample": samples[d][-1] if samples[d] else None
            }
            for d in VEHICLES
        }
    }

    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(f"  - 상세 실측 리포트 저장 완료: {REPORT_PATH}", flush=True)

    # Cleanup
    api_post("/api/following/toggle", {"enabled": False})
    time.sleep(0.5)
    for d_id, vname in VEHICLES.items():
        try:
            client_ctrl.landAsync(vehicle_name=vname).join()
        except Exception:
            pass
    api_post("/api/simulators/stop")

    sys.exit(0 if test_passed else 1)


if __name__ == "__main__":
    main()
