"""
ORCA Collision Avoidance & World Coordinate Verification for Return To Home (_do_rth).

Test Scenario:
1. Take off all 4 drones in Blocks simulator.
2. Advance Bravo (Drone2) and Charlie (Drone3) into cross-return positions:
   - Drone2 (Bravo): Forward-Right (+20.0m, +10.0m)
   - Drone3 (Charlie): Forward-Left (+20.0m, -5.0m)
   - Alpha (Drone1) and Delta (Drone4) hover in place as obstacles.
3. Trigger simultaneous/parallel RTH for Drone2 and Drone3.
   Both drones must climb, cruise back to their respective own home points,
   descend, and land safely while maintaining ORCA safety separation.
4. Independent 20Hz Sampler verifies:
   - True concurrency / Overlap execution: Both drones fly concurrently (overlap >= 5.0s)
   - No sampling errors (sampling_errors == 0)
   - Sufficient samples (samples_count >= 40 per drone)
   - Zero collisions (total_collisions == 0)
   - Minimum separation distance >= 2 * ORCA_AGENT_RADIUS_M (3.0m)
   - Landing precision: Drone2 lands at spawn offset (0, 3.5), Drone3 at (0, 7.0) with error <= 1.5m.

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
ORCA_AGENT_RADIUS_M = 1.5  # Combined safety separation threshold = 2 * 1.5 = 3.0m


def api_post(endpoint: str, payload: dict = None) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    data = json.dumps(payload or {}).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=60) as res:
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


def main():
    print("=" * 80, flush=True)
    print("[ORCA RTH(자동 복귀) 좌표계 수정 & 진정한 동시 비행 충돌 회피 실측 테스트]", flush=True)
    print("=" * 80, flush=True)

    # 1. Simulator Launch
    print("\n[1] Blocks 시뮬레이터 실행...", flush=True)
    res = api_post("/api/simulators/launch", {"id": "blocks", "resolution": "1280x720"})
    print(f"  - {res.get('status')}: {res.get('message')}", flush=True)
    assert wait_for_port(41451, 60.0), "AirSim RPC 포트 오픈 대기 타임아웃"
    time.sleep(2.5)

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

    # 3. Advance Bravo and Charlie into Dispersed Positions
    print("\n[3] Bravo(Drone2)와 Charlie(Drone3) 전방 교차 위치로 전진 배치...", flush=True)
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

    # 4. Start Dedicated 20Hz Telemetry & Collision Sampler
    print("\n[4] 20Hz 고빈도 텔레메트리 & 충돌 샘플러 가동...", flush=True)
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

            time.sleep(0.05)  # 20Hz

    sampler_thread = threading.Thread(target=sampler_worker, daemon=True)
    sampler_thread.start()

    # 5. Trigger Truly Simultaneous RTH for Drone2 (Bravo) and Drone3 (Charlie)
    print("\n[5] Bravo(Drone2)와 Charlie(Drone3) 동시 RTH 복귀 명령 실행...", flush=True)
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
    time.sleep(0.3)  # Dispatch Bravo and Charlie essentially at the same time
    t_charlie.start()

    # Test atomic duplicate prevention while Bravo is running
    time.sleep(1.0)
    dup_res = api_post("/api/rth", {"drone_id": "Drone2"})
    print(f"  - [Bravo 중복 RTH 테스트] 응답: {dup_res.get('status')} | {dup_res.get('message')}", flush=True)

    t_bravo.join(timeout=45.0)
    t_charlie.join(timeout=45.0)

    time.sleep(2.0)
    stop_flag.set()
    sampler_thread.join(timeout=3.0)

    # 6. Post-Flight Measurements & Concurrency Analysis
    print("\n" + "=" * 80, flush=True)
    print("[실측 결과 데이터 분석]", flush=True)

    # Concurrency / Overlap check
    t_start_b = rth_timings["Drone2"]["start_time"]
    t_end_b = rth_timings["Drone2"]["end_time"]
    t_start_c = rth_timings["Drone3"]["start_time"]
    t_end_c = rth_timings["Drone3"]["end_time"]
    overlap_sec = max(0.0, min(t_end_b, t_end_c) - max(t_start_b, t_start_c))
    concurrent_passed = (overlap_sec >= 5.0)
    print(f"  - Bravo 비행 구간: {t_start_b:.2f} ~ {t_end_b:.2f} ({rth_timings['Drone2']['duration']}s)")
    print(f"  - Charlie 비행 구간: {t_start_c:.2f} ~ {t_end_c:.2f} ({rth_timings['Drone3']['duration']}s)")
    print(f"  - 동시 병렬 비행 중첩 시간: {overlap_sec:.2f}초 (기준 >= 5.0초, 병렬 실행={concurrent_passed})")

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

    # Sampling statistics
    total_samples = sum(len(samples[d]) for d in VEHICLES)
    print(f"\n  - 총 수집 샘플 수: {total_samples} 개")
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
    required_separation = 2 * ORCA_AGENT_RADIUS_M
    print(f"  - 비행 중 기체 간 최소 이격 거리: {min_dist_overall:.2f}m (기준 >= {required_separation:.1f}m)")

    # Robust Success Criteria
    sufficient_samples = all(len(samples[d]) >= 40 for d in VEHICLES)
    no_sampling_errors = (len(sampling_errors) == 0)
    no_collisions = (total_collisions == 0)
    safe_separation = (min_dist_overall >= required_separation)
    all_accurate = all(res["accurate"] for res in landing_accuracy.values())
    duplicate_blocked = (dup_res.get("status") in ["ignored", "error"])

    test_passed = (
        concurrent_passed and
        sufficient_samples and
        no_sampling_errors and
        no_collisions and
        safe_separation and
        all_accurate and
        duplicate_blocked
    )

    print("\n" + "=" * 80, flush=True)
    print("[최종 판정 지표]")
    print(f"  1. 진정한 동시 병렬 RTH 비행 입증 (overlap >= 5.0s): {'PASS' if concurrent_passed else 'FAIL'} ({overlap_sec:.2f}s)")
    print(f"  2. 원자적 중복 RTH 거절 방어: {'PASS' if duplicate_blocked else 'FAIL'} ({dup_res.get('status')})")
    print(f"  3. 샘플 수집 충분성 (>=40): {'PASS' if sufficient_samples else 'FAIL'} ({[len(samples[d]) for d in VEHICLES]})")
    print(f"  4. 샘플링 에러 0건: {'PASS' if no_sampling_errors else 'FAIL'} ({len(sampling_errors)} errors)")
    print(f"  5. 무충돌 달성 (collision_count=0): {'PASS' if no_collisions else 'FAIL'} ({total_collisions} collisions)")
    print(f"  6. ORCA 설정 안전 이격 유지 (min >= {required_separation:.1f}m): {'PASS' if safe_separation else 'FAIL'} ({min_dist_overall:.2f}m)")
    print(f"  7. 홈 착륙 정합성 (error <= 1.5m): {'PASS' if all_accurate else 'FAIL'}")
    print(f"\n  => 최종 테스트 판정: {'✅ ALL PASSED (진정한 다중 기체 동시 RTH ORCA 충돌 회피 실측 성공)' if test_passed else '❌ TEST FAILED'}")
    print("=" * 80, flush=True)

    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "test_passed": test_passed,
        "concurrent_overlap_seconds": round(overlap_sec, 2),
        "rth_timings": rth_timings,
        "duplicate_prevention_test": dup_res,
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
        "dispersed_positions": dispersed_positions,
        "final_positions": final_positions,
        "landing_accuracy": landing_accuracy
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

    sys.exit(0 if test_passed else 1)


if __name__ == "__main__":
    main()
