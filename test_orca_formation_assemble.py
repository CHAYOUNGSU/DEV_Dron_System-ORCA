"""
ORCA Collision Avoidance Verification for Formation Assemble (_do_formation_assemble).

Test Scenario:
1. Take off all 4 drones in Blocks simulator.
2. Scatter wingmen (Bravo, Charlie, Delta) in opposing quadrants:
   - Drone2 (Bravo): Front-Right (+15m, +10m)
   - Drone3 (Charlie): Front-Left (+15m, -10m)
   - Drone4 (Delta): Rear-Left (-10m, -10m)
   - Alpha stays at origin (0, 0)
3. Trigger Formation Assemble (Alpha calls all wingmen to trail slots behind it).
   Wingmen must converge through crossing trajectories without collisions.
4. Independent 20Hz Sampler verifies:
   - No sampling errors (sampling_errors == 0)
   - Sufficient samples (samples_count >= 40 per drone)
   - Zero collisions (total_collisions == 0)
   - Minimum separation distance >= 2 * ORCA_AGENT_RADIUS_M (3.0m)
   - Final trail slot alignment error <= 1.5m for all wingmen.

Usage:
    python server.py
    python test_orca_formation_assemble.py
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
REPORT_PATH = "orca_formation_assemble_report.json"
VEHICLES = {"Drone1": "SimpleFlight", "Drone2": "Drone2", "Drone3": "Drone3", "Drone4": "Drone4"}
LABELS = {"Drone1": "Alpha", "Drone2": "Bravo", "Drone3": "Charlie", "Drone4": "Delta"}
SPAWN_OFFSETS = {"Drone1": (0.0, 0.0, 0.0), "Drone2": (0.0, 3.5, 0.0), "Drone3": (0.0, 7.0, 0.0), "Drone4": (0.0, 10.5, 0.0)}
ORCA_AGENT_RADIUS_M = 1.5  # Combined safety separation threshold = 2 * 1.5 = 3.0m


def api_post(endpoint: str, payload: dict = None) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    data = json.dumps(payload or {}).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=35) as res:
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
    print("[ORCA 편대 집결(Formation Assemble) 크로스오버 충돌 회피 실측 테스트]", flush=True)
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

    # 3. Force Scatter Wingmen into Dispersed Quadrants (Cross-over trajectory setup)
    print("\n[3] 윙맨 사방 분산 배치 (교차 충돌 위험 궤적 사전 형성)...", flush=True)
    scatter_targets = {
        "Drone2": (15.0, 10.0, -4.0),   # Front-Right (world)
        "Drone3": (15.0, -10.0, -4.0),  # Front-Left (world)
        "Drone4": (-10.0, -10.0, -4.0)  # Rear-Left (world)
    }

    # Dispatch scatter moves (convert world target to local coordinate by subtracting spawn offset)
    scatter_futures = []
    for d_id, (wx, wy, wz) in scatter_targets.items():
        vname = VEHICLES[d_id]
        off = SPAWN_OFFSETS[d_id]
        local_x = wx - off[0]
        local_y = wy - off[1]
        local_z = wz - off[2]
        print(f"  - {LABELS[d_id]} 분산 이동: 월드 ({wx:.1f}, {wy:.1f}, {wz:.1f}) -> 로컬 ({local_x:.1f}, {local_y:.1f}, {local_z:.1f})", flush=True)
        scatter_futures.append(client_ctrl.moveToPositionAsync(local_x, local_y, local_z, 4.0, vehicle_name=vname))

    for f in scatter_futures:
        try:
            f.join()
        except Exception:
            pass
    time.sleep(1.0)

    # Print scattered positions
    scattered_positions = {}
    for d_id, vname in VEHICLES.items():
        s = client_ctrl.getMultirotorState(vehicle_name=vname)
        p = s.kinematics_estimated.position
        off = SPAWN_OFFSETS[d_id]
        scattered_positions[d_id] = (round(p.x_val + off[0], 2), round(p.y_val + off[1], 2), round(p.z_val + off[2], 2))
        print(f"  - {LABELS[d_id]} 분산 완료 위치(월드): {scattered_positions[d_id]}", flush=True)

    # 4. Start Dedicated 20Hz Telemetry & Collision Sampler
    print("\n[4] 전용 독립 RPC 클라이언트를 통한 고빈도(20Hz) 샘플러 스레드 가동...", flush=True)
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

    # 5. Trigger Formation Assemble with ORCA (Spacing = 8.0m, Velocity = 3.5m/s)
    print("\n[5] 편대 집결 (알파 호출) 실행 - ORCA 충돌 회피 집결 비행 시작...", flush=True)
    assemble_spacing = 8.0
    assemble_velocity = 3.5
    res_assemble = api_post("/api/formation/assemble", {"spacing": assemble_spacing, "velocity": assemble_velocity})
    print(f"  - {res_assemble.get('status')}: {res_assemble.get('message')}", flush=True)

    # Allow convergence stabilization
    time.sleep(2.0)

    stop_flag.set()
    sampler_thread.join(timeout=3.0)

    # 6. Post-Flight Measurements & Final Trail Slot Alignment Check
    print("\n" + "=" * 80, flush=True)
    print("[실측 결과 데이터 분석]", flush=True)

    final_positions = {}
    for d_id, vname in VEHICLES.items():
        s = client_ctrl.getMultirotorState(vehicle_name=vname)
        p = s.kinematics_estimated.position
        off = SPAWN_OFFSETS[d_id]
        final_positions[d_id] = (round(p.x_val + off[0], 2), round(p.y_val + off[1], 2), round(p.z_val + off[2], 2))

    # Calculate expected trail slots behind Alpha
    s_alpha = client_ctrl.getMultirotorState(vehicle_name="SimpleFlight")
    _, _, alpha_yaw = airsim.to_eularian_angles(s_alpha.kinematics_estimated.orientation)
    alpha_wx, alpha_wy, alpha_wz = final_positions["Drone1"]
    back_dir_x = -math.cos(alpha_yaw)
    back_dir_y = -math.sin(alpha_yaw)

    expected_slots = {}
    slot_alignment_results = {}
    for i, w_id in enumerate(["Drone2", "Drone3", "Drone4"], start=1):
        expected_slots[w_id] = (
            round(alpha_wx + back_dir_x * (i * assemble_spacing), 2),
            round(alpha_wy + back_dir_y * (i * assemble_spacing), 2),
            round(alpha_wz, 2)
        )
        actual = final_positions[w_id]
        err = dist3d(actual, expected_slots[w_id])
        slot_alignment_results[w_id] = {
            "expected_slot": expected_slots[w_id],
            "actual_position": actual,
            "error_distance_m": round(err, 2),
            "aligned": bool(err <= 1.5)
        }
        print(f"  - {LABELS[w_id]} 집결 완료 위치: {actual} | 목표 슬롯: {expected_slots[w_id]} | 오차={err:.2f}m (정렬={err <= 1.5})")

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
    all_aligned = all(res["aligned"] for res in slot_alignment_results.values())

    test_passed = (
        sufficient_samples and
        no_sampling_errors and
        no_collisions and
        safe_separation and
        all_aligned
    )

    print("\n" + "=" * 80, flush=True)
    print("[최종 판정 지표]")
    print(f"  1. 샘플 수집 충분성 (>=40): {'PASS' if sufficient_samples else 'FAIL'} ({[len(samples[d]) for d in VEHICLES]})")
    print(f"  2. 샘플링 에러 0건: {'PASS' if no_sampling_errors else 'FAIL'} ({len(sampling_errors)} errors)")
    print(f"  3. 무충돌 달성 (collision_count=0): {'PASS' if no_collisions else 'FAIL'} ({total_collisions} collisions)")
    print(f"  4. ORCA 설정 안전 이격 유지 (min >= {required_separation:.1f}m): {'PASS' if safe_separation else 'FAIL'} ({min_dist_overall:.2f}m)")
    print(f"  5. 전 윙맨 트레일 슬롯 정렬 (error <= 1.5m): {'PASS' if all_aligned else 'FAIL'}")
    print(f"\n  => 최종 테스트 판정: {'✅ ALL PASSED (편대 집결 ORCA 충돌 회피 실측 성공)' if test_passed else '❌ TEST FAILED'}")
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
        "scattered_positions": scattered_positions,
        "final_positions": final_positions,
        "slot_alignment_results": slot_alignment_results,
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
    for d_id, vname in VEHICLES.items():
        try:
            client_ctrl.landAsync(vehicle_name=vname).join()
        except Exception:
            pass
    api_post("/api/simulators/stop")

    sys.exit(0 if test_passed else 1)


if __name__ == "__main__":
    main()
