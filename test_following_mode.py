"""
Diagnostic/regression test for Following Mode (duckling chain autopilot).

Flow: launch Blocks -> Alpha takes off -> Formation Assemble (spreads Bravo/
Charlie/Delta into trail spacing behind Alpha) -> enable Following Mode ->
fly Alpha forward for several seconds -> verify Bravo/Charlie/Delta actually
moved in the same general direction, tracing Alpha's earlier path, instead of
staying put or moving randomly.

Requires python server.py running with Blocks available.
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
REPORT_PATH = "following_mode_report.json"
VEHICLES = {"Drone1": "SimpleFlight", "Drone2": "Drone2", "Drone3": "Drone3", "Drone4": "Drone4"}
LABELS = {"Drone1": "Alpha", "Drone2": "Bravo", "Drone3": "Charlie", "Drone4": "Delta"}


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


def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def main():
    print("=" * 80, flush=True)
    print("[Following Mode 검증 테스트]", flush=True)
    print("=" * 80, flush=True)

    print("\n[1] Blocks 시뮬레이터 실행...", flush=True)
    res = api_post("/api/simulators/launch", {"id": "blocks", "resolution": "1280x720"})
    print(f"  - {res.get('status')}: {res.get('message')}", flush=True)
    assert wait_for_port(41451, 60.0), "AirSim RPC 포트 오픈 대기 타임아웃"
    time.sleep(2.0)

    client = None
    for attempt in range(10):
        try:
            c = airsim.MultirotorClient()
            c.confirmConnection()
            client = c
            break
        except Exception as e:
            print(f"  - 연결 재시도 {attempt + 1}/10: {e}", flush=True)
            time.sleep(1.0)
    assert client is not None, "AirSim 클라이언트 연결 실패"
    print(f"  - 감지된 차량: {client.listVehicles()}", flush=True)

    print("\n[2] Alpha 이륙...", flush=True)
    api_post("/api/takeoff", {"drone_id": "Drone1"})
    time.sleep(3.0)

    print("\n[3] 편대 집결 (알파 호출) - 브라보/찰리/델타를 대형으로 정렬...", flush=True)
    res = api_post("/api/formation/assemble", {"spacing": 8.0, "velocity": 4.0})
    print(f"  - {res.get('status')}: {res.get('message')}", flush=True)
    time.sleep(6.0)

    positions_before = {}
    for d_id, vname in VEHICLES.items():
        s = client.getMultirotorState(vehicle_name=vname)
        p = s.kinematics_estimated.position
        positions_before[d_id] = (p.x_val, p.y_val, p.z_val)
        print(f"  - {LABELS[d_id]}({vname}) 집결 후 위치: ({p.x_val:.1f}, {p.y_val:.1f}, {p.z_val:.1f})", flush=True)

    print("\n[4] Following Mode 활성화...", flush=True)
    res = api_post("/api/following/toggle", {"enabled": True, "lag_seconds": 1.5, "velocity": 3.5})
    print(f"  - {res.get('status')}: {res.get('message')}", flush=True)
    time.sleep(1.0)

    print("\n[5] 알파를 8초간 전진시키며 전체 편대 위치 샘플링...", flush=True)
    samples = {d_id: [] for d_id in VEHICLES}
    stop_flag = threading.Event()

    def sampler():
        t0 = time.time()
        while not stop_flag.is_set():
            for d_id, vname in VEHICLES.items():
                try:
                    s = client.getMultirotorState(vehicle_name=vname)
                    p = s.kinematics_estimated.position
                    samples[d_id].append((round(time.time() - t0, 2), p.x_val, p.y_val, p.z_val))
                except Exception:
                    pass
            time.sleep(0.2)

    sampler_thread = threading.Thread(target=sampler, daemon=True)
    sampler_thread.start()

    # Fly Alpha forward (body-frame +vx) for 8 seconds via the joystick API
    api_post("/api/joystick", {"drone_id": "Drone1", "vx": 3.0, "vy": 0, "vz": 0, "yaw_rate": 0, "duration": 8.0})
    time.sleep(8.5)
    api_post("/api/joystick", {"drone_id": "Drone1", "vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0, "duration": 0.3})
    time.sleep(3.0)  # let followers catch up a bit after Alpha stops

    stop_flag.set()
    sampler_thread.join(timeout=2.0)

    positions_after = {}
    for d_id, vname in VEHICLES.items():
        s = client.getMultirotorState(vehicle_name=vname)
        p = s.kinematics_estimated.position
        positions_after[d_id] = (p.x_val, p.y_val, p.z_val)

    print("\n" + "=" * 80, flush=True)
    print("[결과 - 이동 거리 및 방향]", flush=True)
    alpha_disp = dist(positions_before["Drone1"], positions_after["Drone1"])
    alpha_dx = positions_after["Drone1"][0] - positions_before["Drone1"][0]
    alpha_dy = positions_after["Drone1"][1] - positions_before["Drone1"][1]
    print(f"  - Alpha 이동 거리: {alpha_disp:.2f}m (dx={alpha_dx:.2f}, dy={alpha_dy:.2f})", flush=True)

    results = {}
    for d_id in ["Drone2", "Drone3", "Drone4"]:
        disp = dist(positions_before[d_id], positions_after[d_id])
        dx = positions_after[d_id][0] - positions_before[d_id][0]
        dy = positions_after[d_id][1] - positions_before[d_id][1]
        # dot product with alpha's displacement direction, to check it moved the SAME way (not randomly)
        if alpha_disp > 0.5:
            same_direction = (dx * alpha_dx + dy * alpha_dy) / (alpha_disp) > 0
        else:
            same_direction = None
        print(f"  - {LABELS[d_id]} 이동 거리: {disp:.2f}m (dx={dx:.2f}, dy={dy:.2f}) | 같은 방향={same_direction}", flush=True)
        results[d_id] = {"displacement": round(disp, 2), "same_direction": same_direction}

    followed = all(
        results[d]["displacement"] > 1.5 and results[d]["same_direction"] is not False
        for d in ["Drone2", "Drone3", "Drone4"]
    )
    print(f"\n  => {'Following Mode 정상 동작 (모든 편대기가 알파를 따라 이동함)' if followed else 'Following Mode 이상 - 일부 기체가 따라오지 않음'}", flush=True)
    print("=" * 80, flush=True)

    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "positions_before": positions_before,
        "positions_after": positions_after,
        "alpha_displacement": round(alpha_disp, 2),
        "follower_results": results,
        "followed": followed,
        "samples": samples,
    }
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    # Cleanup
    api_post("/api/following/toggle", {"enabled": False})
    time.sleep(0.5)
    for d_id, vname in VEHICLES.items():
        try:
            client.landAsync(vehicle_name=vname).join()
        except Exception:
            pass
    api_post("/api/simulators/stop")

    sys.exit(0 if followed else 1)


if __name__ == "__main__":
    main()
