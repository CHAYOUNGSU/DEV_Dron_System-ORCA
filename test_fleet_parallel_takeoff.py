"""
Diagnostic: does POST /api/fleet/takeoff actually launch all 4 drones in
parallel, or sequentially (Alpha -> Bravo -> Charlie -> Delta, one after
another)? Samples all 4 vehicles' altitude at high frequency while the
(server-side blocking) fleet takeoff/land call is in flight, so a sequential
"staircase" pattern is directly visible instead of guessed at.
"""
import sys
import io
import os
import json
import time
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
REPORT_PATH = "fleet_parallel_report.json"
VEHICLES = ["SimpleFlight", "Drone2", "Drone3", "Drone4"]
LABELS = {"SimpleFlight": "Alpha", "Drone2": "Bravo", "Drone3": "Charlie", "Drone4": "Delta"}


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


def sample_all(client, samples, stop_flag, t0, hz=20):
    while not stop_flag.is_set():
        t_rel = round(time.time() - t0, 3)
        row = {"t": t_rel}
        for v in VEHICLES:
            try:
                s = client.getMultirotorState(vehicle_name=v)
                row[v] = round(float(s.kinematics_estimated.position.z_val), 3)
            except Exception:
                row[v] = None
        samples.append(row)
        time.sleep(1.0 / hz)


def find_climb_start(samples, vname, ground_z_threshold=-1.0, moved_threshold=0.3):
    """First timestamp where this vehicle's altitude has moved noticeably from its first sample."""
    vals = [(s["t"], s[vname]) for s in samples if s.get(vname) is not None]
    if not vals:
        return None
    base = vals[0][1]
    for t, z in vals:
        if abs(z - base) > moved_threshold:
            return t
    return None


def run_phase(client, label, action_fn):
    samples = []
    stop_flag = threading.Event()
    t0 = time.time()
    sampler = threading.Thread(target=sample_all, args=(client, samples, stop_flag, t0), daemon=True)
    sampler.start()

    result = {}

    def trigger():
        time.sleep(0.5)
        t_call = time.time()
        res = action_fn()
        result["status"] = res.get("status")
        result["message"] = res.get("message")
        result["duration"] = round(time.time() - t_call, 2)

    trigger_thread = threading.Thread(target=trigger, daemon=True)
    trigger_thread.start()
    trigger_thread.join()
    time.sleep(2.0)
    stop_flag.set()
    sampler.join(timeout=2.0)

    print(f"\n[{label}] API 응답 ({result.get('duration')}초): {result.get('status')} | {result.get('message')}", flush=True)

    starts = {}
    for v in VEHICLES:
        t_start = find_climb_start(samples, v)
        starts[v] = t_start
        label_kr = LABELS[v]
        print(f"  - {label_kr}({v}) 움직임 시작 시각: {t_start if t_start is not None else 'N/A'}", flush=True)

    valid_starts = [t for t in starts.values() if t is not None]
    spread = (max(valid_starts) - min(valid_starts)) if len(valid_starts) >= 2 else None
    print(f"  - 4대 움직임 시작 시각 차이(spread): {spread}", flush=True)
    return {"samples": samples, "starts": starts, "spread": spread, "api_result": result}


def main():
    print("=" * 80, flush=True)
    print("[전체 편대 동시 이착륙 - 병렬성 진단 테스트]", flush=True)
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

    vehicles = client.listVehicles()
    print(f"  - 감지된 차량: {vehicles}", flush=True)
    time.sleep(1.0)

    takeoff_result = run_phase(client, "전체 동시 이륙", lambda: api_post("/api/fleet/takeoff"))
    time.sleep(1.5)
    land_result = run_phase(client, "전체 동시 착륙", lambda: api_post("/api/fleet/land"))

    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "takeoff": {"starts": takeoff_result["starts"], "spread": takeoff_result["spread"], "api_result": takeoff_result["api_result"]},
        "land": {"starts": land_result["starts"], "spread": land_result["spread"], "api_result": land_result["api_result"]},
    }
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 80, flush=True)
    to_spread = takeoff_result["spread"]
    ld_spread = land_result["spread"]
    to_parallel = to_spread is not None and to_spread < 1.0
    ld_parallel = ld_spread is not None and ld_spread < 1.0
    print(f"이륙 동시성: spread={to_spread}s -> {'병렬 (OK)' if to_parallel else '순차 (문제)'}", flush=True)
    print(f"착륙 동시성: spread={ld_spread}s -> {'병렬 (OK)' if ld_parallel else '순차 (문제)'}", flush=True)
    print("=" * 80, flush=True)

    api_post("/api/simulators/stop")
    sys.exit(0 if (to_parallel and ld_parallel) else 1)


if __name__ == "__main__":
    main()
