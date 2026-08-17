"""
Diagnostic: does POST /api/formation/assemble cause Alpha's OWN altitude to
dip and recover, even though Alpha is only supposed to hold position while
the wingmen fly to it?

The existing test_flight_regression.py only snapshots Alpha's altitude
before the call and ~8s after it returns, so a transient mid-call dip that
recovers by the end would never show up there. This script instead samples
Alpha's z-position at ~20Hz for the whole duration of the call (which is
triggered in a background thread since the HTTP handler blocks until the
formation maneuver finishes), producing an altitude-vs-time curve that shows
exactly when and by how much Alpha moves.

Requires: python server.py running, and enough free RAM/GPU to boot Blocks.
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
REPORT_PATH = "formation_altitude_dip_report.json"
SAMPLE_HZ = 20


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


def wait_for_port(port: int = 41451, target_open: bool = True, timeout: float = 60.0) -> bool:
    t_start = time.time()
    while time.time() - t_start < timeout:
        if check_port_open(port) == target_open:
            return True
        time.sleep(0.3)
    return check_port_open(port) == target_open


def main():
    print("=" * 80, flush=True)
    print("[편대 집결 - 알파 고도 하강 진단 테스트]", flush=True)
    print("=" * 80, flush=True)

    print("\n[1] Blocks 시뮬레이터 실행...", flush=True)
    res = api_post("/api/simulators/launch", {"id": "blocks", "resolution": "1280x720"})
    print(f"  - {res.get('status')}: {res.get('message')}", flush=True)
    assert wait_for_port(41451, True, 60.0), "AirSim RPC 포트 오픈 대기 타임아웃"
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
    vname = "Drone1" if "Drone1" in vehicles else ("SimpleFlight" if "SimpleFlight" in vehicles else vehicles[0])
    print(f"  - Alpha vehicle name: {vname} | 전체 목록: {vehicles}", flush=True)

    # Fly Alpha up using ONLY the server's own HTTP API (takeoff + joystick), the
    # same way a real user would - not a second, separate airsim client, which
    # would fight the server's own client_control for API-control ownership of
    # the vehicle and could itself distort the reproduction.
    print("\n[2] Alpha를 서버 API(takeoff + joystick)로 고도 약 12m까지 상승...", flush=True)
    res_takeoff = api_post("/api/takeoff", {"drone_id": "Drone1"})
    print(f"  - takeoff: {res_takeoff.get('status')}", flush=True)
    time.sleep(1.0)
    res_climb = api_post("/api/joystick", {"drone_id": "Drone1", "vx": 0, "vy": 0, "vz": -3.0, "yaw_rate": 0, "duration": 3.0})
    print(f"  - climb joystick: {res_climb.get('status')}", flush=True)
    time.sleep(3.5)
    # stop the climb (hover) before measuring baseline
    api_post("/api/joystick", {"drone_id": "Drone1", "vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0, "duration": 0.2})
    time.sleep(1.0)

    state0 = client.getMultirotorState(vehicle_name=vname)
    baseline_z = state0.kinematics_estimated.position.z_val
    print(f"  - 기준 고도: z={baseline_z:.2f} ({abs(baseline_z):.2f}m) | landed_state={state0.landed_state}", flush=True)

    # Isolate whether Alpha's dive is triggered by the WINGMEN's takeoffAsync()
    # calls (dispatched inside the same formation-assemble call, while Alpha is
    # already hovering) by pre-flying all 3 wingmen BEFORE triggering assemble,
    # so this run's assemble call does NOT need to take any of them off.
    if os.environ.get("PREFLY_WINGMEN") == "1":
        print("\n[2b] 편대기(윙맨) 3대를 미리 이륙시켜서 '집결 호출 시 이륙 발생 안함' 조건 생성...", flush=True)
        for d_id in ["Drone2", "Drone3", "Drone4"]:
            r = api_post("/api/takeoff", {"drone_id": d_id})
            print(f"  - {d_id} takeoff: {r.get('status')}", flush=True)
        time.sleep(3.0)
        for w in ["Drone2", "Drone3", "Drone4"]:
            sw = client.getMultirotorState(vehicle_name=w)
            print(f"  - {w} landed_state={sw.landed_state} z={sw.kinematics_estimated.position.z_val:.2f}", flush=True)

    # 3. Sample Alpha's altitude continuously while triggering formation assemble
    # in the background (the HTTP call blocks server-side for several seconds).
    samples = []
    stop_flag = threading.Event()

    def sampler():
        t_start = time.time()
        while not stop_flag.is_set():
            t_rel = round(time.time() - t_start, 3)
            try:
                s = client.getMultirotorState(vehicle_name=vname)
                z = s.kinematics_estimated.position.z_val
                coll = client.simGetCollisionInfo(vehicle_name=vname)
                samples.append({
                    "t": t_rel, "z": round(float(z), 3),
                    "api_control": client.isApiControlEnabled(vehicle_name=vname),
                    "collided": bool(coll.has_collided),
                })
            except Exception as e:
                samples.append({"t": t_rel, "z": None, "error": str(e)})
            time.sleep(1.0 / SAMPLE_HZ)

    api_result = {}

    def trigger_formation():
        time.sleep(1.0)  # capture a clean baseline window first
        t_call = time.time()
        print(f"\n[3] POST /api/formation/assemble 호출 (t={t_call - sampler_t0:.2f}s)...", flush=True)
        res = api_post("/api/formation/assemble", {"spacing": 12.0, "velocity": 4.0})
        api_result["status"] = res.get("status")
        api_result["message"] = res.get("message")
        api_result["call_duration_sec"] = round(time.time() - t_call, 2)
        print(f"  - 응답 ({api_result['call_duration_sec']}초 소요): {res.get('status')} | {res.get('message')}", flush=True)

    sampler_t0 = time.time()
    sampler_thread = threading.Thread(target=sampler, daemon=True)
    sampler_thread.start()

    trigger_thread = threading.Thread(target=trigger_formation, daemon=True)
    trigger_thread.start()
    trigger_thread.join()

    time.sleep(4.0)  # keep sampling a bit after the call returns to see full recovery
    stop_flag.set()
    sampler_thread.join(timeout=2.0)

    # 4. Analyze the altitude-vs-time curve
    valid = [s for s in samples if s.get("z") is not None]
    zs = [s["z"] for s in valid]
    max_z = max(zs)  # least negative z = LOWEST altitude point reached (NED: more negative = higher)
    max_z_sample = next(s for s in valid if s["z"] == max_z)
    dip_meters = max_z - baseline_z  # positive = altitude was lost

    print("\n" + "=" * 80, flush=True)
    print("[고도(z) 시계열 - 0.25초 간격 요약]", flush=True)
    last_printed = -1.0
    for s in valid:
        if s["t"] - last_printed >= 0.25:
            alt_m = abs(s["z"])
            bar = "#" * int(alt_m)
            flags = f"  api={s.get('api_control')} collided={s.get('collided')}"
            print(f"  t={s['t']:6.2f}s  z={s['z']:7.2f}  alt={alt_m:5.2f}m  {bar}{flags}", flush=True)
            last_printed = s["t"]

    print("\n" + "=" * 80, flush=True)
    print("[결과]", flush=True)
    print(f"  - 기준 고도: {abs(baseline_z):.2f}m (z={baseline_z:.2f})", flush=True)
    print(f"  - 최저 고도: {abs(max_z):.2f}m (z={max_z:.2f}) @ t={max_z_sample['t']:.2f}s", flush=True)
    print(f"  - 하강폭: {dip_meters:.2f}m", flush=True)
    print(f"  - API 응답: {api_result}", flush=True)

    dip_detected = dip_meters > 1.0
    print(f"\n  => {'하강 현상 재현됨 (dip_detected=True)' if dip_detected else '하강 현상 재현 안됨'}", flush=True)
    print("=" * 80, flush=True)

    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "baseline_z": baseline_z,
        "max_z": max_z,
        "dip_meters": round(dip_meters, 2),
        "dip_detected": dip_detected,
        "api_result": api_result,
        "samples": samples,
    }
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    # Cleanup: land and stop
    try:
        client.landAsync(vehicle_name=vname).join()
    except Exception:
        pass
    api_post("/api/simulators/stop")


if __name__ == "__main__":
    main()
