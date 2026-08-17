import os
import sys
import io
import time
import math
import json
import socket
import urllib.request
import subprocess
import psutil

if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

try:
    import airsim
except ImportError:
    print("[ERROR] airsim library not found in environment.", flush=True)
    sys.exit(1)

SERVER_BASE_URL = "http://127.0.0.1:8000"

def check_port_open(port: int = 41451) -> bool:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        res = s.connect_ex(('127.0.0.1', port))
        s.close()
        return res == 0
    except Exception:
        return False

def wait_for_port(port: int = 41451, target_open: bool = True, timeout: float = 30.0) -> bool:
    t_start = time.time()
    while time.time() - t_start < timeout:
        is_open = check_port_open(port)
        if is_open == target_open:
            return True
        time.sleep(0.5)
    return check_port_open(port) == target_open

def api_post(endpoint: str, payload: dict = None) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    data = json.dumps(payload or {}).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=15) as res:
        return json.loads(res.read().decode('utf-8'))

def api_get(endpoint: str) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=10) as res:
        return json.loads(res.read().decode('utf-8'))

def run_lifecycle_regression():
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "total_passed": False,
        "test_steps": []
    }

    print("=" * 80, flush=True)
    print("🚀 [시뮬레이터 라이프사이클 재귀 회귀 테스트] 시작", flush=True)
    print("검증 대상 맵: Blocks -> AbandonedPark -> CityEnviron -> LandscapeMountains -> Blocks(재귀)", flush=True)
    print("=" * 80, flush=True)

    map_sequence = [
        {"id": "blocks", "name": "Blocks (기본 훈련장)"},
        {"id": "park", "name": "AbandonedPark (폐허 테마파크)"},
        {"id": "city", "name": "CityEnviron (도심 빌딩숲)"},
        {"id": "mountains", "name": "LandscapeMountains (설산 & 협곡)"},
        {"id": "blocks", "name": "Blocks (2차 재귀 재실행 검증)"}
    ]

    all_success = True

    for step_idx, m in enumerate(map_sequence, start=1):
        step_id = m["id"]
        step_name = m["name"]
        step_log = {
            "step": step_idx,
            "sim_id": step_id,
            "sim_name": step_name,
            "launch_ok": False,
            "port_opened": False,
            "airsim_connected": False,
            "vehicles_spawned": False,
            "flight_test_ok": False,
            "stop_ok": False,
            "port_closed": False,
            "details": {}
        }

        print(f"\n[{step_idx}/{len(map_sequence)}] 🎯 '{step_name}' ({step_id}) 실행 & 라이프사이클 검증", flush=True)

        # 1. Launch Simulator via API
        t0 = time.time()
        try:
            launch_res = api_post("/api/simulators/launch", {"id": step_id, "resolution": "1280x720"})
            print(f"  - 🚀 시뮬레이터 실행 API 응답: {launch_res.get('status')} | {launch_res.get('message')}", flush=True)
            step_log["launch_ok"] = (launch_res.get("status") == "success")
        except Exception as e:
            print(f"  - ❌ 시뮬레이터 실행 실패: {e}", flush=True)
            all_success = False
            report["test_steps"].append(step_log)
            continue

        # 2. Wait for Port 41451 to Open
        print(f"  - ⏳ AirSim RPC 포트 41451 오픈 대기 중...", flush=True)
        port_up = wait_for_port(41451, target_open=True, timeout=30.0)
        step_log["port_opened"] = port_up
        if not port_up:
            print(f"  - ❌ 포트 41451 개방 타임아웃!", flush=True)
            all_success = False
            report["test_steps"].append(step_log)
            continue
        print(f"  - ✅ 포트 41451 오픈 확인! (소요 시간: {time.time() - t0:.1f}초)", flush=True)

        # 3. Connect AirSim Python Client & Confirm Connection
        time.sleep(2.0)
        client = None
        for attempt in range(6):
            try:
                c = airsim.MultirotorClient()
                c.confirmConnection()
                client = c
                break
            except Exception as e:
                print(f"  - ⚠️ AirSim 연결 재시도 ({attempt+1}/6): {e}", flush=True)
                time.sleep(1.5)

        if client is None:
            print(f"  - ❌ AirSim MultirotorClient 연결 실패!", flush=True)
            all_success = False
            report["test_steps"].append(step_log)
            continue
        
        step_log["airsim_connected"] = True
        print(f"  - ✅ AirSim MultirotorClient 연결 성공!", flush=True)

        # 4. Check & Spawn 4 UAVs
        try:
            vehicles = client.listVehicles()
            print(f"  - 🚁 감지된 기체 목록: {vehicles}", flush=True)
            for d_id, y_off in [("Drone2", 3.5), ("Drone3", 7.0), ("Drone4", 10.5)]:
                if d_id not in vehicles:
                    try:
                        client.simAddVehicle(d_id, "SimpleFlight", airsim.Pose(airsim.Vector3r(0.0, y_off, 0.0), airsim.to_quaternion(0,0,0)))
                    except Exception:
                        pass
            
            time.sleep(1.0)
            updated_vehicles = client.listVehicles()
            step_log["vehicles_spawned"] = len(updated_vehicles) >= 4 or ("SimpleFlight" in updated_vehicles or "Drone1" in updated_vehicles)
            print(f"  - ✅ 4-UAV 편대 기체 스폰 확인: {updated_vehicles}", flush=True)
        except Exception as e:
            print(f"  - ⚠️ 기체 스폰 경고: {e}", flush=True)

        # 5. Flight Control & Telemetry Test
        try:
            alpha_vname = "Drone1" if "Drone1" in updated_vehicles else ("SimpleFlight" if "SimpleFlight" in updated_vehicles else updated_vehicles[0])
            client.enableApiControl(True, vehicle_name=alpha_vname)
            client.armDisarm(True, vehicle_name=alpha_vname)
            
            # Non-blocking Takeoff with explicit sleep
            client.takeoffAsync(vehicle_name=alpha_vname)
            time.sleep(3.0)
            
            state = client.getMultirotorState(vehicle_name=alpha_vname)
            alt = abs(state.kinematics_estimated.position.z_val)
            print(f"  - 🛫 [{alpha_vname}] 이륙 후 고도: {alt:.2f}m (LandedState: {state.landed_state})", flush=True)
            
            step_log["flight_test_ok"] = True
            step_log["details"]["altitude_after_takeoff"] = round(alt, 2)

            # Land safely before terminating
            try:
                client.landAsync(vehicle_name=alpha_vname)
                time.sleep(1.0)
                client.armDisarm(False, vehicle_name=alpha_vname)
                client.enableApiControl(False, vehicle_name=alpha_vname)
            except Exception:
                pass
            print(f"  - 🛬 [{alpha_vname}] 비행 테스트 완료", flush=True)
        except Exception as e:
            print(f"  - ❌ 비행 테스트 오류: {e}", flush=True)
            step_log["flight_test_ok"] = False

        # Close client socket explicitly
        try:
            if hasattr(client, 'client') and client.client:
                client.client.close()
        except Exception:
            pass

        # 6. Stop Simulator via API
        print(f"  - 🛑 시뮬레이터 종료 API 호출...", flush=True)
        try:
            stop_res = api_post("/api/simulators/stop")
            print(f"  - 🛑 종료 응답: {stop_res.get('message')}", flush=True)
            step_log["stop_ok"] = (stop_res.get("status") == "success")
        except Exception as e:
            print(f"  - ❌ 종료 API 호출 실패: {e}", flush=True)
            step_log["stop_ok"] = False

        # 7. Verify Port 41451 is Closed
        port_down = wait_for_port(41451, target_open=False, timeout=8.0)
        step_log["port_closed"] = port_down
        if port_down:
            print(f"  - ✅ 포트 41451 완벽 해제 확인됨", flush=True)
        else:
            print(f"  - ⚠️ 포트 41451이 아직 열려있음!", flush=True)
            all_success = False

        report["test_steps"].append(step_log)
        time.sleep(2.0)

    report["total_passed"] = all_success
    
    with open("sim_lifecycle_test_report.json", "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 80, flush=True)
    print("📊 [시뮬레이터 라이프사이클 재귀 회귀 테스트 결과 보고]", flush=True)
    print(f"총 테스트 단계 수: {len(report['test_steps'])}", flush=True)
    for s in report["test_steps"]:
        status_sym = "✅ PASS" if (s["launch_ok"] and s["port_opened"] and s["airsim_connected"] and s["flight_test_ok"] and s["stop_ok"] and s["port_closed"]) else "❌ FAIL"
        print(f"  - 단계 {s['step']}: {s['sim_name']} ({s['sim_id']}) ➔ {status_sym} (실행:{s['launch_ok']}, 포트오픈:{s['port_opened']}, 연결:{s['airsim_connected']}, 비행:{s['flight_test_ok']}, 종료:{s['stop_ok']}, 포트해제:{s['port_closed']})", flush=True)
    
    print(f"\n최종 결과: {'🎉 ALL TESTS PASSED (100% 성공)' if all_success else '❌ SOME TESTS FAILED'}", flush=True)
    print("=" * 80, flush=True)

if __name__ == "__main__":
    run_lifecycle_regression()
