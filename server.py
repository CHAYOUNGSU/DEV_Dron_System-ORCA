import sys
import io
import os
import math
import socket
import threading
import asyncio
import subprocess
import base64
import time
from collections import deque
import cv2
import numpy as np
import psutil
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# UTF-8 Console output
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

try:
    import airsim
    HAS_AIRSIM_LIB = True
except ImportError:
    HAS_AIRSIM_LIB = False

app = FastAPI(title="AirSim 4-UAV Fleet Command & Formation Flight Cockpit")

# Available Simulator Assets
SIMULATORS = {
    "blocks": {
        "id": "blocks",
        "name": "Blocks (기본 비행 훈련장)",
        "icon": "",
        "desc": "기본 장애물 충돌 회피, 호버링 및 4대 편대 정밀 착륙 연습 맵",
        "rel_path": os.path.join("assets", "Blocks", "Blocks", "WindowsNoEditor", "Blocks.exe"),
        "exe_names": ["Blocks.exe", "Blocks-Win64-Shipping.exe"]
    },
    "city": {
        "id": "city",
        "name": "CityEnviron (도심 고층 빌딩숲)",
        "icon": "",
        "desc": "초고층 빌딩 사이 4대 편대 자율 비행 및 고도 관제 맵",
        "rel_path": os.path.join("assets", "CityEnviron", "WindowsNoEditor", "CityEnviron.exe"),
        "exe_names": ["CityEnviron.exe", "CityEnviron-Win64-Shipping.exe"]
    },
    "mountains": {
        "id": "mountains",
        "name": "LandscapeMountains (설산 & 협곡)",
        "icon": "",
        "desc": "광활한 설산 협곡, 장거리 초고속 편대 종대 비행 및 고고도 RTH 테스트 맵",
        "rel_path": os.path.join("assets", "LandscapeMountains", "LandscapeMountains", "WindowsNoEditor", "LandscapeMountains.exe"),
        "exe_names": ["LandscapeMountains.exe", "LandscapeMountains-Win64-Shipping.exe"]
    },
    "park": {
        "id": "park",
        "name": "AbandonedPark (폐허 테마파크)",
        "icon": "",
        "desc": "롤러코스터, 대관람차 등 복잡한 3D 놀이기구 사이 4대 정밀 탐색 맵",
        "rel_path": os.path.join("assets", "AbandonedPark", "AbandonedPark", "WindowsNoEditor", "AbandonedPark.exe"),
        "exe_names": ["AbandonedPark.exe", "AbandonedPark-Win64-Shipping.exe"]
    }
}

# Currently Selected Active Drone by User ('Drone1', 'Drone2', 'Drone3', 'Drone4')
selected_drone_id = "Drone1"

# 4-UAV Fleet Configuration (Alpha: Leader, Bravo/Charlie/Delta: Wingmen)
DRONES_CONFIG = {
    "Drone1": {
        "id": "Drone1",
        "name": "Drone 1 (알파 / 편대장)",
        "role": "편대장 (Leader)",
        "tag": "ALPHA-01",
        "color": "#38bdf8",
        "badge_class": "badge-alpha",
        "order": 0,
        "spawn_offset": (0.0, 0.0, 0.0)
    },
    "Drone2": {
        "id": "Drone2",
        "name": "Drone 2 (브라보 / 2호기)",
        "role": "편대 2번기 (Wingman 1)",
        "tag": "BRAVO-02",
        "color": "#f59e0b",
        "badge_class": "badge-bravo",
        "order": 1,
        "spawn_offset": (0.0, 3.5, 0.0)
    },
    "Drone3": {
        "id": "Drone3",
        "name": "Drone 3 (찰리 / 3호기)",
        "role": "편대 3번기 (Wingman 2)",
        "tag": "CHARLIE-03",
        "color": "#10b981",
        "badge_class": "badge-charlie",
        "order": 2,
        "spawn_offset": (0.0, 7.0, 0.0)
    },
    "Drone4": {
        "id": "Drone4",
        "name": "Drone 4 (델타 / 4호기)",
        "role": "편대 4번기 (Wingman 3)",
        "tag": "DELTA-04",
        "color": "#a855f7",
        "badge_class": "badge-delta",
        "order": 3,
        "spawn_offset": (0.0, 10.5, 0.0)
    }
}

# Dual Independent AirSim Clients
client_telemetry = None
client_control = None
is_airsim_connected = False

control_lock = threading.Lock()

# RPC calls default to a 3600s (1 hour) timeout inside the airsim client library.
# The background worker must never block that long. is_switching_simulator only
# stops the worker from STARTING a new connection during a map switch - it can't
# do anything about a call that was already in flight the instant the old
# process gets killed (the telemetry loop issues 5 RPC calls every ~40ms, so
# there is nearly always one in flight). That in-flight call still blocks for
# the full timeout before the worker notices and recovers, so keep this short:
# a healthy AirSim instance answers in well under 100ms.
WORKER_RPC_TIMEOUT_SEC = 2

# Set for the full duration of a simulator kill -> relaunch cycle. While True,
# airsim_worker() must not attempt to (re)connect at all: the old process's RPC
# port can still be briefly open while it's dying, and connecting to it right
# then can leave the worker stuck on a call that never returns (see
# WORKER_RPC_TIMEOUT_SEC above) - previously this permanently froze telemetry
# and the camera feed after the first map switch.
is_switching_simulator = False

# =========================================================================
# 🦆 Following Mode (Duckling Chain Autopilot)
# =========================================================================
# Bravo follows Alpha, Charlie follows Bravo, Delta follows Charlie - each
# drone targets its leader's position from FOLLOW_LAG_SECONDS ago (not a fixed
# geometric offset), so followers literally retrace the leader's actual flown
# path rather than snapping to a rigid slot. That is what gives it the loose,
# organic "mother duck and ducklings" feel instead of a locked formation.
FOLLOW_CHAIN = {"Drone2": "Drone1", "Drone3": "Drone2", "Drone4": "Drone3"}
FOLLOW_HISTORY_DRONES = ["Drone1", "Drone2", "Drone3"]  # every drone that is someone's leader
FOLLOW_TICK_INTERVAL_SEC = 0.3

following_mode_enabled = False
following_lag_seconds = 2.0
following_velocity = 3.0
position_history = {d_id: deque(maxlen=400) for d_id in FOLLOW_HISTORY_DRONES}  # (t, x, y, z), ~2min @ 25Hz worst case

def record_position_history(d_id, x, y, z):
    if d_id in position_history:
        position_history[d_id].append((time.time(), x, y, z))

def is_follower_locked(drone_id: str) -> bool:
    """True while Following Mode is autopiloting this drone - it should not
    also be fighting a manual joystick/takeoff/rotate/RTH command at the same
    time. Landing is intentionally exempt (always allowed as a safety override)."""
    return following_mode_enabled and drone_id in FOLLOW_CHAIN

def get_lagged_leader_position(leader_id, lag_seconds):
    """Most recent recorded position of leader_id at least lag_seconds old (i.e. 'where the leader was')."""
    hist = position_history.get(leader_id)
    if not hist:
        return None
    target_t = time.time() - lag_seconds
    best = None
    for entry in hist:
        if entry[0] <= target_t:
            best = entry
        else:
            break
    return best or hist[0]

def create_empty_telemetry(drone_id):
    cfg = DRONES_CONFIG[drone_id]
    return {
        "drone_id": drone_id,
        "drone_name": cfg["name"],
        "drone_role": cfg["role"],
        "drone_tag": cfg["tag"],
        "badge_color": cfg["color"],
        "connected": False,
        "simulated": True,
        "active_sim_id": None,
        "active_sim_name": "미실행 (선택 대기)",
        "active_sim_icon": "🟡",
        "armed": False,
        "api_control": False,
        "landed_state": "Landed",
        "x": cfg["spawn_offset"][0],
        "y": cfg["spawn_offset"][1],
        "z": 0.0,
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
        "speed": 0.0,
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 0.0,
        "lat": 47.641468,
        "lon": -122.140165,
        "alt": 0.0,
        "battery": 98.5,
        "status_msg": "시뮬레이터 대기 중"
    }

latest_telemetries = {
    "Drone1": create_empty_telemetry("Drone1"),
    "Drone2": create_empty_telemetry("Drone2"),
    "Drone3": create_empty_telemetry("Drone3"),
    "Drone4": create_empty_telemetry("Drone4")
}

latest_frames_b64 = {
    "Drone1": "",
    "Drone2": "",
    "Drone3": "",
    "Drone4": ""
}

latest_frames_time = {
    "Drone1": time.time(),
    "Drone2": time.time(),
    "Drone3": time.time(),
    "Drone4": time.time()
}

def close_airsim_client(client):
    """Safely close AirSim RPC client socket connection"""
    if client is not None:
        try:
            if hasattr(client, 'client') and client.client:
                client.client.close()
        except Exception:
            pass

def detect_running_simulator():
    """Scan running processes for known AirSim executables"""
    all_running_exes = set()
    try:
        for p in psutil.process_iter(['name']):
            try:
                name = p.info['name']
                if name:
                    all_running_exes.add(name.lower())
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
    except Exception:
        pass

    for sim_id, info in SIMULATORS.items():
        for target in info['exe_names']:
            if target.lower() in all_running_exes:
                return {
                    "id": sim_id,
                    "name": info["name"],
                    "icon": info["icon"]
                }
    return None

def check_port_open(port: int = 41451) -> bool:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.3)
        res = s.connect_ex(('127.0.0.1', port))
        s.close()
        return res == 0
    except Exception:
        return False

def wait_port_closed(port: int = 41451, timeout: float = 5.0) -> bool:
    """Wait until the RPC port is completely released and closed"""
    t_start = time.time()
    while time.time() - t_start < timeout:
        if not check_port_open(port):
            return True
        time.sleep(0.2)
    return not check_port_open(port)

def wait_port_open(port: int = 41451, timeout: float = 15.0) -> bool:
    """Wait until the RPC port is open and accepting connections"""
    t_start = time.time()
    while time.time() - t_start < timeout:
        if check_port_open(port):
            return True
        time.sleep(0.3)
    return check_port_open(port)

def kill_all_simulators():
    """Robustly terminate any running AirSim processes and release RPC port"""
    global client_control, client_telemetry, is_airsim_connected, cached_vehicles_list, last_vehicles_check, is_switching_simulator

    # 0. Block the worker thread from (re)connecting for the whole teardown -
    # the old process's RPC port can still be open for a moment while it dies,
    # and reconnecting to it right then can hang the worker (see
    # WORKER_RPC_TIMEOUT_SEC). Caller is responsible for clearing this flag
    # once it is safe to resume (new process launched, or nothing more to do).
    is_switching_simulator = True

    # 1. Close active Python RPC sockets first
    is_airsim_connected = False
    close_airsim_client(client_control)
    close_airsim_client(client_telemetry)
    client_control = None
    client_telemetry = None
    # Stale vehicle-name cache from the previous map must never leak into the next one
    cached_vehicles_list = []
    last_vehicles_check = 0.0
    print("[KILL] 🧹 RPC 클라이언트 소켓 종료 및 차량 목록 캐시 초기화 완료", flush=True)

    all_target_names = set()
    for s in SIMULATORS.values():
        for n in s['exe_names']:
            all_target_names.add(n.lower())
    all_target_names.add("crashreportclient.exe")
    all_target_names.add("crashreportclient-win64-shipping.exe")

    procs_to_kill = []
    killed_names = []

    # 2. Gather processes and all recursive children via psutil
    try:
        for p in psutil.process_iter(['name', 'pid']):
            try:
                pname = p.info['name']
                if pname and pname.lower() in all_target_names:
                    try:
                        for child in p.children(recursive=True):
                            procs_to_kill.append(child)
                    except Exception:
                        pass
                    procs_to_kill.append(p)
                    killed_names.append(pname)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
    except Exception:
        pass

    for p in procs_to_kill:
        try:
            p.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    # 3. Force kill using Windows taskkill for guaranteed cleanup
    if sys.platform == 'win32':
        for target in all_target_names:
            try:
                subprocess.run(
                    ["taskkill", "/F", "/T", "/IM", target],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False
                )
            except Exception:
                pass

    # 4. Wait for processes to disappear
    if procs_to_kill:
        gone, alive = psutil.wait_procs(procs_to_kill, timeout=2.0)
        for p in alive:
            try:
                p.kill()
            except Exception:
                pass

    # 5. Wait for RPC socket port 41451 to be released
    wait_port_closed(41451, timeout=4.0)

    # 6. Reset drone status to simulated
    for d_id in ["Drone1", "Drone2", "Drone3", "Drone4"]:
        latest_telemetries[d_id]["connected"] = False
        latest_telemetries[d_id]["simulated"] = True
        latest_telemetries[d_id]["armed"] = False
        latest_telemetries[d_id]["api_control"] = False
        latest_telemetries[d_id]["landed_state"] = "Landed"
        latest_telemetries[d_id]["speed"] = 0.0
        latest_telemetries[d_id]["active_sim_name"] = "시뮬레이터 미실행"
        latest_telemetries[d_id]["active_sim_icon"] = "🟡"
        latest_telemetries[d_id]["active_sim_id"] = None
        latest_telemetries[d_id]["status_msg"] = "시뮬레이터 대기 중"

    return list(set(killed_names))

cached_vehicles_list = []
last_vehicles_check = 0.0

def get_real_vehicle_name(client, requested_drone_id: str) -> str:
    """Resolve AirSim internal vehicle name for Drone1, Drone2, Drone3, Drone4 using cached list"""
    global cached_vehicles_list, last_vehicles_check
    t_now = time.time()
    if t_now - last_vehicles_check > 1.5 or not cached_vehicles_list:
        try:
            cached_vehicles_list = client.listVehicles()
            last_vehicles_check = t_now
        except Exception:
            pass

    vehicles = cached_vehicles_list
    if requested_drone_id == "Drone1":
        if "Drone1" in vehicles:
            return "Drone1"
        if "SimpleFlight" in vehicles:
            return "SimpleFlight"
        if len(vehicles) > 0:
            return vehicles[0]
        return "SimpleFlight"
    elif requested_drone_id in ["Drone2", "Drone3", "Drone4"]:
        if requested_drone_id in vehicles:
            return requested_drone_id
        # Dynamic spawn if missing
        offset = DRONES_CONFIG[requested_drone_id]["spawn_offset"]
        try:
            client.simAddVehicle(requested_drone_id, "SimpleFlight", airsim.Pose(airsim.Vector3r(offset[0], offset[1], offset[2]), airsim.to_quaternion(0, 0, 0)))
            cached_vehicles_list.append(requested_drone_id)
            return requested_drone_id
        except Exception:
            pass
        return requested_drone_id
    return requested_drone_id

def ensure_all_drones_spawned(client) -> bool:
    """
    Ensure Drone 2, Drone 3, Drone 4 are spawned into the simulation environment.
    Returns True only if a post-spawn listVehicles() actually confirms all 3 wingmen
    are present. A freshly-opened AirSim RPC port does not guarantee the level has
    finished BeginPlay, so simAddVehicle can silently no-op on a too-early call -
    the caller must be able to tell success from failure and retry.
    """
    global cached_vehicles_list, last_vehicles_check
    try:
        vehicles = client.listVehicles()
        cached_vehicles_list = vehicles
        last_vehicles_check = time.time()
        print(f"[SPAWN] 🔍 현재 감지된 차량 목록: {vehicles}", flush=True)

        for d_id in ["Drone2", "Drone3", "Drone4"]:
            if d_id not in vehicles:
                offset = DRONES_CONFIG[d_id]["spawn_offset"]
                try:
                    client.simAddVehicle(d_id, "SimpleFlight", airsim.Pose(airsim.Vector3r(offset[0], offset[1], offset[2]), airsim.to_quaternion(0, 0, 0)))
                    print(f"[SPAWN] 🚀 {d_id} 스폰 요청 완료 (offset={offset})", flush=True)
                except Exception as e_spawn:
                    print(f"[SPAWN] ⚠️ {d_id} 스폰 요청 실패: {type(e_spawn).__name__}: {e_spawn}", flush=True)

        cached_vehicles_list = client.listVehicles()
        last_vehicles_check = time.time()
        missing = [d for d in ["Drone2", "Drone3", "Drone4"] if d not in cached_vehicles_list]
        if missing:
            print(f"[SPAWN] ❌ 스폰 검증 실패 - 아직 없는 기체: {missing} | 전체 목록: {cached_vehicles_list}", flush=True)
            return False
        print(f"[SPAWN] ✅ 4-UAV 편대 스폰 검증 완료: {cached_vehicles_list}", flush=True)
        return True
    except Exception as e:
        print(f"[SPAWN] ❌ 스폰 절차 중 예외 발생: {type(e).__name__}: {e}", flush=True)
        return False

def get_telemetry_snapshot(drone_id: str = None):
    d_id = drone_id or selected_drone_id
    t = latest_telemetries.get(d_id, latest_telemetries["Drone1"])
    return f"[{t.get('drone_tag', d_id)} | 상태: {t.get('landed_state', 'Unknown')} | 고도: {abs(t['z']):.1f}m | 좌표: ({t['x']:.1f}, {t['y']:.1f}, {t['z']:.1f}) | 자세(P/R/Y): {t['pitch']:.1f}°/{t['roll']:.1f}°/{t['yaw']:.1f}°]"

# 🚀 Dedicated Lock-Free 4-UAV Fleet Telemetry & Camera Worker Thread (25 FPS)
def airsim_worker():
    global client_telemetry, is_airsim_connected, latest_telemetries, latest_frames_b64, latest_frames_time, cached_vehicles_list
    print("[WORKER] 🚀 4-UAV Fleet Telemetry & Process Monitor Thread Started")
    tick = 0
    last_proc_check = 0
    cached_sim_info = None
    spawn_verified = False
    last_spawn_attempt = 0.0
    connect_started_at = 0.0
    per_drone_error_logged_at = {"Drone1": 0.0, "Drone2": 0.0, "Drone3": 0.0, "Drone4": 0.0}
    last_frame_raw_bytes = {}
    stale_frame_streak = {}

    while True:
        tick += 1
        t_now = time.time()

        # Check active simulator process every 1 second
        if t_now - last_proc_check > 1.0:
            cached_sim_info = detect_running_simulator()
            last_proc_check = t_now

        # A simulator kill->relaunch cycle is in progress: the old process's RPC
        # port can still be briefly open while it's dying. Reconnecting to it in
        # that window can hang a call against a process that's mid-death, and
        # with the airsim client's default 3600s timeout that would freeze this
        # entire loop (telemetry + camera) until the process exits this function -
        # which it never does. Just wait it out.
        if is_switching_simulator:
            time.sleep(0.05)
            continue

        port_open = check_port_open()

        if port_open and HAS_AIRSIM_LIB:
            try:
                sim_name = cached_sim_info["name"] if cached_sim_info else "AirSim 엔진"
                sim_icon = cached_sim_info["icon"] if cached_sim_info else "🎮"
                sim_id = cached_sim_info["id"] if cached_sim_info else "airsim"

                if client_telemetry is None or not is_airsim_connected:
                    connect_started_at = t_now
                    print(f"[WORKER] 🔌 RPC 포트 오픈 감지, MultirotorClient 연결 시도... (대상: {sim_name})", flush=True)
                    c = airsim.MultirotorClient(timeout_value=WORKER_RPC_TIMEOUT_SEC)
                    c.confirmConnection()
                    client_telemetry = c
                    is_airsim_connected = True
                    spawn_verified = False
                    last_spawn_attempt = 0.0
                    cached_vehicles_list = []
                    print(f"[WORKER] ✅ 텔레메트리 클라이언트 연결 성공 ({sim_name}) | 포트 오픈 후 {t_now - connect_started_at:.2f}초 소요", flush=True)

                # Retry spawn verification until all 4 vehicles are confirmed present.
                # A freshly-opened RPC port does not mean the level has finished
                # loading, so the first attempt can legitimately fail - keep retrying
                # every 1s instead of locking in a broken fleet state forever.
                if not spawn_verified and client_telemetry is not None and (t_now - last_spawn_attempt > 1.0):
                    last_spawn_attempt = t_now
                    spawn_verified = ensure_all_drones_spawned(client_telemetry)
                    if not spawn_verified:
                        print(f"[WORKER] ⏳ 편대 스폰 미완료, 1초 후 재시도... (경과: {t_now - connect_started_at:.1f}초)", flush=True)

                sim_name = cached_sim_info["name"] if cached_sim_info else "AirSim 엔진"
                sim_icon = cached_sim_info["icon"] if cached_sim_info else "🎮"
                sim_id = cached_sim_info["id"] if cached_sim_info else "unknown"

                # Poll telemetry for all 4 Drones
                for d_id in ["Drone1", "Drone2", "Drone3", "Drone4"]:
                    v_name = get_real_vehicle_name(client_telemetry, d_id)
                    try:
                        state = client_telemetry.getMultirotorState(vehicle_name=v_name)
                        pos = state.kinematics_estimated.position
                        vel = state.kinematics_estimated.linear_velocity
                        orient = state.kinematics_estimated.orientation
                        pitch, roll, yaw = airsim.to_eularian_angles(orient)
                        speed = (vel.x_val**2 + vel.y_val**2 + vel.z_val**2) ** 0.5

                        is_api = client_telemetry.isApiControlEnabled(vehicle_name=v_name)
                        landed_str = "Flying" if state.landed_state != airsim.LandedState.Landed else "Landed"

                        latest_telemetries[d_id] = {
                            "drone_id": d_id,
                            "drone_name": DRONES_CONFIG[d_id]["name"],
                            "drone_role": DRONES_CONFIG[d_id]["role"],
                            "drone_tag": DRONES_CONFIG[d_id]["tag"],
                            "badge_color": DRONES_CONFIG[d_id]["color"],
                            "connected": True,
                            "simulated": False,
                            "active_sim_id": sim_id,
                            "active_sim_name": sim_name,
                            "active_sim_icon": sim_icon,
                            "armed": is_api or (landed_str == "Flying"),
                            "api_control": is_api,
                            "landed_state": landed_str,
                            "x": round(pos.x_val, 2),
                            "y": round(pos.y_val, 2),
                            "z": round(pos.z_val, 2),
                            "vx": round(vel.x_val, 2),
                            "vy": round(vel.y_val, 2),
                            "vz": round(vel.z_val, 2),
                            "speed": round(speed, 2),
                            "pitch": round(np.degrees(pitch), 1),
                            "roll": round(np.degrees(roll), 1),
                            "yaw": round(np.degrees(yaw), 1),
                            "lat": 47.641468,
                            "lon": -122.140165,
                            "alt": round(abs(pos.z_val), 2),
                            "battery": 98.0,
                            "status_msg": f"{sim_icon} {sim_name} 비행 중 ({landed_str})"
                        }
                        record_position_history(d_id, pos.x_val, pos.y_val, pos.z_val)

                        # Camera image for active selected drone (PNG compressed feed)
                        if d_id == selected_drone_id:
                            try:
                                t_cap = time.time()
                                responses = client_telemetry.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, True)], vehicle_name=v_name)
                                if responses and len(responses) > 0 and len(responses[0].image_data_uint8) > 0:
                                    raw_bytes = bytes(responses[0].image_data_uint8)
                                    # On a freshly-loaded (esp. geometrically heavy) map, the render
                                    # target can keep returning the same stale frame for a bit before
                                    # it starts actually rendering. Only advertise a "new" frame to the
                                    # frontend when the pixel data actually changed - otherwise the FPS
                                    # counter and UI falsely look "live" while showing a frozen image.
                                    if raw_bytes != last_frame_raw_bytes.get(d_id):
                                        last_frame_raw_bytes[d_id] = raw_bytes
                                        b64_str = base64.b64encode(raw_bytes).decode('utf-8')
                                        latest_frames_b64[d_id] = f"data:image/png;base64,{b64_str}"
                                        latest_frames_time[d_id] = t_cap
                                        stale_frame_streak[d_id] = 0
                                    else:
                                        stale_frame_streak[d_id] = stale_frame_streak.get(d_id, 0) + 1
                                        if stale_frame_streak[d_id] in (25, 125) or (stale_frame_streak[d_id] % 500 == 0):
                                            print(f"[WORKER] 📷 {d_id}({v_name}) 렌더 타깃이 {stale_frame_streak[d_id]}틱째 동일 프레임만 반환 중 (아직 렌더링 준비 안됐을 수 있음)", flush=True)
                                elif t_now - per_drone_error_logged_at.get(f"{d_id}_cam", 0.0) > 3.0:
                                    per_drone_error_logged_at[f"{d_id}_cam"] = t_now
                                    print(f"[WORKER] 📷 {d_id}({v_name}) simGetImages 응답은 왔지만 이미지 데이터가 비어있음 (responses={len(responses) if responses else 0})", flush=True)
                            except Exception as e_cam:
                                if t_now - per_drone_error_logged_at.get(f"{d_id}_cam", 0.0) > 3.0:
                                    per_drone_error_logged_at[f"{d_id}_cam"] = t_now
                                    print(f"[WORKER] ❌ {d_id}({v_name}) 카메라 캡처 실패: {type(e_cam).__name__}: {e_cam}", flush=True)

                    except Exception as e_drone:
                        # Throttled to once per 3s per drone - at 25Hz this would otherwise flood the console
                        if t_now - per_drone_error_logged_at.get(d_id, 0.0) > 3.0:
                            per_drone_error_logged_at[d_id] = t_now
                            print(f"[WORKER] ❌ {d_id}(vehicle_name={v_name}) 텔레메트리 폴링 실패: {type(e_drone).__name__}: {e_drone}", flush=True)

            except Exception as e:
                print(f"[WORKER ERROR] 텔레메트리 클라이언트 레벨 예외로 재연결: {type(e).__name__}: {e}", flush=True)
                close_airsim_client(client_telemetry)
                client_telemetry = None
                is_airsim_connected = False
                spawn_verified = False
                time.sleep(0.3)
        else:
            if is_airsim_connected or client_telemetry is not None:
                print("[WORKER] 🔌 RPC 포트 닫힘 감지, 텔레메트리 클라이언트 연결 해제", flush=True)
                close_airsim_client(client_telemetry)
                client_telemetry = None
                is_airsim_connected = False
                spawn_verified = False
            
            t = tick * 0.05
            active_name = cached_sim_info["name"] if cached_sim_info else "시뮬레이터 미실행"
            active_icon = cached_sim_info["icon"] if cached_sim_info else "🟡"
            active_id = cached_sim_info["id"] if cached_sim_info else None

            for d_id in ["Drone1", "Drone2", "Drone3", "Drone4"]:
                idx = DRONES_CONFIG[d_id]["order"]
                offset_phase = idx * 0.8
                latest_telemetries[d_id]["connected"] = False
                latest_telemetries[d_id]["simulated"] = True
                latest_telemetries[d_id]["active_sim_id"] = active_id
                latest_telemetries[d_id]["active_sim_name"] = active_name
                latest_telemetries[d_id]["active_sim_icon"] = active_icon
                latest_telemetries[d_id]["pitch"] = round(np.sin(t + offset_phase) * 3.0, 1)
                latest_telemetries[d_id]["roll"] = round(np.cos((t + offset_phase) * 1.5) * 2.0, 1)
                latest_telemetries[d_id]["yaw"] = round(((t * 5.0) + (idx * 45)) % 360, 1)

        time.sleep(0.04) # 25 FPS

worker_thread = threading.Thread(target=airsim_worker, daemon=True)
worker_thread.start()

def get_control_client():
    global client_control
    if client_control is None:
        print("[CONTROL] 🔌 신규 제어 클라이언트 생성 중...", flush=True)
        c = airsim.MultirotorClient(timeout_value=WORKER_RPC_TIMEOUT_SEC)
        c.confirmConnection()
        client_control = c
        print("[CONTROL] ✅ 신규 제어 클라이언트 연결 완료", flush=True)
    else:
        try:
            client_control.ping()
        except Exception as e_ping:
            print(f"[CONTROL] ⚠️ 기존 제어 클라이언트 ping 실패({type(e_ping).__name__}: {e_ping}), 재연결 시도...", flush=True)
            close_airsim_client(client_control)
            c = airsim.MultirotorClient(timeout_value=WORKER_RPC_TIMEOUT_SEC)
            c.confirmConnection()
            client_control = c
            print("[CONTROL] ✅ 제어 클라이언트 재연결 완료", flush=True)
    return client_control

def ensure_api_control(ctrl, vehicle_name: str):
    """
    Only call enableApiControl(True) when API control isn't already active.
    Confirmed via isolated testing (bypassing this app entirely): re-issuing
    enableApiControl(True) on a vehicle that is ALREADY airborne and under API
    control resets AirSim's landed_state flag to Landed even though it is still
    physically in the air. SimpleFlight then stops fighting gravity and the
    vehicle free-falls to the ground over ~2 seconds before it can recover -
    this was the actual cause of Alpha dropping and re-climbing every time
    "Formation Assemble" was pressed while already flying. armDisarm(True) on an
    already-armed vehicle was tested separately and does NOT have this effect,
    so it does not need the same guard.
    """
    if not ctrl.isApiControlEnabled(vehicle_name=vehicle_name):
        ctrl.enableApiControl(True, vehicle_name=vehicle_name)

# 🦆 Following Mode background autopilot: Bravo->Alpha, Charlie->Bravo, Delta->Charlie
def following_worker():
    print("[FOLLOW] 🦆 Following Mode 워커 스레드 시작", flush=True)
    while True:
        if not following_mode_enabled or not is_airsim_connected:
            time.sleep(0.2)
            continue
        try:
            with control_lock:
                ctrl = get_control_client()
                for follower_id, leader_id in FOLLOW_CHAIN.items():
                    target = get_lagged_leader_position(leader_id, following_lag_seconds)
                    if target is None:
                        continue
                    _, tx, ty, tz = target
                    f_vname = get_real_vehicle_name(ctrl, follower_id)
                    state = ctrl.getMultirotorState(vehicle_name=f_vname)
                    if state.landed_state == airsim.LandedState.Landed:
                        continue  # this duckling hasn't taken off yet - don't force it into the air
                    ensure_api_control(ctrl, f_vname)
                    ctrl.moveToPositionAsync(tx, ty, tz, following_velocity, vehicle_name=f_vname)
        except Exception as e:
            print(f"[FOLLOW] ⚠️ 오류: {type(e).__name__}: {e}", flush=True)
        time.sleep(FOLLOW_TICK_INTERVAL_SEC)

following_thread = threading.Thread(target=following_worker, daemon=True)
following_thread.start()

# Request Models
class SelectDroneRequest(BaseModel):
    drone_id: str

class MoveRequest(BaseModel):
    drone_id: str = None
    x: float
    y: float
    z: float
    velocity: float = 3.0

class JoystickVelocityRequest(BaseModel):
    drone_id: str = None
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    duration: float = 0.25

class DroneActionRequest(BaseModel):
    drone_id: str = None

class FormationAssembleRequest(BaseModel):
    spacing: float = 12.0  # Distance between drones in line (meters, 3x expanded)
    velocity: float = 4.0  # Cruise speed to join formation (m/s)

class LaunchSimulatorRequest(BaseModel):
    id: str
    resolution: str = "1280x720"

class FollowingModeRequest(BaseModel):
    enabled: bool
    lag_seconds: float = 2.0   # how many seconds "behind" each duckling trails its leader
    velocity: float = 3.0      # cruise speed followers use to catch up to their target

# =========================================================================
# Multi-Drone Selection & Status APIs
# =========================================================================
@app.get("/api/drones")
def get_drones_list():
    drones_list = []
    for d_id, cfg in DRONES_CONFIG.items():
        t = latest_telemetries.get(d_id, {})
        drones_list.append({
            "id": d_id,
            "name": cfg["name"],
            "role": cfg["role"],
            "tag": cfg["tag"],
            "badge_color": cfg["color"],
            "badge_class": cfg["badge_class"],
            "is_selected": (d_id == selected_drone_id),
            "landed_state": t.get("landed_state", "Unknown"),
            "altitude": round(abs(t.get("z", 0.0)), 1),
            "speed": t.get("speed", 0.0),
            "armed": t.get("armed", False),
            "api_control": t.get("api_control", False)
        })
    return {
        "selected_drone_id": selected_drone_id,
        "drones": drones_list,
        "connected": is_airsim_connected
    }

@app.post("/api/drones/select")
def select_drone(req: SelectDroneRequest):
    global selected_drone_id
    if req.drone_id not in DRONES_CONFIG:
        return {"status": "error", "message": f"유효하지 않은 드론 ID입니다: {req.drone_id}"}
    
    selected_drone_id = req.drone_id
    cfg = DRONES_CONFIG[selected_drone_id]
    snap = get_telemetry_snapshot(selected_drone_id)
    return {
        "status": "success",
        "selected_drone_id": selected_drone_id,
        "message": f"제어 대상 드론이 [{cfg['tag']} - {cfg['name']}] 으로 전환되었습니다. | {snap}"
    }

# =========================================================================
# 🎖️ Alpha Leader Call & Trail Formation Assembly System
# =========================================================================
def _do_formation_assemble(spacing: float = 12.0, velocity: float = 4.0):
    """
    Alpha (Drone 1) calls all wingmen (Drone 2 Bravo, Drone 3 Charlie, Drone 4 Delta).
    1. Check Alpha leader flight state:
       - If Alpha is airborne (z < -1.0 or landed_state == Flying), strictly preserve Alpha's current (X, Y, Z).
       - If Alpha is on ground, take off Alpha safely to 5m altitude.
    2. Command Alpha to lock position and maintain solid hover.
    3. All wingmen on ground take off in parallel and climb to Alpha's altitude at their own spot (동일고도 유지).
    4. Wingmen fly smoothly to their dedicated trail slots behind Alpha with 3x expanded spacing (12m interval).
    """
    with control_lock:
        ctrl = get_control_client()
        ensure_all_drones_spawned(ctrl)

        alpha_vname = get_real_vehicle_name(ctrl, "Drone1")
        ensure_api_control(ctrl, alpha_vname)
        ctrl.armDisarm(True, vehicle_name=alpha_vname)

        # 1. Check Alpha leader state
        s_alpha = ctrl.getMultirotorState(vehicle_name=alpha_vname)
        cur_z = s_alpha.kinematics_estimated.position.z_val
        cur_x = s_alpha.kinematics_estimated.position.x_val
        cur_y = s_alpha.kinematics_estimated.position.y_val
        _, _, alpha_yaw = airsim.to_eularian_angles(s_alpha.kinematics_estimated.orientation)

        is_alpha_on_ground = (s_alpha.landed_state == airsim.LandedState.Landed) or (cur_z > -1.0)
        print(f"[FORMATION] 🔍 Alpha 상태 판정: landed_state={s_alpha.landed_state} (Landed={airsim.LandedState.Landed}) | cur_z={cur_z:.2f} | is_alpha_on_ground={is_alpha_on_ground}", flush=True)

        if is_alpha_on_ground:
            print("[FORMATION] 🛫 Alpha 지상으로 판정 -> takeoffAsync() 호출", flush=True)
            ctrl.takeoffAsync(vehicle_name=alpha_vname).join()
            target_z = -5.0
            ctrl.moveToPositionAsync(cur_x, cur_y, target_z, 2.0, vehicle_name=alpha_vname).join()
            s_alpha = ctrl.getMultirotorState(vehicle_name=alpha_vname)
            alpha_pos = s_alpha.kinematics_estimated.position
        else:
            # Alpha is already airborne: Preserve exact current altitude and position!
            target_z = cur_z
            alpha_pos = s_alpha.kinematics_estimated.position
            print(f"[FORMATION] ✅ Alpha 비행 중으로 판정 -> 현재 위치/고도 유지 (target_z={target_z:.2f})", flush=True)

        # 2. Lock Alpha in autonomous hover at its position. hoverAsync() (AirSim's
        # dedicated "hold current position/altitude" command) instead of
        # moveToPositionAsync(current_x, current_y, current_z, ...): the real fix
        # for the ground-dive was guarding the redundant enableApiControl() call
        # above via ensure_api_control() (see its docstring), but hoverAsync is
        # still the more correct way to express "just hold position" than
        # commanding a move-to-position controller to a ~0-distance target.
        print(f"[FORMATION] 🔒 Alpha 호버 고정 명령: hoverAsync() (현재 위치 유지, target_z={target_z:.2f})", flush=True)
        alpha_yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=math.degrees(alpha_yaw))
        ctrl.hoverAsync(vehicle_name=alpha_vname)

        # 3. Take off all landed wingmen
        wingmen_ids = ["Drone2", "Drone3", "Drone4"]
        need_takeoff = False
        for w_id in wingmen_ids:
            w_vname = get_real_vehicle_name(ctrl, w_id)
            ensure_api_control(ctrl, w_vname)
            ctrl.armDisarm(True, vehicle_name=w_vname)

            s_w = ctrl.getMultirotorState(vehicle_name=w_vname)
            w_z = s_w.kinematics_estimated.position.z_val
            if (s_w.landed_state == airsim.LandedState.Landed) or (w_z > -1.0):
                ctrl.takeoffAsync(vehicle_name=w_vname)
                need_takeoff = True

        if need_takeoff:
            time.sleep(2.5)

        # 4. Stage 1: Climb each wingman vertically to target_z at its own current spot (동일고도 유지)
        for w_id in wingmen_ids:
            w_vname = get_real_vehicle_name(ctrl, w_id)
            s_w = ctrl.getMultirotorState(vehicle_name=w_vname)
            wx = s_w.kinematics_estimated.position.x_val
            wy = s_w.kinematics_estimated.position.y_val
            ctrl.moveToPositionAsync(wx, wy, target_z, 3.0, yaw_mode=alpha_yaw_mode, vehicle_name=w_vname)

        time.sleep(2.0)

        # 5. Stage 2: Transit wingmen to trailing formation slots behind Alpha (3x expanded X, Y spacing)
        back_dir_x = -math.cos(alpha_yaw)
        back_dir_y = -math.sin(alpha_yaw)

        for i, w_id in enumerate(wingmen_ids, start=1):
            w_vname = get_real_vehicle_name(ctrl, w_id)
            slot_dist = i * spacing
            tx = alpha_pos.x_val + (back_dir_x * slot_dist)
            ty = alpha_pos.y_val + (back_dir_y * slot_dist)

            yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=math.degrees(alpha_yaw))
            ctrl.moveToPositionAsync(tx, ty, target_z, velocity, yaw_mode=yaw_mode, vehicle_name=w_vname)

        # 6. Re-assert Alpha station-keeping hover (see note in step 2 - hoverAsync,
        # not moveToPositionAsync-to-self, to avoid the ground-dive trajectory bug)
        ctrl.hoverAsync(vehicle_name=alpha_vname)

        return target_z

@app.post("/api/formation/assemble")
async def formation_assemble(req: FormationAssembleRequest = None):
    spacing = req.spacing if req else 12.0
    vel = req.velocity if req else 4.0
    
    if is_airsim_connected:
        try:
            target_z = await asyncio.to_thread(_do_formation_assemble, spacing, vel)
            snap_a = get_telemetry_snapshot("Drone1")
            alt_str = f"{abs(target_z):.1f}m"
            return {
                "status": "success",
                "message": f"[편대장 ALPHA-01 집결 명령] 브라보, 찰리, 델타가 알파 고도({alt_str})로 일렬 종대(Trail) 포메이션 집결 비행을 시작합니다. | {snap_a}"
            }
        except Exception as e:
            return {"status": "error", "message": f"편대 집결 실패: {str(e)}"}

    # Demo Simulated Formation
    alpha_z = latest_telemetries["Drone1"]["z"] if latest_telemetries["Drone1"]["z"] < -1.0 else -5.0
    latest_telemetries["Drone1"]["z"] = alpha_z
    latest_telemetries["Drone1"]["landed_state"] = "Flying"
    latest_telemetries["Drone1"]["armed"] = True
    
    for i, w_id in enumerate(["Drone2", "Drone3", "Drone4"], start=1):
        latest_telemetries[w_id]["x"] = round(latest_telemetries["Drone1"]["x"] - (i * spacing), 2)
        latest_telemetries[w_id]["y"] = latest_telemetries["Drone1"]["y"]
        latest_telemetries[w_id]["z"] = alpha_z
        latest_telemetries[w_id]["landed_state"] = "Flying"
        latest_telemetries[w_id]["armed"] = True
        latest_telemetries[w_id]["yaw"] = latest_telemetries["Drone1"]["yaw"]

    return {
        "status": "simulated",
        "message": f"[가상 데모] 편대장 ALPHA 호출 -> 4대 일렬 종대 편대 집결 완료 (고도: {abs(alpha_z):.1f}m, 간격: {spacing}m)"
    }

# =========================================================================
# 🦆 Following Mode (Duckling Chain Autopilot) API
# =========================================================================
@app.post("/api/following/toggle")
def toggle_following_mode(req: FollowingModeRequest):
    global following_mode_enabled, following_lag_seconds, following_velocity, selected_drone_id
    following_mode_enabled = req.enabled
    following_lag_seconds = max(0.5, req.lag_seconds)
    following_velocity = max(0.5, req.velocity)

    if following_mode_enabled:
        # Alpha is the one you actively fly while the ducklings autopilot behind it
        selected_drone_id = "Drone1"
        if is_airsim_connected:
            return {
                "status": "success",
                "enabled": True,
                "message": f"Following Mode 시작. 브라보/찰리/델타가 {following_lag_seconds:.1f}초 지연으로 앞기체를 순차 추격합니다. 알파(편대장)를 조종해 출발하세요."
            }
        return {
            "status": "simulated",
            "enabled": True,
            "message": "[가상 데모] Following Mode 활성화됨 (실제 추격 비행은 3D 시뮬레이터 연결 후 동작합니다)"
        }

    return {
        "status": "success" if is_airsim_connected else "simulated",
        "enabled": False,
        "message": "Following Mode 해제 - 브라보/찰리/델타 개별 조종 가능"
    }

@app.get("/api/following/status")
def get_following_status():
    return {
        "enabled": following_mode_enabled,
        "lag_seconds": following_lag_seconds,
        "velocity": following_velocity,
        "chain": FOLLOW_CHAIN
    }

# =========================================================================
# Fleet All Takeoff & All Land APIs
# =========================================================================
def _do_fleet_takeoff():
    # NOTE: takeoffAsync()/moveToPositionAsync() return futures immediately -
    # calling .join() on each one INSIDE the loop (as this used to) blocks until
    # that single drone's maneuver fully finishes before the next drone's
    # takeoff is even dispatched, turning "동시 이륙" into a sequential
    # Alpha->Bravo->Charlie->Delta queue. Dispatch all 4 first, THEN join all 4,
    # so they actually fly at the same time.
    with control_lock:
        ctrl = get_control_client()
        ensure_all_drones_spawned(ctrl)

        v_names = []
        for d_id in ["Drone1", "Drone2", "Drone3", "Drone4"]:
            v_name = get_real_vehicle_name(ctrl, d_id)
            ensure_api_control(ctrl, v_name)
            ctrl.armDisarm(True, vehicle_name=v_name)
            v_names.append(v_name)

        takeoff_futures = [ctrl.takeoffAsync(vehicle_name=v) for v in v_names]
        for f in takeoff_futures:
            f.join()

        climb_futures = []
        for v_name in v_names:
            pos = ctrl.getMultirotorState(vehicle_name=v_name).kinematics_estimated.position
            climb_futures.append(ctrl.moveToPositionAsync(pos.x_val, pos.y_val, pos.z_val - 3.0, 2.0, vehicle_name=v_name))
        for f in climb_futures:
            f.join()
        return True

@app.post("/api/fleet/takeoff")
async def fleet_takeoff():
    if is_airsim_connected:
        try:
            await asyncio.to_thread(_do_fleet_takeoff)
            return {"status": "success", "message": "[전체 편대 동시 이륙] 알파/브라보/찰리/델타 4대 동시 수직 이륙 완료."}
        except Exception as e:
            return {"status": "error", "message": f"전체 이륙 실패: {str(e)}"}
    
    for d_id in ["Drone1", "Drone2", "Drone3", "Drone4"]:
        latest_telemetries[d_id]["armed"] = True
        latest_telemetries[d_id]["api_control"] = True
        latest_telemetries[d_id]["z"] = -3.0
        latest_telemetries[d_id]["landed_state"] = "Flying"
    return {"status": "simulated", "message": "[가상 데모] 4대 편대 전 기체 동시 수직 이륙 완료 (고도 3m)"}

def _do_fleet_land():
    # Same fix as _do_fleet_takeoff: dispatch all 4 landAsync() calls first,
    # THEN join all 4, instead of joining inside the loop (which landed them
    # one at a time, Alpha->Bravo->Charlie->Delta, instead of together).
    with control_lock:
        ctrl = get_control_client()
        v_names = [get_real_vehicle_name(ctrl, d_id) for d_id in ["Drone1", "Drone2", "Drone3", "Drone4"]]

        land_futures = []
        for v_name in v_names:
            try:
                land_futures.append((v_name, ctrl.landAsync(vehicle_name=v_name)))
            except Exception:
                pass

        for v_name, f in land_futures:
            try:
                f.join()
                ctrl.armDisarm(False, vehicle_name=v_name)
                ctrl.enableApiControl(False, vehicle_name=v_name)
            except Exception:
                pass
        return True

@app.post("/api/fleet/land")
async def fleet_land():
    if is_airsim_connected:
        try:
            await asyncio.to_thread(_do_fleet_land)
            return {"status": "success", "message": "[전체 편대 동시 착륙] 4대 기체 안전 착륙 완료."}
        except Exception as e:
            return {"status": "error", "message": f"전체 착륙 실패: {str(e)}"}
    
    for d_id in ["Drone1", "Drone2", "Drone3", "Drone4"]:
        latest_telemetries[d_id]["armed"] = False
        latest_telemetries[d_id]["api_control"] = False
        latest_telemetries[d_id]["z"] = 0.0
        latest_telemetries[d_id]["landed_state"] = "Landed"
    return {"status": "simulated", "message": "[가상 데모] 4대 편대 전 기체 안전 착륙 완료"}

@app.get("/api/simulators")
def get_simulators():
    active = detect_running_simulator()
    sim_list = []
    for sid, info in SIMULATORS.items():
        is_running = (active is not None and active["id"] == sid)
        abs_path = os.path.abspath(info["rel_path"])
        exists = os.path.exists(abs_path)
        sim_list.append({
            "id": sid,
            "name": info["name"],
            "icon": info["icon"],
            "desc": info["desc"],
            "is_running": is_running,
            "exists": exists
        })
    return {
        "active": active,
        "simulators": sim_list,
        "connected": is_airsim_connected
    }

@app.post("/api/simulators/launch")
def launch_simulator(req: LaunchSimulatorRequest):
    global client_control, client_telemetry, latest_frames_b64, is_switching_simulator
    sid = req.id.lower()
    if sid not in SIMULATORS:
        return {"status": "error", "message": f"존재하지 않는 시뮬레이터 모델 ID: {req.id}"}

    info = SIMULATORS[sid]
    abs_path = os.path.abspath(info["rel_path"])
    if not os.path.exists(abs_path):
        return {"status": "error", "message": f"실행 파일이 존재하지 않습니다: {abs_path}"}

    try:
        # 1. Fully kill previous simulator and clear client sockets
        t_kill_start = time.time()
        print(f"[LAUNCH] 🛑 '{sid}' 전환 요청 - 기존 시뮬레이터 종료 절차 시작...", flush=True)
        kill_all_simulators()
        print(f"[LAUNCH] 🛑 기존 시뮬레이터 종료 완료 ({time.time() - t_kill_start:.2f}초 소요)", flush=True)
        time.sleep(0.5)

        # 2. Parse resolution
        res_x = "1280"
        res_y = "720"
        if "x" in req.resolution:
            parts = req.resolution.split("x")
            if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
                res_x, res_y = parts[0], parts[1]

        exe_dir = os.path.dirname(abs_path)
        cmd = [abs_path, "-windowed", f"-ResX={res_x}", f"-ResY={res_y}", "-WinX=50", "-WinY=50"]
        flags = subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == 'win32' else 0
        proc = subprocess.Popen(cmd, cwd=exe_dir, creationflags=flags)
        print(f"[LAUNCH] 🚀 신규 프로세스 실행 (PID={proc.pid}): {abs_path} (cwd: {exe_dir})", flush=True)
        return {
            "status": "success",
            "message": f"[{info['name']}] 시뮬레이터 창 모드({res_x}x{res_y}) 실행 완료. 4대 편대 드론이 자동 연동됩니다.",
            "active_sim": {"id": sid, "name": info["name"], "icon": info["icon"]}
        }
    except Exception as e:
        return {"status": "error", "message": f"시뮬레이터 실행 실패: {str(e)}"}
    finally:
        # The new process has been spawned (or launch failed outright) - either
        # way it is now safe for the worker thread to resume polling for the
        # RPC port and reconnect. Always clear this, even on the exception path,
        # so a failed launch cannot permanently wedge the worker thread.
        is_switching_simulator = False
        print("[LAUNCH] 🔓 워커 스레드 재연결 재개 (is_switching_simulator=False)", flush=True)

@app.post("/api/simulators/stop")
def stop_all_simulators():
    global client_control, client_telemetry, latest_frames_b64, is_switching_simulator
    killed = kill_all_simulators()
    is_switching_simulator = False
    for d_id in latest_frames_b64:
        latest_frames_b64[d_id] = ""
    return {
        "status": "success",
        "message": f"시뮬레이터 프로세스 종료 완료 ({len(killed)}개 프로세스 정리됨)",
        "killed": killed
    }

@app.get("/api/status")
def get_status():
    active = detect_running_simulator()
    return {
        "connected": is_airsim_connected,
        "simulated": not is_airsim_connected,
        "active_sim": active,
        "selected_drone_id": selected_drone_id,
        "drones_config": DRONES_CONFIG,
        "snapshot": get_telemetry_snapshot(selected_drone_id),
        "following_mode_enabled": following_mode_enabled
    }

@app.post("/api/connect")
def connect_airsim():
    """
    Report the current connection state instead of opening a second, competing
    AirSim RPC client. The background airsim_worker() thread is the single owner
    of client_telemetry - creating another client here (as the old implementation
    did) raced with it on every map switch with no locking, overwriting
    client_telemetry from two threads at once and leaking the orphaned socket.
    That race was the root cause of "camera/control dead after switching maps".
    """
    active = detect_running_simulator()
    port_open = check_port_open()

    if not port_open:
        return {"status": "simulated", "message": f"3D 시뮬레이터 미실행 (포트 41451 닫힘) -> [가상 데모 모드] 동작 중 {get_telemetry_snapshot()}"}

    if is_airsim_connected:
        sim_desc = active['name'] if active else "AirSim 엔진"
        return {"status": "success", "message": f"실제 3D 시뮬레이터 ({sim_desc}) 4대 편대 연결 성공. {get_telemetry_snapshot()}"}

    return {"status": "connecting", "message": "AirSim 엔진 연결 및 편대 스폰 진행 중... (백그라운드 워커 스레드 처리 대기)"}

# =========================================================================
# Targeted Flight Commands (Applied to Selected Drone)
# =========================================================================
def _do_takeoff(target_drone_id: str):
    with control_lock:
        ctrl = get_control_client()
        ensure_all_drones_spawned(ctrl)
        v_name = get_real_vehicle_name(ctrl, target_drone_id)

        ensure_api_control(ctrl, v_name)
        ctrl.armDisarm(True, vehicle_name=v_name)
        ctrl.takeoffAsync(vehicle_name=v_name).join()
        
        state = ctrl.getMultirotorState(vehicle_name=v_name)
        pos = state.kinematics_estimated.position
        target_z = pos.z_val - 3.0
        ctrl.moveToPositionAsync(pos.x_val, pos.y_val, target_z, 2.0, vehicle_name=v_name).join()
        return True

@app.post("/api/takeoff")
async def takeoff(req: DroneActionRequest = None):
    t_id = (req.drone_id if req and req.drone_id else selected_drone_id)
    snap_before = get_telemetry_snapshot(t_id)
    tag = DRONES_CONFIG[t_id]["tag"]

    if is_follower_locked(t_id):
        return {"status": "error", "message": f"[{tag}] Following Mode 중에는 개별 이륙 명령을 보낼 수 없습니다. 먼저 Following Mode를 꺼주세요."}

    if is_airsim_connected:
        try:
            ok = await asyncio.to_thread(_do_takeoff, t_id)
            if ok:
                snap_after = get_telemetry_snapshot(t_id)
                return {"status": "success", "message": f"[{tag}] 제자리 수직 이륙 완료 (고도 +3m) | 이전: {snap_before} -> 현재: {snap_after}"}
        except Exception as e:
            return {"status": "error", "message": f"[{tag}] 이륙 실패: {str(e)} | 상태: {snap_before}"}
    
    latest_telemetries[t_id]["armed"] = True
    latest_telemetries[t_id]["api_control"] = True
    latest_telemetries[t_id]["z"] = -3.0
    latest_telemetries[t_id]["landed_state"] = "Flying"
    return {"status": "simulated", "message": f"[{tag} 데모] 제자리 수직 이륙 완료 (고도 3m) | {get_telemetry_snapshot(t_id)}"}

def _do_land(target_drone_id: str):
    with control_lock:
        ctrl = get_control_client()
        v_name = get_real_vehicle_name(ctrl, target_drone_id)
        ctrl.landAsync(vehicle_name=v_name).join()
        ctrl.armDisarm(False, vehicle_name=v_name)
        ctrl.enableApiControl(False, vehicle_name=v_name)
        return True

@app.post("/api/land")
async def land(req: DroneActionRequest = None):
    t_id = (req.drone_id if req and req.drone_id else selected_drone_id)
    snap_before = get_telemetry_snapshot(t_id)
    tag = DRONES_CONFIG[t_id]["tag"]
    
    if is_airsim_connected:
        try:
            ok = await asyncio.to_thread(_do_land, t_id)
            if ok:
                snap_after = get_telemetry_snapshot(t_id)
                return {"status": "success", "message": f"[{tag}] 드론 안전 착륙 완료 | 이전: {snap_before} -> 현재: {snap_after}"}
        except Exception as e:
            return {"status": "error", "message": f"[{tag}] 착륙 실패: {str(e)} | 상태: {snap_before}"}
    
    latest_telemetries[t_id]["z"] = 0.0
    latest_telemetries[t_id]["armed"] = False
    latest_telemetries[t_id]["api_control"] = False
    latest_telemetries[t_id]["landed_state"] = "Landed"
    return {"status": "simulated", "message": f"[{tag} 데모] 드론 착륙 완료 | {get_telemetry_snapshot(t_id)}"}

# 🏠 RTH (Return To Home for Target Drone)
def _do_rth(target_drone_id: str):
    with control_lock:
        ctrl = get_control_client()
        v_name = get_real_vehicle_name(ctrl, target_drone_id)
        ensure_api_control(ctrl, v_name)

        state = ctrl.getMultirotorState(vehicle_name=v_name)
        pos = state.kinematics_estimated.position
        home_x, home_y, _ = DRONES_CONFIG[target_drone_id]["spawn_offset"]
        
        # 1. Climb 15m higher than current position
        safe_climb_z = min(pos.z_val - 15.0, -15.0)
        ctrl.moveToPositionAsync(pos.x_val, pos.y_val, safe_climb_z, 3.0, vehicle_name=v_name).join()
        
        # 2. Cruise horizontally back to Home Origin for this drone
        ctrl.moveToPositionAsync(home_x, home_y, safe_climb_z, 6.0, vehicle_name=v_name).join()
        
        # 3. Slow descent to 3m above home point
        ctrl.moveToPositionAsync(home_x, home_y, -3.0, 2.0, vehicle_name=v_name).join()
        
        # 4. Safe Precision Landing
        ctrl.landAsync(vehicle_name=v_name).join()
        ctrl.armDisarm(False, vehicle_name=v_name)
        ctrl.enableApiControl(False, vehicle_name=v_name)
        return True

@app.post("/api/rth")
async def return_to_home(req: DroneActionRequest = None):
    t_id = (req.drone_id if req and req.drone_id else selected_drone_id)
    snap_before = get_telemetry_snapshot(t_id)
    tag = DRONES_CONFIG[t_id]["tag"]

    if is_follower_locked(t_id):
        return {"status": "error", "message": f"[{tag}] Following Mode 중에는 개별 RTH 명령을 보낼 수 없습니다. 먼저 Following Mode를 꺼주세요."}

    if is_airsim_connected:
        try:
            ok = await asyncio.to_thread(_do_rth, t_id)
            if ok:
                snap_after = get_telemetry_snapshot(t_id)
                return {"status": "success", "message": f"[{tag}] RTH 자동 복귀 및 안전 착륙 완료. | 이전: {snap_before} -> 현재: {snap_after}"}
        except Exception as e:
            return {"status": "error", "message": f"[{tag}] RTH 복귀 실패: {str(e)} | 상태: {snap_before}"}
    
    home_x, home_y, _ = DRONES_CONFIG[t_id]["spawn_offset"]
    latest_telemetries[t_id]["x"] = home_x
    latest_telemetries[t_id]["y"] = home_y
    latest_telemetries[t_id]["z"] = 0.0
    latest_telemetries[t_id]["armed"] = False
    latest_telemetries[t_id]["landed_state"] = "Landed"
    return {"status": "simulated", "message": f"[{tag} 데모] RTH 원점 복귀 및 착륙 완료 | {get_telemetry_snapshot(t_id)}"}

# Reset / Respawn All Drones to Starting Point
def _do_reset(target_drone_id: str):
    with control_lock:
        ctrl = get_control_client()
        ctrl.reset()
        ensure_all_drones_spawned(ctrl)
        v_name = get_real_vehicle_name(ctrl, target_drone_id)
        ensure_api_control(ctrl, v_name)
        ctrl.armDisarm(False, vehicle_name=v_name)
        return True

@app.post("/api/reset")
async def reset_drone(req: DroneActionRequest = None):
    t_id = (req.drone_id if req and req.drone_id else selected_drone_id)
    snap_before = get_telemetry_snapshot(t_id)
    tag = DRONES_CONFIG[t_id]["tag"]
    
    if is_airsim_connected:
        try:
            ok = await asyncio.to_thread(_do_reset, t_id)
            if ok:
                snap_after = get_telemetry_snapshot(t_id)
                return {"status": "success", "message": f"[{tag}] 시뮬레이션 및 4대 편대 위치 초기화 완료 | 이전: {snap_before} -> 현재: {snap_after}"}
        except Exception as e:
            return {"status": "error", "message": f"[{tag}] 초기화 실패: {str(e)} | 상태: {snap_before}"}
    
    for d in DRONES_CONFIG.keys():
        home_x, home_y, _ = DRONES_CONFIG[d]["spawn_offset"]
        latest_telemetries[d]["x"] = home_x
        latest_telemetries[d]["y"] = home_y
        latest_telemetries[d]["z"] = 0.0
        latest_telemetries[d]["vx"] = 0.0
        latest_telemetries[d]["vy"] = 0.0
        latest_telemetries[d]["vz"] = 0.0
        latest_telemetries[d]["speed"] = 0.0
        latest_telemetries[d]["pitch"] = 0.0
        latest_telemetries[d]["roll"] = 0.0
        latest_telemetries[d]["yaw"] = 0.0
        latest_telemetries[d]["armed"] = False
        latest_telemetries[d]["landed_state"] = "Landed"
    return {"status": "simulated", "message": f"[{tag} 데모] 4대 편대 위치 초기화 완료 | {get_telemetry_snapshot(t_id)}"}

def _do_rotate(target_drone_id: str):
    with control_lock:
        ctrl = get_control_client()
        v_name = get_real_vehicle_name(ctrl, target_drone_id)
        ensure_api_control(ctrl, v_name)
        ctrl.rotateByYawRateAsync(36.0, 10.0, vehicle_name=v_name).join()
        return True

@app.post("/api/rotate")
async def rotate(req: DroneActionRequest = None):
    t_id = (req.drone_id if req and req.drone_id else selected_drone_id)
    snap_before = get_telemetry_snapshot(t_id)
    tag = DRONES_CONFIG[t_id]["tag"]

    if is_follower_locked(t_id):
        return {"status": "error", "message": f"[{tag}] Following Mode 중에는 개별 회전 명령을 보낼 수 없습니다. 먼저 Following Mode를 꺼주세요."}

    if is_airsim_connected:
        try:
            ok = await asyncio.to_thread(_do_rotate, t_id)
            if ok:
                snap_after = get_telemetry_snapshot(t_id)
                return {"status": "success", "message": f"[{tag}] 360도 회전 스캔 비행 완료 | 현재: {snap_after}"}
        except Exception as e:
            return {"status": "error", "message": f"[{tag}] 회전 실패: {str(e)} | 상태: {snap_before}"}
    
    return {"status": "simulated", "message": f"[{tag} 데모] 360도 스캔 회전 완료 | {get_telemetry_snapshot(t_id)}"}

def _do_joystick_velocity(target_drone_id, vx, vy, vz, yaw_rate, duration):
    with control_lock:
        ctrl = get_control_client()
        v_name = get_real_vehicle_name(ctrl, target_drone_id)
        ensure_api_control(ctrl, v_name)
        if abs(vx) < 0.05 and abs(vy) < 0.05 and abs(vz) < 0.05 and abs(yaw_rate) < 0.05:
            ctrl.moveByVelocityBodyFrameAsync(0, 0, 0, duration, vehicle_name=v_name)
        else:
            yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate)
            ctrl.moveByVelocityBodyFrameAsync(vx, vy, vz, duration, airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode, vehicle_name=v_name)
        return True

@app.post("/api/joystick")
async def joystick_control(req: JoystickVelocityRequest):
    t_id = req.drone_id if req.drone_id else selected_drone_id
    tag = DRONES_CONFIG[t_id]["tag"]

    if is_follower_locked(t_id):
        return {"status": "error", "drone_id": t_id, "message": f"[{tag}] Following Mode 중에는 개별 조이스틱 조종을 할 수 없습니다."}

    if is_airsim_connected:
        try:
            await asyncio.to_thread(_do_joystick_velocity, t_id, req.vx, req.vy, req.vz, req.yaw_rate, req.duration)
            return {"status": "success", "drone_id": t_id, "message": f"[{tag}] 속도제어: Vx={req.vx:.1f}, Vy={req.vy:.1f}, Vz={req.vz:.1f}, Yaw={req.yaw_rate:.1f}°/s"}
        except Exception as e:
            return {"status": "error", "drone_id": t_id, "message": f"[{tag}] 조이스틱 제어 오류: {str(e)}"}
    
    latest_telemetries[t_id]["vx"] = req.vx
    latest_telemetries[t_id]["vy"] = req.vy
    latest_telemetries[t_id]["vz"] = req.vz
    latest_telemetries[t_id]["z"] = round(latest_telemetries[t_id]["z"] + req.vz * 0.1, 2)
    latest_telemetries[t_id]["x"] = round(latest_telemetries[t_id]["x"] + req.vx * 0.1, 2)
    latest_telemetries[t_id]["y"] = round(latest_telemetries[t_id]["y"] + req.vy * 0.1, 2)
    latest_telemetries[t_id]["yaw"] = round((latest_telemetries[t_id]["yaw"] + req.yaw_rate * 0.1) % 360, 1)
    return {"status": "simulated", "drone_id": t_id, "message": f"[{tag}] 가상 조이스틱 제어 적용됨"}

def _do_capture(target_drone_id, filepath):
    with control_lock:
        ctrl = get_control_client()
        v_name = get_real_vehicle_name(ctrl, target_drone_id)
        responses = ctrl.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name=v_name)
        if responses and len(responses) > 0 and len(responses[0].image_data_uint8) > 0:
            img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
            cv2.imwrite(filepath, img_rgb)
            return True
    return False

@app.post("/api/capture")
async def capture_photo(req: DroneActionRequest = None):
    t_id = (req.drone_id if req and req.drone_id else selected_drone_id)
    tag = DRONES_CONFIG[t_id]["tag"]
    filename = f"{t_id}_capture_{int(time.time())}.png"
    filepath = os.path.join(os.getcwd(), filename)
    snap = get_telemetry_snapshot(t_id)
    
    if is_airsim_connected:
        try:
            ok = await asyncio.to_thread(_do_capture, t_id, filepath)
            if ok:
                return {"status": "success", "filename": filename, "message": f"[{tag}] 고해상도 사진 저장 완료: {filename} | {snap}"}
        except Exception as e:
            return {"status": "error", "message": f"[{tag}] 사진 캡처 실패: {str(e)} | {snap}"}
    
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, f"AIRSIM [{tag}] FPV CAMERA FEED (DEMO)", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.circle(img, (320, 240), 40, (0, 255, 255), 2)
    cv2.imwrite(filepath, img)
    return {"status": "simulated", "filename": filename, "message": f"[{tag} 데모] 샘플 사진 저장 완료: {filename} | {snap}"}

# WebSocket for Zero-Latency 4-UAV Fleet Telemetry & Active Camera Stream
@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    last_sent_frame_time = 0.0
    try:
        while True:
            t_now = time.time()
            active_drone = selected_drone_id
            active_frame = latest_frames_b64.get(active_drone, "")
            active_frame_time = latest_frames_time.get(active_drone, 0.0)

            payload = {
                "selected_drone_id": active_drone,
                "telemetry": latest_telemetries.get(active_drone, latest_telemetries["Drone1"]),
                "all_telemetries": latest_telemetries,
                "frame": active_frame if (active_frame_time != last_sent_frame_time) else "",
                "capture_time": active_frame_time,
                "server_time": t_now,
                "following_mode_enabled": following_mode_enabled
            }
            last_sent_frame_time = active_frame_time
            await websocket.send_json(payload)
            await asyncio.sleep(0.04) # 25Hz sync rate
    except WebSocketDisconnect:
        pass
    except Exception:
        pass

# Mount static frontend
app.mount("/", StaticFiles(directory="public", html=True), name="public")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)
