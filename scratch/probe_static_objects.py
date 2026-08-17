import airsim
import json
import time
import urllib.request
import os

SERVER_BASE_URL = "http://127.0.0.1:8000"

def api_post(endpoint: str, payload: dict = None) -> dict:
    url = f"{SERVER_BASE_URL}{endpoint}"
    data = json.dumps(payload or {}).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=60) as res:
        return json.loads(res.read().decode('utf-8'))

def probe_map(sim_id: str):
    print(f"\n==========================================")
    print(f"Probing map: {sim_id}")
    print(f"==========================================")
    api_post("/api/simulators/launch", {"id": sim_id, "resolution": "1280x720"})
    time.sleep(3.5)

    c = airsim.MultirotorClient(timeout_value=10)
    c.confirmConnection()

    objs = c.simListSceneObjects('.*')
    print(f"Total objects in {sim_id}: {len(objs)}")

    # Sample objects and poses
    poses = {}
    for o in objs:
        try:
            p = c.simGetObjectPose(o)
            pos = p.position
            # Keep objects near fleet area (within 150m)
            if abs(pos.x_val) < 150.0 and abs(pos.y_val) < 150.0:
                poses[o] = (round(pos.x_val, 2), round(pos.y_val, 2), round(pos.z_val, 2))
        except Exception:
            pass

    print(f"Objects near origin (<150m): {len(poses)}")
    for name, pos in list(poses.items())[:25]:
        print(f"  {name}: {pos}")

    os.makedirs("scratch", exist_ok=True)
    with open(f"scratch/probe_{sim_id}_objects.json", "w", encoding="utf-8") as f:
        json.dump({"total_objects": len(objs), "near_origin_count": len(poses), "poses": poses, "sample_names": objs[:100]}, f, indent=2)

    api_post("/api/simulators/stop")
    time.sleep(1.5)

if __name__ == "__main__":
    probe_map("blocks")
    probe_map("park")
