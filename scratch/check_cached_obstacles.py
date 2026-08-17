import airsim

c = airsim.MultirotorClient()
c.confirmConnection()

all_objs = c.simListSceneObjects('.*')
exclude_keywords = (
    'camera', 'ground', 'asphalt', 'light', 'sky', 'particle',
    'trigger', 'cine', 'player', 'postprocess', 'fog', 'volume', 'terrain'
)

filtered_names = []
for name in all_objs:
    name_lower = name.lower()
    if any(k in name_lower for k in exclude_keywords):
        continue
    filtered_names.append(name)

obstacles = []
for name in filtered_names:
    try:
        p = c.simGetObjectPose(name)
        pos = p.position
        if abs(pos.x_val) <= 150.0 and abs(pos.y_val) <= 150.0:
            dist_to_origin = (pos.x_val**2 + pos.y_val**2)**0.5
            obstacles.append((dist_to_origin, name, (pos.x_val, pos.y_val, pos.z_val)))
    except Exception:
        pass

obstacles.sort(key=lambda x: x[0])
print(f"Total filtered obstacles in Blocks: {len(obstacles)}")
print("Closest 10 obstacles to origin (0,0):")
for dist, name, pos in obstacles[:15]:
    print(f"  dist={dist:.2f}m | {name}: pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
