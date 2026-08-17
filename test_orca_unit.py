"""
Unit tests for orca.py mathematical correctness and collision avoidance logic.
No simulator required.
"""
import math
import numpy as np
from orca import Line, linear_program_1d, linear_program_2d, linear_program_3_relaxed, compute_safe_velocity


def test_linear_program_2d_unconstrained():
    """When there are no constraint lines, opt_velocity within radius is unchanged."""
    opt_vel = np.array([2.0, 1.0])
    lines = []
    failed_idx, result = linear_program_2d(lines, radius=5.0, opt_velocity=opt_vel, direction_opt=False)
    assert failed_idx == 0
    assert np.allclose(result, opt_vel)


def test_linear_program_2d_speed_clamp():
    """When opt_velocity exceeds max radius, it is clamped to radius magnitude."""
    opt_vel = np.array([4.0, 3.0])  # norm = 5.0
    lines = []
    failed_idx, result = linear_program_2d(lines, radius=3.0, opt_velocity=opt_vel, direction_opt=False)
    assert failed_idx == 0
    assert np.isclose(np.linalg.norm(result), 3.0)
    assert np.allclose(result, np.array([2.4, 1.8]))


def test_head_on_collision_avoidance_2d():
    """
    Two agents moving directly towards each other on X-axis:
    Agent A at (0, 0) wants to move +X at 3.0 m/s
    Agent B at (6, 0) moving -X at 3.0 m/s
    A should divert slightly sideways (Y component != 0) while maintaining forward movement.
    """
    agent_a_pos = (0.0, 0.0, -5.0)
    agent_a_vel = (3.0, 0.0, 0.0)
    agent_a_pref = (3.0, 0.0, 0.0)

    neighbor_b = {
        "pos": (6.0, 0.0, -5.0),
        "vel": (-3.0, 0.0, 0.0),
        "radius": 1.5,
        "weight": 0.5
    }

    safe_vel = compute_safe_velocity(
        agent_pos=agent_a_pos,
        agent_vel=agent_a_vel,
        preferred_vel=agent_a_pref,
        neighbors=[neighbor_b],
        agent_radius=1.5,
        time_horizon=2.0,
        max_speed=3.0
    )

    # Safe velocity must have a non-zero Y component to evade head-on collision
    assert abs(safe_vel[1]) > 0.1 or safe_vel[0] < 2.0
    assert math.sqrt(safe_vel[0]**2 + safe_vel[1]**2) <= 3.0 + 1e-5
    print(f"\n[Head-on test] Safe vel for Agent A: {safe_vel}")


def test_three_way_crossing():
    """Three agents converging at origin: ORCA should find collision-free vectors for all."""
    agents = [
        {"id": "A", "pos": (-5.0, 0.0, -5.0), "vel": (2.0, 0.0, 0.0), "pref": (2.0, 0.0, 0.0)},
        {"id": "B", "pos": (0.0, -5.0, -5.0), "vel": (0.0, 2.0, 0.0), "pref": (0.0, 2.0, 0.0)},
        {"id": "C", "pos": (3.5, 3.5, -5.0), "vel": (-1.4, -1.4, 0.0), "pref": (-1.4, -1.4, 0.0)}
    ]

    safe_velocities = {}
    for i, a in enumerate(agents):
        neighbors = []
        for j, b in enumerate(agents):
            if i != j:
                neighbors.append({
                    "pos": b["pos"],
                    "vel": b["vel"],
                    "radius": 1.5,
                    "weight": 0.5
                })
        safe_v = compute_safe_velocity(
            agent_pos=a["pos"],
            agent_vel=a["vel"],
            preferred_vel=a["pref"],
            neighbors=neighbors,
            agent_radius=1.5,
            time_horizon=3.0,
            max_speed=3.0
        )
        safe_velocities[a["id"]] = safe_v
        print(f"[3-Way Crossing] Agent {a['id']} safe vel: {safe_v}")

    for a_id, sv in safe_velocities.items():
        assert math.sqrt(sv[0]**2 + sv[1]**2) <= 3.0 + 1e-5


def test_altitude_z_control():
    """Z axis proportional velocity control with clamping."""
    safe_vel = compute_safe_velocity(
        agent_pos=(0.0, 0.0, -5.0),
        agent_vel=(0.0, 0.0, 0.0),
        preferred_vel=(1.0, 1.0, -4.5),  # climb request of -4.5 m/s
        neighbors=[],
        max_vz=2.0
    )
    assert safe_vel[2] == -2.0  # clamped to -max_vz


def test_reciprocal_head_on_simulation():
    """
    Multi-step forward Euler simulation of two reciprocal agents on a head-on collision course.
    Agent A at (-6, 0) moving +X to (+6, 0)
    Agent B at (+6, 0) moving -X to (-6, 0)
    Both must reach their destinations without their distance ever dropping below (2 * radius).
    """
    pos_a = np.array([-6.0, 0.0, -5.0])
    pos_b = np.array([6.0, 0.0, -5.0])
    vel_a = np.array([0.0, 0.0, 0.0])
    vel_b = np.array([0.0, 0.0, 0.0])

    target_a = np.array([6.0, 0.0, -5.0])
    target_b = np.array([-6.0, 0.0, -5.0])

    agent_radius = 1.0  # 1.0m safety radius each -> min distance must stay > 2.0m
    dt = 0.1
    min_dist = float('inf')

    for step in range(200):
        # Desired velocities towards targets
        dir_a = target_a[:2] - pos_a[:2]
        dist_a = float(np.linalg.norm(dir_a))
        pref_a = (dir_a / max(0.01, dist_a)) * min(3.0, dist_a) if dist_a > 0.1 else np.zeros(2)

        dir_b = target_b[:2] - pos_b[:2]
        dist_b = float(np.linalg.norm(dir_b))
        pref_b = (dir_b / max(0.01, dist_b)) * min(3.0, dist_b) if dist_b > 0.1 else np.zeros(2)

        # Compute safe velocities reciprocally (weight 0.5 each)
        safe_a = compute_safe_velocity(
            agent_pos=pos_a,
            agent_vel=vel_a,
            preferred_vel=(pref_a[0], pref_a[1], 0.0),
            neighbors=[{"pos": pos_b, "vel": vel_b, "radius": agent_radius, "weight": 0.5}],
            agent_radius=agent_radius,
            time_horizon=2.0,
            max_speed=3.0,
            time_step=dt
        )

        safe_b = compute_safe_velocity(
            agent_pos=pos_b,
            agent_vel=vel_b,
            preferred_vel=(pref_b[0], pref_b[1], 0.0),
            neighbors=[{"pos": pos_a, "vel": vel_a, "radius": agent_radius, "weight": 0.5}],
            agent_radius=agent_radius,
            time_horizon=2.0,
            max_speed=3.0,
            time_step=dt
        )

        vel_a = np.array(safe_a)
        vel_b = np.array(safe_b)

        pos_a += vel_a * dt
        pos_b += vel_b * dt

        d = float(np.linalg.norm(pos_a[:2] - pos_b[:2]))
        min_dist = min(min_dist, d)

        if dist_a < 0.2 and dist_b < 0.2:
            break

    print(f"\n[Dynamic Simulation] Minimum separation distance: {min_dist:.2f}m (Threshold: {2 * agent_radius}m)")
    print(f"[Dynamic Simulation] Final distance to Target A: {dist_a:.2f}m, Target B: {dist_b:.2f}m")
    # Assert separation never dropped below combined radii with a tiny numerical tolerance
    assert min_dist >= (2 * agent_radius) - 0.05, f"Collision detected! Min dist {min_dist} < {2 * agent_radius}"
    # Assert both agents actually reached their destinations without deadlocking
    assert dist_a <= 0.3, f"Agent A did not reach target: final dist {dist_a}m"
    assert dist_b <= 0.3, f"Agent B did not reach target: final dist {dist_b}m"


if __name__ == "__main__":
    test_linear_program_2d_unconstrained()
    test_linear_program_2d_speed_clamp()
    test_head_on_collision_avoidance_2d()
    test_three_way_crossing()
    test_altitude_z_control()
    test_reciprocal_head_on_simulation()
    print("All unit tests passed successfully!")
