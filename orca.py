"""
Pure Python + NumPy implementation of 2D Optimal Reciprocal Collision Avoidance (ORCA).

Based on:
J. van den Berg, S. J. Guy, M. Lin, D. Manocha,
"Reciprocal n-Body Collision Avoidance", Robotics Research (2011).

This module operates in a 2D plane (X, Y) with an independent proportional
Z-axis (altitude) velocity controller.
"""

from typing import NamedTuple, Sequence
import numpy as np


class Line(NamedTuple):
    """
    A directed line representing the boundary of a half-plane constraint.
    Points on or to the 'left' of the directed line (or where dot((v - point), normal) >= 0)
    are permitted.
    """
    point: np.ndarray      # shape (2,)
    direction: np.ndarray  # shape (2,), unit vector


EPSILON = 1e-7


def _det2d(a: np.ndarray, b: np.ndarray) -> float:
    """2D determinant (cross product in 2D): a[0]*b[1] - a[1]*b[0]."""
    return float(a[0] * b[1] - a[1] * b[0])


def linear_program_1d(
    lines: Sequence[Line],
    line_no: int,
    radius: float,
    opt_velocity: np.ndarray,
    direction_opt: bool
) -> tuple[bool, np.ndarray]:
    """
    Solves a 1D linear program along line line_no subject to:
    - distance from origin <= radius
    - satisfying all previous line constraints (index < line_no)
    """
    line = lines[line_no]
    dot_prod = float(np.dot(line.point, line.direction))
    discriminant = dot_prod ** 2 + radius ** 2 - float(np.dot(line.point, line.point))

    if discriminant < 0.0:
        # Max speed circle does not intersect line
        return False, np.zeros(2)

    sqrt_disc = float(np.sqrt(max(0.0, discriminant)))
    t_left = -dot_prod - sqrt_disc
    t_right = -dot_prod + sqrt_disc

    for i in range(line_no):
        other = lines[i]
        denom = _det2d(line.direction, other.direction)
        numer = _det2d(other.direction, line.point - other.point)

        if abs(denom) <= EPSILON:
            # Lines are nearly parallel
            if numer < 0.0:
                # Fully cut off
                return False, np.zeros(2)
            continue

        t = numer / denom
        if denom > 0.0:
            # Line i bounds the right side
            t_right = min(t_right, t)
        else:
            # Line i bounds the left side
            t_left = max(t_left, t)

        if t_left > t_right:
            return False, np.zeros(2)

    if direction_opt:
        # Optimize in the direction of opt_velocity
        if float(np.dot(opt_velocity, line.direction)) > 0.0:
            result = line.point + t_right * line.direction
        else:
            result = line.point + t_left * line.direction
    else:
        # Optimize closest point to opt_velocity
        t = float(np.dot(line.direction, opt_velocity - line.point))
        t = np.clip(t, t_left, t_right)
        result = line.point + t * line.direction

    return True, result


def linear_program_2d(
    lines: Sequence[Line],
    radius: float,
    opt_velocity: np.ndarray,
    direction_opt: bool
) -> tuple[int, np.ndarray]:
    """
    Solves a 2D linear program to find a velocity closest to opt_velocity
    within radius satisfying all half-plane constraints.

    Returns:
        (failed_line_index, result_velocity)
        If successful, failed_line_index == len(lines).
    """
    if direction_opt:
        # Optimize in direction of opt_velocity with magnitude radius
        opt_len = float(np.linalg.norm(opt_velocity))
        if opt_len > EPSILON:
            result = (opt_velocity / opt_len) * radius
        else:
            result = np.zeros(2)
    else:
        if float(np.dot(opt_velocity, opt_velocity)) > radius ** 2:
            result = (opt_velocity / float(np.linalg.norm(opt_velocity))) * radius
        else:
            result = opt_velocity.copy()

    for i, line in enumerate(lines):
        # Check if current result violates line constraint
        # Allowed if det(line.direction, line.point - result) <= 0
        if _det2d(line.direction, line.point - result) > 0.0:
            # Point violates constraint, solve 1D LP on this line
            success, result_1d = linear_program_1d(lines, i, radius, opt_velocity, direction_opt)
            if not success:
                return i, result
            result = result_1d

    return len(lines), result


def linear_program_3_relaxed(
    lines: Sequence[Line],
    num_obstacles: int,
    begin_line: int,
    radius: float,
    result: np.ndarray
) -> np.ndarray:
    """
    Stage 3 2D Relaxation Fallback LP:
    When the set of half-plane constraints is over-constrained (empty feasible region),
    this relaxes the constraints outwards uniformly to find the velocity that minimizes
    maximum constraint penetration within the speed radius.
    """
    distance = 0.0
    proj_lines: list[Line] = []

    for i in range(begin_line, len(lines)):
        line_i = lines[i]
        if _det2d(line_i.direction, line_i.point - result) > distance:
            # Result penetrates this line by more than distance
            proj_lines = list(lines[:num_obstacles])

            for j in range(num_obstacles, i):
                line_j = lines[j]
                denom = _det2d(line_i.direction, line_j.direction)
                if abs(denom) <= EPSILON:
                    # Parallel lines
                    if float(np.dot(line_i.direction, line_j.direction)) > 0.0:
                        # Same direction
                        continue
                    else:
                        point = 0.5 * (line_i.point + line_j.point)
                else:
                    point = line_i.point + (_det2d(line_j.direction, line_i.point - line_j.point) / denom) * line_i.direction

                diff = line_j.direction - line_i.direction
                diff_len = float(np.linalg.norm(diff))
                if diff_len > EPSILON:
                    direction = diff / diff_len
                    proj_lines.append(Line(point=point, direction=direction))

            failed_idx, new_res = linear_program_2d(proj_lines, radius, np.array([-line_i.direction[1], line_i.direction[0]]), True)
            if failed_idx == len(proj_lines):
                result = new_res
            distance = _det2d(line_i.direction, line_i.point - result)

    return result


def compute_orca_halfplanes_2d(
    agent_pos_2d: np.ndarray,
    agent_vel_2d: np.ndarray,
    neighbors: Sequence[dict],
    agent_radius: float,
    time_horizon: float,
    time_step: float = 0.3
) -> list[Line]:
    """
    Computes ORCA half-plane constraints for an agent against all neighbors in 2D.
    """
    orca_lines: list[Line] = []
    inv_time_horizon = 1.0 / max(EPSILON, time_horizon)
    inv_time_step = 1.0 / max(EPSILON, time_step)

    for neighbor in neighbors:
        n_pos_2d = np.array(neighbor["pos"][:2], dtype=float)
        n_vel_2d = np.array(neighbor.get("vel", (0.0, 0.0))[:2], dtype=float)
        n_radius = float(neighbor.get("radius", agent_radius))
        is_static = float(neighbor.get("weight", 0.5)) >= 0.9
        cur_inv_th = (1.0 / 0.8) if is_static else inv_time_horizon

        rel_pos = n_pos_2d - agent_pos_2d
        rel_vel = agent_vel_2d - n_vel_2d
        dist_sq = float(np.dot(rel_pos, rel_pos))
        combined_radius = agent_radius + n_radius
        combined_radius_sq = combined_radius ** 2

        if dist_sq > combined_radius_sq:
            # No collision yet
            w = rel_vel - cur_inv_th * rel_pos
            w_len_sq = float(np.dot(w, w))
            dot_prod_1 = float(np.dot(w, rel_pos))

            if not is_static and dot_prod_1 < 0.0 and dot_prod_1 ** 2 > combined_radius_sq * w_len_sq:
                # Project on cutoff circle (reciprocal agents only)
                w_len = float(np.sqrt(max(0.0, w_len_sq)))
                if w_len > EPSILON:
                    unit_w = w / w_len
                    direction = np.array([unit_w[1], -unit_w[0]])
                    u = (combined_radius * cur_inv_th - w_len) * unit_w
                else:
                    direction = np.array([0.0, 1.0])
                    u = np.zeros(2)
            else:
                # Project on legs (sides of VO cone)
                dist = float(np.sqrt(max(0.0, dist_sq)))
                if dist > EPSILON:
                    leg = float(np.sqrt(max(0.0, dist_sq - combined_radius_sq)))
                    if _det2d(rel_pos, w) > 0.0:
                        # Left leg
                        dir_x = (rel_pos[0] * leg - rel_pos[1] * combined_radius) / dist_sq
                        dir_y = (rel_pos[1] * leg + rel_pos[0] * combined_radius) / dist_sq
                    else:
                        # Right leg
                        dir_x = -(rel_pos[0] * leg + rel_pos[1] * combined_radius) / dist_sq
                        dir_y = -(rel_pos[1] * leg - rel_pos[0] * combined_radius) / dist_sq
                    direction = np.array([dir_x, dir_y])
                    u = float(np.dot(rel_vel, direction)) * direction - rel_vel
                else:
                    direction = np.array([0.0, 1.0])
                    u = np.zeros(2)
        else:
            # Already colliding or overlapping: emergency avoidance using time_step
            w = rel_vel - inv_time_step * rel_pos
            w_len = float(np.linalg.norm(w))
            if w_len > EPSILON:
                unit_w = w / w_len
                direction = np.array([unit_w[1], -unit_w[0]])
                u = (combined_radius * inv_time_step - w_len) * unit_w
            else:
                direction = np.array([0.0, 1.0])
                u = np.zeros(2)

        # 50:50 reciprocal responsibility: point = agent_vel + 0.5 * u
        # (If neighbor is an obstacle with weight 1.0, agent takes 1.0 * u)
        weight = float(neighbor.get("weight", 0.5))
        line_point = agent_vel_2d + weight * u
        orca_lines.append(Line(point=line_point, direction=direction))

    return orca_lines


def compute_safe_velocity(
    agent_pos: tuple[float, float, float] | Sequence[float],
    agent_vel: tuple[float, float, float] | Sequence[float],
    preferred_vel: tuple[float, float, float] | Sequence[float],
    neighbors: list[dict],
    agent_radius: float = 1.5,
    time_horizon: float = 2.0,
    max_speed: float = 3.0,
    max_vz: float = 2.0,
    time_step: float = 0.3
) -> tuple[float, float, float]:
    """
    Computes a collision-free velocity vector for the agent.

    Args:
        agent_pos: Current position (x, y, z) in NED frame.
        agent_vel: Current velocity (vx, vy, vz) in NED frame.
        preferred_vel: Target desired velocity (pref_vx, pref_vy, pref_vz).
        neighbors: List of neighbor dicts:
                   [{"pos": (x,y,z), "vel": (vx,vy,vz), "radius": float, "weight": float}, ...]
        agent_radius: Safety radius around this agent (meters).
        time_horizon: Forward prediction time horizon (seconds).
        max_speed: Maximum allowable horizontal speed (m/s).
        max_vz: Maximum vertical climbing/descending speed (m/s).
        time_step: Control loop time step (seconds).

    Returns:
        Safe velocity tuple (safe_vx, safe_vy, safe_vz) in NED frame.
    """
    pos_2d = np.array(agent_pos[:2], dtype=float)
    vel_2d = np.array(agent_vel[:2], dtype=float)
    pref_2d = np.array(preferred_vel[:2], dtype=float)

    # 1. Compute 2D ORCA Halfplanes
    orca_lines = compute_orca_halfplanes_2d(
        agent_pos_2d=pos_2d,
        agent_vel_2d=vel_2d,
        neighbors=neighbors,
        agent_radius=agent_radius,
        time_horizon=time_horizon,
        time_step=time_step
    )

    # 2. Add proactive bypass bias for static obstacles directly in the path
    opt_2d = pref_2d.copy()
    if len(neighbors) > 0 and float(np.linalg.norm(pref_2d)) > EPSILON:
        pref_speed = float(np.linalg.norm(pref_2d))
        unit_pref = pref_2d / pref_speed
        
        # Check if there is a static obstacle directly ahead (dot product > 0.5 within 12m)
        has_obstacle_ahead = False
        for n in neighbors:
            if float(n.get("weight", 0.5)) >= 0.9:  # Static obstacle (weight=1.0)
                rel = np.array(n["pos"][:2], dtype=float) - pos_2d
                d_rel = float(np.linalg.norm(rel))
                if 0.5 < d_rel < 12.0:
                    unit_rel = rel / d_rel
                    if float(np.dot(unit_pref, unit_rel)) > 0.4:  # Obstacle is in forward cone
                        has_obstacle_ahead = True
                        break

        if has_obstacle_ahead:
            # Strong lateral component (+X direction) to actively navigate around the obstacle
            bypass_vec = np.array([-unit_pref[1], unit_pref[0]]) * (pref_speed * 0.8)
            opt_2d = pref_2d + bypass_vec
        else:
            right_bias = np.array([-unit_pref[1], unit_pref[0]]) * (pref_speed * 0.15)
            opt_2d = pref_2d + right_bias

    # 3. Solve 2D Linear Program
    failed_line, result_2d = linear_program_2d(
        lines=orca_lines,
        radius=max_speed,
        opt_velocity=opt_2d,
        direction_opt=False
    )

    # 3. If over-constrained, apply Stage 3 2D Relaxation Fallback
    if failed_line < len(orca_lines):
        result_2d = linear_program_3_relaxed(
            lines=orca_lines,
            num_obstacles=0,
            begin_line=failed_line,
            radius=max_speed,
            result=result_2d
        )

    # 4. Z-axis independent proportional velocity (clipped)
    pref_vz = float(preferred_vel[2]) if len(preferred_vel) > 2 else 0.0
    safe_vz = float(np.clip(pref_vz, -max_vz, max_vz))

    return (float(result_2d[0]), float(result_2d[1]), safe_vz)
