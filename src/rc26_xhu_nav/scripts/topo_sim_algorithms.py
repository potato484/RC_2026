#!/usr/bin/env python3
"""Helper algorithms for the rc26_xhu_nav 3D simulation server."""

from __future__ import annotations

import math
import random
from typing import Any


def pose_point(pose: dict[str, Any]) -> dict[str, float]:
    return {
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "z": float(pose.get("z", 0.0)),
        "yaw": float(pose.get("yaw", 0.0)),
    }


def node_lookup(document: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(node["id"]): node for node in document.get("nodes", [])}


def edge_lookup(document: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(edge["id"]): edge for edge in document.get("edges", [])}


def polyline_from_node_ids(document: dict[str, Any], node_ids: list[str]) -> list[dict[str, float]]:
    nodes = node_lookup(document)
    points: list[dict[str, float]] = []
    for node_id in node_ids:
        node = nodes.get(str(node_id))
        if node is None:
            continue
        points.append(pose_point(node.get("pose", {})))
    return points


def polyline_from_edge_ids(
    document: dict[str, Any],
    edge_ids: list[str],
    *,
    fallback_node_path: list[str] | None = None,
) -> list[dict[str, float]]:
    nodes = node_lookup(document)
    edges = edge_lookup(document)
    if not edge_ids:
        return polyline_from_node_ids(document, fallback_node_path or [])

    points: list[dict[str, float]] = []
    first_edge = edges.get(str(edge_ids[0]))
    if first_edge is None:
        return polyline_from_node_ids(document, fallback_node_path or [])

    start_node = nodes.get(str(first_edge.get("from", "")))
    if start_node is not None:
        points.append(pose_point(start_node.get("pose", {})))

    for edge_id in edge_ids:
        edge = edges.get(str(edge_id))
        if edge is None:
            continue
        for control_point in edge.get("control_points", []):
            points.append(pose_point(control_point))
        end_node = nodes.get(str(edge.get("to", "")))
        if end_node is not None:
            points.append(pose_point(end_node.get("pose", {})))
    return points


def build_world_bounds(
    document: dict[str, Any],
    scene_features: list[dict[str, Any]],
    margin: float = 0.8,
) -> dict[str, float]:
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    for feature in scene_features:
        for point in feature.get("points", []):
            xs.append(float(point["x"]))
            ys.append(float(point["y"]))
            zs.append(float(point.get("z", 0.0)))
    for node in document.get("nodes", []):
        pose = node.get("pose", {})
        xs.append(float(pose.get("x", 0.0)))
        ys.append(float(pose.get("y", 0.0)))
        zs.append(float(pose.get("z", 0.0)))
    if not xs or not ys:
        return {"min_x": -5.0, "max_x": 5.0, "min_y": -5.0, "max_y": 5.0, "min_z": 0.0, "max_z": 1.5}
    return {
        "min_x": min(xs) - margin,
        "max_x": max(xs) + margin,
        "min_y": min(ys) - margin,
        "max_y": max(ys) + margin,
        "min_z": min(zs) - 0.1,
        "max_z": max(zs) + 0.8,
    }


def build_obstacle_rects(
    scene_features: list[dict[str, Any]],
    *,
    blocked_points: list[dict[str, float]] | None = None,
    block_radius: float = 0.34,
) -> list[dict[str, float]]:
    rects: list[dict[str, float]] = []
    for feature in scene_features:
        render_class = str(feature.get("render_class", ""))
        area_xy = float(feature.get("area_xy", 0.0))
        z_span = float(feature.get("z_span", 0.0))
        # Full 3D meshes are rendered in the viewer, but only vertical barriers
        # should become 2D planning keep-outs. Horizontal platform surfaces are
        # drivable support geometry and would incorrectly cover start/goal slots.
        #
        # The viewer may now preserve stair risers and platform side walls as
        # near-zero XY structural surfaces for rendering depth. They should not
        # collapse into 2D keep-outs unless they are explicit fence geometry.
        if render_class != "world-fence" and area_xy < 1e-4 and z_span >= 0.18:
            continue
        is_planning_barrier = render_class == "world-fence" or z_span >= 0.18
        if not is_planning_barrier:
            continue
        points = feature.get("points", [])
        if len(points) < 3:
            continue
        xs = [float(point["x"]) for point in points]
        ys = [float(point["y"]) for point in points]
        zs = [float(point.get("z", 0.0)) for point in points]
        rects.append(
            {
                "id": str(feature.get("id", "")),
                "min_x": min(xs) - 0.03,
                "max_x": max(xs) + 0.03,
                "min_y": min(ys) - 0.03,
                "max_y": max(ys) + 0.03,
                "min_z": min(zs) - 0.05,
                "max_z": max(zs) + 0.25,
            }
        )

    for index, point in enumerate(blocked_points or []):
        rects.append(
            {
                "id": f"blocked-{index}",
                "min_x": float(point["x"]) - block_radius,
                "max_x": float(point["x"]) + block_radius,
                "min_y": float(point["y"]) - block_radius,
                "max_y": float(point["y"]) + block_radius,
                "min_z": float(point.get("z", 0.0)) - 0.1,
                "max_z": float(point.get("z", 0.0)) + 0.8,
            }
        )
    return rects


def point_in_rect(point: dict[str, float], rect: dict[str, float]) -> bool:
    return (
        rect["min_x"] <= float(point["x"]) <= rect["max_x"]
        and rect["min_y"] <= float(point["y"]) <= rect["max_y"]
    )


def segment_hits_rect(start: dict[str, float], end: dict[str, float], rect: dict[str, float]) -> bool:
    distance = math.hypot(float(end["x"]) - float(start["x"]), float(end["y"]) - float(start["y"]))
    steps = max(2, int(distance / 0.08) + 1)
    for step in range(steps + 1):
        ratio = step / steps
        sample = {
            "x": float(start["x"]) + (float(end["x"]) - float(start["x"])) * ratio,
            "y": float(start["y"]) + (float(end["y"]) - float(start["y"])) * ratio,
        }
        if point_in_rect(sample, rect):
            return True
    return False


def segment_hits_any_rect(start: dict[str, float], end: dict[str, float], rects: list[dict[str, float]]) -> bool:
    return any(segment_hits_rect(start, end, rect) for rect in rects)


def distance_xy(left: dict[str, float], right: dict[str, float]) -> float:
    return math.hypot(float(left["x"]) - float(right["x"]), float(left["y"]) - float(right["y"]))


def steer_towards(start: dict[str, float], target: dict[str, float], max_step: float) -> dict[str, float]:
    dx = float(target["x"]) - float(start["x"])
    dy = float(target["y"]) - float(start["y"])
    distance = math.hypot(dx, dy)
    if distance <= max_step:
        return {"x": float(target["x"]), "y": float(target["y"]), "z": float(target.get("z", start.get("z", 0.0))), "yaw": 0.0}
    ratio = max_step / max(distance, 1e-6)
    return {
        "x": float(start["x"]) + dx * ratio,
        "y": float(start["y"]) + dy * ratio,
        "z": float(start.get("z", 0.0)),
        "yaw": math.atan2(dy, dx),
    }


def reconstruct_tree_path(nodes: list[dict[str, Any]], goal_index: int) -> list[dict[str, float]]:
    path: list[dict[str, float]] = []
    cursor = goal_index
    while cursor >= 0:
        path.append(nodes[cursor]["point"])
        cursor = int(nodes[cursor]["parent"])
    path.reverse()
    return path


def generate_rrt_run(
    document: dict[str, Any],
    start_pose: dict[str, float],
    goal_pose: dict[str, float],
    scene_features: list[dict[str, Any]],
    *,
    blocked_points: list[dict[str, float]] | None = None,
    seed: int = 7,
    max_iterations: int = 220,
    step_size: float = 0.55,
    goal_bias: float = 0.18,
) -> dict[str, Any]:
    bounds = build_world_bounds(document, scene_features)
    rects = build_obstacle_rects(scene_features, blocked_points=blocked_points)
    rng = random.Random(seed)

    nodes: list[dict[str, Any]] = [{"point": start_pose, "parent": -1}]
    tree_segments: list[dict[str, Any]] = []
    frames: list[dict[str, Any]] = []
    success = False
    goal_index = -1

    for iteration in range(max_iterations):
        if rng.random() < goal_bias:
            sample = goal_pose
        else:
            sample = {
                "x": rng.uniform(bounds["min_x"], bounds["max_x"]),
                "y": rng.uniform(bounds["min_y"], bounds["max_y"]),
                "z": start_pose["z"],
                "yaw": 0.0,
            }

        nearest_index = min(range(len(nodes)), key=lambda idx: distance_xy(nodes[idx]["point"], sample))
        new_point = steer_towards(nodes[nearest_index]["point"], sample, step_size)
        if any(point_in_rect(new_point, rect) for rect in rects):
            continue
        if segment_hits_any_rect(nodes[nearest_index]["point"], new_point, rects):
            continue

        nodes.append({"point": new_point, "parent": nearest_index})
        new_index = len(nodes) - 1
        tree_segments.append({"from": nodes[nearest_index]["point"], "to": new_point})
        current_path = reconstruct_tree_path(nodes, new_index)

        if distance_xy(new_point, goal_pose) <= step_size and not segment_hits_any_rect(new_point, goal_pose, rects):
            nodes.append({"point": goal_pose, "parent": new_index})
            goal_index = len(nodes) - 1
            tree_segments.append({"from": new_point, "to": goal_pose})
            current_path = reconstruct_tree_path(nodes, goal_index)
            success = True

        frames.append(
            {
                "stepIndex": len(frames),
                "algorithm": "rrt",
                "phase": "goal_reached" if success else "tree_expand",
                "label": "RRT tree expanded",
                "robotPose": current_path[-1],
                "openSet": [],
                "expandedNodes": [],
                "bestPath": {"nodeIds": [], "points": current_path},
                "treeSegments": tree_segments.copy(),
                "candidateTrajectories": [],
                "selectedTrajectory": [],
                "metrics": {
                    "iteration": iteration + 1,
                    "tree_size": len(nodes),
                    "goal_distance": round(distance_xy(current_path[-1], goal_pose), 4),
                },
            }
        )
        if success:
            break

    final_path = reconstruct_tree_path(nodes, goal_index) if success and goal_index >= 0 else []
    return {
        "success": success,
        "algorithm": "rrt",
        "summary": {
            "iterations": len(frames),
            "tree_size": len(nodes),
            "goal_distance": round(distance_xy(final_path[-1], goal_pose), 4) if final_path else None,
        },
        "path_points": final_path,
        "frames": frames,
        "obstacles": rects,
    }


def simulate_trajectory(
    start_pose: dict[str, float],
    velocity: tuple[float, float, float],
    *,
    horizon_s: float,
    dt: float,
) -> list[dict[str, float]]:
    vx, vy, wz = velocity
    pose = dict(start_pose)
    trajectory = [dict(pose)]
    steps = max(1, int(horizon_s / dt))
    for _ in range(steps):
        pose = {
            "x": float(pose["x"]) + vx * dt,
            "y": float(pose["y"]) + vy * dt,
            "z": float(pose.get("z", 0.0)),
            "yaw": float(pose.get("yaw", 0.0)) + wz * dt,
        }
        trajectory.append(pose)
    return trajectory


def trajectory_collides(trajectory: list[dict[str, float]], rects: list[dict[str, float]]) -> bool:
    for left, right in zip(trajectory, trajectory[1:]):
        if segment_hits_any_rect(left, right, rects):
            return True
    return False


def trajectory_clearance(trajectory: list[dict[str, float]], rects: list[dict[str, float]]) -> float:
    if not rects:
        return 99.0
    clearance = 99.0
    for pose in trajectory:
        for rect in rects:
            dx = max(rect["min_x"] - pose["x"], 0.0, pose["x"] - rect["max_x"])
            dy = max(rect["min_y"] - pose["y"], 0.0, pose["y"] - rect["max_y"])
            clearance = min(clearance, math.hypot(dx, dy))
    return clearance


def choose_reference_target(reference_path: list[dict[str, float]], pose: dict[str, float], lookahead: int = 2) -> dict[str, float]:
    if not reference_path:
        return pose
    nearest_index = min(range(len(reference_path)), key=lambda idx: distance_xy(reference_path[idx], pose))
    return reference_path[min(len(reference_path) - 1, nearest_index + lookahead)]


def generate_dwa_run(
    document: dict[str, Any],
    start_pose: dict[str, float],
    goal_pose: dict[str, float],
    scene_features: list[dict[str, Any]],
    *,
    reference_path: list[dict[str, float]] | None = None,
    blocked_points: list[dict[str, float]] | None = None,
    max_steps: int = 18,
) -> dict[str, Any]:
    rects = build_obstacle_rects(scene_features, blocked_points=blocked_points)
    reference = reference_path or [start_pose, goal_pose]
    frames: list[dict[str, Any]] = []
    current_pose = dict(start_pose)
    traversed_points = [dict(start_pose)]
    success = False

    vx_samples = (-0.55, 0.0, 0.55)
    vy_samples = (-0.55, 0.0, 0.55)
    wz_samples = (-0.65, 0.0, 0.65)

    for step in range(max_steps):
        target = choose_reference_target(reference, current_pose)
        candidates: list[dict[str, Any]] = []
        best_candidate: dict[str, Any] | None = None
        best_score = -1e9
        for vx in vx_samples:
            for vy in vy_samples:
                for wz in wz_samples:
                    trajectory = simulate_trajectory(current_pose, (vx, vy, wz), horizon_s=1.2, dt=0.3)
                    collision = trajectory_collides(trajectory, rects)
                    clearance = trajectory_clearance(trajectory, rects)
                    end_pose = trajectory[-1]
                    distance_score = -distance_xy(end_pose, target) * 2.4
                    goal_score = -distance_xy(end_pose, goal_pose)
                    speed_score = math.hypot(vx, vy) * 0.7
                    clearance_score = min(clearance, 1.5) * 0.9
                    turn_penalty = abs(wz) * 0.15
                    score = distance_score + goal_score + speed_score + clearance_score - turn_penalty
                    if collision:
                        score -= 8.0
                    candidate = {
                        "velocity": {"vx": vx, "vy": vy, "wz": wz},
                        "points": trajectory,
                        "score": round(score, 4),
                        "collision": collision,
                        "clearance": round(clearance, 4),
                        "selected": False,
                    }
                    candidates.append(candidate)
                    if score > best_score:
                        best_score = score
                        best_candidate = candidate

        if best_candidate is None:
            break

        best_candidate["selected"] = True
        selected_trajectory = best_candidate["points"]
        current_pose = selected_trajectory[-1]
        traversed_points.extend(selected_trajectory[1:])

        frames.append(
            {
                "stepIndex": len(frames),
                "algorithm": "dwa",
                "phase": "tracking",
                "label": "Holonomic DWA selected a control window trajectory",
                "robotPose": current_pose,
                "openSet": [],
                "expandedNodes": [],
                "bestPath": {"nodeIds": [], "points": traversed_points.copy()},
                "treeSegments": [],
                "candidateTrajectories": candidates,
                "selectedTrajectory": selected_trajectory,
                "metrics": {
                    "step": step + 1,
                    "target_distance": round(distance_xy(current_pose, target), 4),
                    "goal_distance": round(distance_xy(current_pose, goal_pose), 4),
                    "candidate_count": len(candidates),
                },
            }
        )

        if distance_xy(current_pose, goal_pose) < 0.32:
            success = True
            frames[-1]["phase"] = "goal_reached"
            frames[-1]["label"] = "Holonomic DWA reached the goal window"
            break

    return {
        "success": success,
        "algorithm": "dwa",
        "summary": {
            "steps": len(frames),
            "goal_distance": round(distance_xy(current_pose, goal_pose), 4),
        },
        "path_points": traversed_points,
        "frames": frames,
        "obstacles": rects,
    }
