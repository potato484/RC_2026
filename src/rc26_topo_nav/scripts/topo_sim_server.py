#!/usr/bin/env python3
"""FastAPI + WebSocket adapter for the rc26_topo_nav 3D simulation viewer."""

from __future__ import annotations

import argparse
import asyncio
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Literal

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

try:
    from ament_index_python.packages import (  # type: ignore
        PackageNotFoundError,
        get_package_prefix,
        get_package_share_directory,
    )
except ImportError:  # pragma: no cover - optional in source-tree runs
    PackageNotFoundError = None
    get_package_prefix = None
    get_package_share_directory = None


SCRIPT_DIR = Path(__file__).resolve().parent


def detect_source_pkg_root() -> Path | None:
    candidate = SCRIPT_DIR.parent
    if (candidate / "scripts").is_dir() and (candidate / "config").is_dir():
        return candidate
    return None


def detect_share_root() -> Path | None:
    if get_package_share_directory is None or PackageNotFoundError is None:
        return None
    try:
        return Path(get_package_share_directory("rc26_topo_nav"))
    except PackageNotFoundError:
        return None


def detect_package_prefix() -> Path | None:
    if get_package_prefix is None or PackageNotFoundError is None:
        return None
    try:
        return Path(get_package_prefix("rc26_topo_nav"))
    except PackageNotFoundError:
        return None


SOURCE_PKG_ROOT = detect_source_pkg_root()
SHARE_ROOT = detect_share_root()
PACKAGE_PREFIX = detect_package_prefix()
PKG_ROOT = SOURCE_PKG_ROOT or SHARE_ROOT or SCRIPT_DIR.parent
WORKSPACE_ROOT = SOURCE_PKG_ROOT.parents[1] if SOURCE_PKG_ROOT is not None else None


def preferred_package_path(*parts: str) -> Path:
    candidates: list[Path] = []
    if SOURCE_PKG_ROOT is not None:
        candidates.append(SOURCE_PKG_ROOT.joinpath(*parts))
    if SHARE_ROOT is not None:
        candidates.append(SHARE_ROOT.joinpath(*parts))
    if not candidates:
        return PKG_ROOT.joinpath(*parts)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def resolve_frontend_root() -> Path:
    candidates: list[Path] = []
    if SOURCE_PKG_ROOT is not None:
        candidates.append(SOURCE_PKG_ROOT / "sim_viewer")
    if SHARE_ROOT is not None:
        candidates.append(SHARE_ROOT / "sim_viewer")
    if not candidates:
        return PKG_ROOT / "sim_viewer"
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    return candidates[0]


FRONTEND_ROOT = resolve_frontend_root()
FRONTEND_DIST = FRONTEND_ROOT / "dist"

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from topo_sim_algorithms import (  # noqa: E402
    build_obstacle_rects,
    build_world_bounds,
    edge_lookup,
    generate_dwa_run,
    generate_rrt_run,
    node_lookup,
    polyline_from_edge_ids,
    polyline_from_node_ids,
    pose_point,
)


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


RENDER = load_module("render_graph_sim_html", SCRIPT_DIR / "render_graph_sim_html.py")


def default_graph_for_team(team: str) -> Path:
    return preferred_package_path(
        "config",
        "r2_field_graph_red.yaml" if team.lower() == "red" else "r2_field_graph_blue.yaml",
    )


def default_world_file() -> Path:
    return preferred_package_path("sim_assets", "worlds", "robocon2026_v2_aligned.world")


def default_kfs_config_file() -> Path:
    return preferred_package_path("sim_assets", "config", "kfs_config_v2_aligned.yaml")


def round_float(value: float, digits: int = 4) -> float:
    rounded = round(float(value), digits)
    if abs(rounded) < 10 ** (-digits):
        return 0.0
    return rounded


def actual_world_z(pose: dict[str, Any]) -> float:
    return round_float(float(pose.get("z", 0.0)) + float(pose.get("world_anchor_z") or 0.0), 4)


def sim_pose_from_graph_pose(pose: dict[str, Any], *, block_id: int, alignment: dict[str, Any] | None) -> dict[str, float]:
    if alignment is None:
        pose_data = pose_point(pose)
        pose_data["world_anchor_z"] = None
        pose_data["world_z"] = pose_data["z"]
        return pose_data

    sim_pose = RENDER.map_graph_pose_to_sim(pose, block_id=block_id, alignment=alignment)
    return {
        "x": float(sim_pose["x"]),
        "y": float(sim_pose["y"]),
        "z": float(sim_pose["z"]),
        "world_anchor_z": sim_pose.get("world_anchor_z"),
        "world_z": actual_world_z(sim_pose),
        "yaw": float(pose.get("yaw", 0.0)),
    }


def parse_world_lights(world_path: Path) -> dict[str, Any]:
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        return {"ambient": [0.2, 0.2, 0.2, 1.0], "background": [0.4, 0.4, 0.4, 1.0], "lights": []}

    scene = world.find("scene")
    ambient = [0.2, 0.2, 0.2, 1.0]
    background = [0.4, 0.4, 0.4, 1.0]
    if scene is not None:
        ambient_text = scene.findtext("ambient", default="0.2 0.2 0.2 1")
        background_text = scene.findtext("background", default="0.4 0.4 0.4 1")
        ambient = [float(part) for part in ambient_text.split()]
        background = [float(part) for part in background_text.split()]

    lights = []
    for light in world.findall("light"):
        lights.append(
            {
                "name": light.get("name", ""),
                "type": light.get("type", "directional"),
                "pose": RENDER.parse_pose_text(light.findtext("pose")),
                "direction": [float(part) for part in light.findtext("direction", default="0 0 -1").split()],
                "diffuse": [float(part) for part in light.findtext("diffuse", default="255 255 255 1").split()],
                "cast_shadows": light.findtext("cast_shadows", default="1") == "1",
            }
        )
    return {"ambient": ambient, "background": background, "lights": lights}


def build_camera_presets(bounds: dict[str, float]) -> list[dict[str, Any]]:
    center_x = (bounds["min_x"] + bounds["max_x"]) * 0.5
    center_y = (bounds["min_y"] + bounds["max_y"]) * 0.5
    span = max(bounds["max_x"] - bounds["min_x"], bounds["max_y"] - bounds["min_y"], 1.0)
    return [
        {
            "id": "orbit",
            "kind": "perspective",
            "position": {"x": center_x + span * 0.75, "y": center_y - span * 0.9, "z": bounds["max_z"] + span * 0.55},
            "target": {"x": center_x, "y": center_y, "z": max(0.3, bounds["max_z"] * 0.45)},
        },
        {
            "id": "follow",
            "kind": "perspective",
            "position": {"x": center_x - span * 0.55, "y": center_y - span * 0.7, "z": bounds["max_z"] + span * 0.28},
            "target": {"x": center_x, "y": center_y, "z": max(0.25, bounds["max_z"] * 0.35)},
        },
        {
            "id": "first_person",
            "kind": "perspective",
            "position": {"x": center_x, "y": bounds["min_y"] - span * 0.22, "z": 0.85},
            "target": {"x": center_x, "y": center_y, "z": 0.65},
        },
        {
            "id": "top_ortho",
            "kind": "orthographic",
            "position": {"x": center_x, "y": center_y, "z": bounds["max_z"] + span},
            "target": {"x": center_x, "y": center_y, "z": 0.0},
        },
        {
            "id": "side_ortho",
            "kind": "orthographic",
            "position": {"x": bounds["max_x"] + span * 0.65, "y": center_y, "z": bounds["max_z"] + span * 0.18},
            "target": {"x": center_x, "y": center_y, "z": max(0.25, bounds["max_z"] * 0.35)},
        },
        {
            "id": "side_perspective",
            "kind": "perspective",
            "position": {
                "x": bounds["max_x"] + span * 0.34,
                "y": center_y - span * 0.62,
                "z": bounds["max_z"] + span * 0.12,
            },
            "target": {"x": center_x, "y": center_y, "z": max(0.45, bounds["max_z"] * 0.42)},
        },
    ]


def default_start_goal(document: dict[str, Any]) -> tuple[str, str]:
    nodes = node_lookup(document)
    start = "ramp_entry_south" if "ramp_entry_south" in nodes else next(iter(nodes.keys()))
    goal = "ramp_exit_north" if "ramp_exit_north" in nodes else list(nodes.keys())[-1]
    return start, goal


def build_scene_manifest(
    *,
    team: str,
    graph_file: Path,
    world_file: Path,
    kfs_config_file: Path,
    include_full_geometry: bool = True,
) -> dict[str, Any]:
    document = RENDER.load_yaml(graph_file)
    sim_config = RENDER.load_yaml(kfs_config_file)
    alignment = RENDER.derive_graph_alignment(document, sim_config, team)
    world_context = RENDER.parse_world_context(
        world_file,
        world_file.parent.parent / "models",
        filter_noise=not include_full_geometry,
    )
    scene_features = world_context["scene_features"]
    meilin_slots = RENDER.build_meilin_slots(sim_config, team)
    nodes = node_lookup(document)

    graph_nodes = []
    for node in document.get("nodes", []):
        pose = sim_pose_from_graph_pose(node.get("pose", {}), block_id=int(node.get("block_id", 0)), alignment=alignment)
        graph_nodes.append(
            {
                "id": str(node["id"]),
                "type": str(node.get("type", "")),
                "block_id": int(node.get("block_id", 0)),
                "base_cost": float(node.get("base_cost", 0.0)),
                "operation_tag": str(node.get("operation_tag", "")),
                "pose": pose,
            }
        )

    graph_edges = []
    for edge in document.get("edges", []):
        from_node = nodes[str(edge["from"])]
        to_node = nodes[str(edge["to"])]
        points = [sim_pose_from_graph_pose(from_node.get("pose", {}), block_id=int(from_node.get("block_id", 0)), alignment=alignment)]
        for control_point in edge.get("control_points", []):
            points.append(sim_pose_from_graph_pose(control_point, block_id=0, alignment=alignment))
        points.append(sim_pose_from_graph_pose(to_node.get("pose", {}), block_id=int(to_node.get("block_id", 0)), alignment=alignment))
        graph_edges.append(
            {
                "id": str(edge["id"]),
                "from": str(edge["from"]),
                "to": str(edge["to"]),
                "motion_type": str(edge.get("motion_type", "")),
                "height_change": float(edge.get("height_change", 0.0)),
                "required_mode": str(edge.get("required_mode", "")),
                "base_cost": float(edge.get("base_cost", 0.0)),
                "points": points,
            }
        )

    bounds = build_world_bounds(document, scene_features)
    default_start, default_goal = default_start_goal(document)

    return {
        "meta": {
            "team": team,
            "graph_file": str(graph_file),
            "world_file": str(world_file),
            "kfs_config_file": str(kfs_config_file),
            "full_geometry": include_full_geometry,
        },
        "alignment": alignment,
        "bounds": bounds,
        "lights": parse_world_lights(world_file),
        "sceneFeatures": scene_features,
        "graphNodes": graph_nodes,
        "graphEdges": graph_edges,
        "tasks": document.get("tasks", []),
        "routes": document.get("routes", []),
        "meilinSlots": meilin_slots,
        "cameraPresets": build_camera_presets(bounds),
        "defaults": {"startNode": default_start, "goalNode": default_goal},
    }


def blocked_sim_points(manifest: dict[str, Any], blocked_nodes: list[str]) -> list[dict[str, float]]:
    node_map = {item["id"]: item for item in manifest["graphNodes"]}
    return [node_map[node_id]["pose"] for node_id in blocked_nodes if node_id in node_map]


def resolve_cli_binary() -> Path:
    candidates: list[Path] = []
    if PACKAGE_PREFIX is not None:
        candidates.append(PACKAGE_PREFIX / "lib" / "rc26_topo_nav" / "planner_trace_cli")
    if WORKSPACE_ROOT is not None:
        candidates.extend(
            [
                WORKSPACE_ROOT / "install" / "rc26_topo_nav" / "lib" / "rc26_topo_nav" / "planner_trace_cli",
                WORKSPACE_ROOT / "build" / "rc26_topo_nav" / "planner_trace_cli",
            ]
        )
    which_path = shutil.which("planner_trace_cli")
    if which_path:
        candidates.append(Path(which_path))

    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("planner_trace_cli not found; build rc26_topo_nav first")


def planner_cli_env() -> dict[str, str]:
    env = os.environ.copy()
    runtime_dirs: list[Path] = []
    if PACKAGE_PREFIX is not None:
        runtime_dirs.append(PACKAGE_PREFIX / "lib")
    if WORKSPACE_ROOT is not None:
        runtime_dirs.extend(sorted(path for path in (WORKSPACE_ROOT / "install").glob("*/lib") if path.is_dir()))
        runtime_dirs.append(WORKSPACE_ROOT / "build" / "rc26_topo_nav")
    existing = env.get("LD_LIBRARY_PATH", "")
    merged = [str(path) for path in runtime_dirs if path.is_dir()]
    if existing:
        merged.append(existing)
    if merged:
        env["LD_LIBRARY_PATH"] = os.pathsep.join(merged)
    return env


def run_planner_trace_cli(
    request: "PlannerRunRequest",
    manifest: dict[str, Any],
    *,
    heuristic_scale: float,
) -> dict[str, Any]:
    cli_binary = resolve_cli_binary()
    graph_file = Path(manifest["meta"]["graph_file"])
    command = [str(cli_binary), "--graph", str(graph_file), "--start", request.start_node]
    if request.goal_node:
        command.extend(["--goal-node", request.goal_node])
    elif request.goal_task:
        command.extend(["--goal-task", request.goal_task])
    elif request.goal_route:
        command.extend(["--goal-route", request.goal_route])
    for node_id in request.blocked_nodes:
        command.extend(["--blocked-node", node_id])
    for edge_id in request.blocked_edges:
        command.extend(["--blocked-edge", edge_id])
    if heuristic_scale > 0.0:
        command.extend(["--heuristic-scale", str(heuristic_scale)])

    completed = subprocess.run(
        command,
        capture_output=True,
        text=True,
        cwd=WORKSPACE_ROOT or PACKAGE_PREFIX or PKG_ROOT,
        env=planner_cli_env(),
    )
    if completed.returncode not in (0, 2):
        raise RuntimeError(
            "planner_trace_cli failed\n"
            f"command: {' '.join(command)}\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    return json.loads(completed.stdout)


def frame_pose_from_node(manifest: dict[str, Any], node_id: str) -> dict[str, float] | None:
    for node in manifest["graphNodes"]:
        if node["id"] == node_id:
            return node["pose"]
    return None


def normalize_astar_trace(raw_trace: dict[str, Any], manifest: dict[str, Any]) -> dict[str, Any]:
    graph_document = RENDER.load_yaml(Path(manifest["meta"]["graph_file"]))
    node_map = {node["id"]: node for node in manifest["graphNodes"]}
    best_path_points = polyline_from_edge_ids(
        graph_document,
        [str(edge_id) for edge_id in raw_trace.get("edge_path", [])],
        fallback_node_path=[str(node_id) for node_id in raw_trace.get("node_path", [])],
    )
    scene_rects = build_obstacle_rects(manifest["sceneFeatures"])

    frames = []
    for frame in raw_trace.get("frames", []):
        active_node = str(frame.get("node_id", ""))
        robot_pose = node_map.get(active_node, {}).get("pose") if active_node else None
        frontier = []
        for entry in frame.get("frontier", []):
            pose = node_map.get(str(entry.get("node_id", "")), {}).get("pose")
            if pose is None:
                continue
            frontier.append(
                {
                    "nodeId": str(entry["node_id"]),
                    "pose": pose,
                    "gCost": float(entry.get("g_cost", 0.0)),
                    "fCost": float(entry.get("f_cost", 0.0)),
                }
            )
        expanded_nodes = []
        for node_id in frame.get("expanded_nodes", []):
            pose = node_map.get(str(node_id), {}).get("pose")
            if pose is not None:
                expanded_nodes.append({"nodeId": str(node_id), "pose": pose})
        frame_path_points = polyline_from_node_ids(graph_document, [str(node_id) for node_id in frame.get("best_path", [])])
        frames.append(
            {
                "stepIndex": int(frame.get("step_index", len(frames))),
                "algorithm": "astar",
                "phase": str(frame.get("event", "")),
                "label": str(frame.get("message", "")),
                "robotPose": robot_pose,
                "openSet": frontier,
                "expandedNodes": expanded_nodes,
                "bestPath": {
                    "nodeIds": [str(node_id) for node_id in frame.get("best_path", [])],
                    "points": frame_path_points,
                },
                "treeSegments": [],
                "candidateTrajectories": [],
                "selectedTrajectory": [],
                "metrics": {
                    "gCost": float(frame.get("g_cost", 0.0)),
                    "fCost": float(frame.get("f_cost", 0.0)),
                    "stepCost": float(frame.get("step_cost", 0.0)),
                },
            }
        )

    return {
        "success": bool(raw_trace.get("success", False)),
        "algorithm": "astar",
        "summary": {
            "goalKind": raw_trace.get("goal_kind", "node"),
            "goalValue": raw_trace.get("goal_value", ""),
            "framesCount": len(frames),
            "totalCost": raw_trace.get("total_cost"),
            "selectedCandidate": raw_trace.get("selected_candidate"),
            "candidateResults": raw_trace.get("candidate_results", []),
        },
        "path_points": best_path_points,
        "frames": frames,
        "obstacles": scene_rects,
    }


def resolve_goal_pose_from_trace(manifest: dict[str, Any], astar_run: dict[str, Any]) -> dict[str, float]:
    path_points = astar_run.get("path_points", [])
    if path_points:
        return path_points[-1]
    goal_value = astar_run["summary"].get("goalValue")
    pose = frame_pose_from_node(manifest, str(goal_value))
    if pose is not None:
        return pose
    defaults = manifest["defaults"]
    fallback = frame_pose_from_node(manifest, defaults["goalNode"])
    if fallback is None:
        raise RuntimeError("Could not resolve goal pose from trace or manifest defaults")
    return fallback


def run_offline_request(request: "PlannerRunRequest", manifest: dict[str, Any]) -> dict[str, Any]:
    graph_document = RENDER.load_yaml(Path(manifest["meta"]["graph_file"]))
    start_pose = frame_pose_from_node(manifest, request.start_node)
    if start_pose is None:
        raise HTTPException(status_code=400, detail=f"Unknown start node: {request.start_node}")

    if request.algorithm == "astar":
        heuristic_scale = 0.0 if request.strict_runtime else 1.0
        raw_trace = run_planner_trace_cli(request, manifest, heuristic_scale=heuristic_scale)
        return normalize_astar_trace(raw_trace, manifest)

    reference_astar = run_planner_trace_cli(
        request,
        manifest,
        heuristic_scale=0.0,
    )
    reference_run = normalize_astar_trace(reference_astar, manifest)
    goal_pose = resolve_goal_pose_from_trace(manifest, reference_run)
    blocked_points = blocked_sim_points(manifest, request.blocked_nodes)

    if request.algorithm == "rrt":
        return generate_rrt_run(
            graph_document,
            start_pose,
            goal_pose,
            manifest["sceneFeatures"],
            blocked_points=blocked_points,
            seed=request.random_seed,
        )

    if request.algorithm == "dwa":
        return generate_dwa_run(
            graph_document,
            start_pose,
            goal_pose,
            manifest["sceneFeatures"],
            reference_path=reference_run["path_points"],
            blocked_points=blocked_points,
        )

    raise HTTPException(status_code=400, detail=f"Unsupported algorithm: {request.algorithm}")


class PlannerRunRequest(BaseModel):
    algorithm: Literal["astar", "rrt", "dwa"] = "astar"
    mode: Literal["offline-sim", "live-ros"] = "offline-sim"
    team: Literal["blue", "red"] = "blue"
    start_node: str
    goal_node: str | None = None
    goal_task: str | None = None
    goal_route: str | None = None
    strict_runtime: bool = True
    animation_speed: float = Field(default=1.0, ge=0.1, le=8.0)
    blocked_nodes: list[str] = Field(default_factory=list)
    blocked_edges: list[str] = Field(default_factory=list)
    graph_file: str | None = None
    world_file: str | None = None
    kfs_config_file: str | None = None
    random_seed: int = 7


class RunControlRequest(BaseModel):
    action: Literal["play", "pause", "step", "reset", "seek"]
    cursor: int | None = None
    speed: float | None = Field(default=None, ge=0.1, le=8.0)


class LiveStartRequest(BaseModel):
    namespace: str = ""


# The adapter is imported directly in unit tests via importlib, so rebuild the
# models with an explicit typing namespace instead of relying on module lookup.
PlannerRunRequest.model_rebuild(_types_namespace={"Literal": Literal})
RunControlRequest.model_rebuild(_types_namespace={"Literal": Literal})
LiveStartRequest.model_rebuild()


class SimulationRun:
    def __init__(self, request: PlannerRunRequest, snapshot: dict[str, Any]) -> None:
        self.id = uuid.uuid4().hex
        self.request = request
        self.snapshot = snapshot
        self.frames = list(snapshot.get("frames", []))
        self.summary = dict(snapshot.get("summary", {}))
        self.state = "paused"
        self.cursor = 0
        self.speed = request.animation_speed
        self.subscribers: set[WebSocket] = set()
        self._ticker_task = asyncio.create_task(self._ticker_loop())

    async def _ticker_loop(self) -> None:
        try:
            while True:
                await asyncio.sleep(max(0.02, 0.12 / max(self.speed, 0.1)))
                if self.state != "playing" or not self.frames:
                    continue
                if self.cursor >= len(self.frames) - 1:
                    self.state = "paused"
                    await self.broadcast({"type": "state", "runId": self.id, "state": self.state, "cursor": self.cursor})
                    continue
                self.cursor += 1
                await self.broadcast(self.current_payload())
        except asyncio.CancelledError:
            return

    def current_payload(self) -> dict[str, Any]:
        frame = self.frames[self.cursor] if self.frames else None
        return {
            "type": "frame",
            "runId": self.id,
            "state": self.state,
            "cursor": self.cursor,
            "frameCount": len(self.frames),
            "summary": self.summary,
            "frame": frame,
        }

    async def broadcast(self, payload: dict[str, Any]) -> None:
        stale: list[WebSocket] = []
        for websocket in list(self.subscribers):
            try:
                await websocket.send_json(payload)
            except Exception:
                stale.append(websocket)
        for websocket in stale:
            self.subscribers.discard(websocket)

    async def subscribe(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.subscribers.add(websocket)
        await websocket.send_json(
            {
                "type": "meta",
                "runId": self.id,
                "state": self.state,
                "cursor": self.cursor,
                "frameCount": len(self.frames),
                "summary": self.summary,
            }
        )
        if self.frames:
            await websocket.send_json(self.current_payload())

    async def unsubscribe(self, websocket: WebSocket) -> None:
        self.subscribers.discard(websocket)

    async def control(self, request: RunControlRequest) -> dict[str, Any]:
        if request.speed is not None:
            self.speed = request.speed
        if request.action == "play":
            self.state = "playing"
        elif request.action == "pause":
            self.state = "paused"
        elif request.action == "reset":
            self.state = "paused"
            self.cursor = 0
        elif request.action == "seek":
            if request.cursor is None:
                raise HTTPException(status_code=400, detail="seek requires cursor")
            self.cursor = max(0, min(request.cursor, max(len(self.frames) - 1, 0)))
        elif request.action == "step":
            self.state = "paused"
            if request.cursor is not None:
                self.cursor = max(0, min(request.cursor, max(len(self.frames) - 1, 0)))
            elif self.frames:
                self.cursor = min(self.cursor + 1, len(self.frames) - 1)
        await self.broadcast(self.current_payload())
        return {"runId": self.id, "state": self.state, "cursor": self.cursor, "speed": self.speed}


class LiveRosBridge:
    def __init__(self) -> None:
        self.namespace = ""
        self.loop: asyncio.AbstractEventLoop | None = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.subscribers: set[WebSocket] = set()
        self._lock = threading.Lock()
        self.state: dict[str, Any] = {
            "routePath": [],
            "corridorPath": [],
            "activeEdge": "",
            "gateStatus": "",
            "blockOverlay": [],
            "trackingState": None,
            "timestamp": time.time(),
        }

    async def start(self, namespace: str = "") -> dict[str, Any]:
        self.namespace = namespace
        if self.thread and self.thread.is_alive():
            return {"status": "running", "namespace": self.namespace}
        self.loop = asyncio.get_running_loop()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._spin, name="topo-sim-live-bridge", daemon=True)
        self.thread.start()
        return {"status": "starting", "namespace": self.namespace}

    def _topic(self, name: str) -> str:
        namespace = self.namespace.strip("/")
        if not namespace:
            return name
        return f"/{namespace}{name}"

    def _spin(self) -> None:
        try:
            import rclpy
            from nav_msgs.msg import Path as RosPath
            from rc26_interfaces.msg import MfBlockOverlay, XhuTrackingState
            from std_msgs.msg import String
        except ImportError as exc:
            self._schedule_broadcast({"type": "live_error", "message": f"Live ROS bridge import failed: {exc}"})
            return

        if not rclpy.ok():
            rclpy.init(args=None)
        node = rclpy.create_node("topo_sim_live_bridge")

        def path_to_points(msg: Any) -> list[dict[str, float]]:
            return [
                {
                    "x": float(item.pose.position.x),
                    "y": float(item.pose.position.y),
                    "z": float(item.pose.position.z),
                    "yaw": 0.0,
                }
                for item in msg.poses
            ]

        def update_state(**kwargs: Any) -> None:
            with self._lock:
                self.state.update(kwargs)
                self.state["timestamp"] = time.time()
                snapshot = dict(self.state)
            self._schedule_broadcast({"type": "live_state", **snapshot})

        node.create_subscription(RosPath, self._topic("/topo_nav/route"), lambda msg: update_state(routePath=path_to_points(msg)), 10)
        node.create_subscription(
            RosPath,
            self._topic("/topo_nav/corridor"),
            lambda msg: update_state(corridorPath=path_to_points(msg)),
            10,
        )
        node.create_subscription(String, self._topic("/xhu_nav/active_edge"), lambda msg: update_state(activeEdge=msg.data), 10)
        node.create_subscription(String, self._topic("/xhu_nav/semantic_gate"), lambda msg: update_state(gateStatus=msg.data), 10)
        node.create_subscription(
            MfBlockOverlay,
            self._topic("/mf_block_overlay"),
            lambda msg: update_state(
                blockOverlay=[
                    {
                        "gridId": int(cell.grid_id),
                        "state": int(cell.state),
                        "confidence": float(cell.confidence),
                        "keepoutActive": bool(cell.keepout_active),
                    }
                    for cell in msg.cells
                ]
            ),
            10,
        )
        node.create_subscription(
            XhuTrackingState,
            self._topic("/xhu_nav/tracking_state"),
            lambda msg: update_state(
                trackingState={
                    "corridorId": msg.corridor_id,
                    "edgeId": msg.edge_id,
                    "status": msg.status,
                    "terminal": bool(msg.terminal),
                    "distanceToGoal": float(msg.distance_to_goal),
                    "reason": msg.reason,
                    "cmd": {"vx": float(msg.cmd_vx), "vy": float(msg.cmd_vy), "wz": float(msg.cmd_wz)},
                }
            ),
            10,
        )

        try:
            while rclpy.ok() and not self.stop_event.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        except rclpy.executors.ExternalShutdownException:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _schedule_broadcast(self, payload: dict[str, Any]) -> None:
        if self.loop is None:
            return
        asyncio.run_coroutine_threadsafe(self.broadcast(payload), self.loop)

    async def subscribe(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.subscribers.add(websocket)
        with self._lock:
            snapshot = dict(self.state)
        await websocket.send_json({"type": "live_state", **snapshot})

    async def unsubscribe(self, websocket: WebSocket) -> None:
        self.subscribers.discard(websocket)

    async def broadcast(self, payload: dict[str, Any]) -> None:
        stale: list[WebSocket] = []
        for websocket in list(self.subscribers):
            try:
                await websocket.send_json(payload)
            except Exception:
                stale.append(websocket)
        for websocket in stale:
            self.subscribers.discard(websocket)


app = FastAPI(title="RC26 Topo 3D Simulation Adapter", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

RUNS: dict[str, SimulationRun] = {}
LIVE_BRIDGE = LiveRosBridge()


def request_paths(request: PlannerRunRequest) -> tuple[Path, Path, Path]:
    graph_file = Path(request.graph_file) if request.graph_file else default_graph_for_team(request.team)
    world_file = Path(request.world_file) if request.world_file else default_world_file()
    kfs_file = Path(request.kfs_config_file) if request.kfs_config_file else default_kfs_config_file()
    return graph_file, world_file, kfs_file


@app.get("/api/health")
async def health() -> dict[str, Any]:
    return {"ok": True, "runs": len(RUNS), "frontend_dist": str(FRONTEND_DIST)}


@app.get("/api/scene-manifest")
async def scene_manifest(
    team: Literal["blue", "red"] = "blue",
    graph_file: str | None = None,
    world_file: str | None = None,
    kfs_config_file: str | None = None,
    full_geometry: bool = True,
) -> JSONResponse:
    manifest = build_scene_manifest(
        team=team,
        graph_file=Path(graph_file) if graph_file else default_graph_for_team(team),
        world_file=Path(world_file) if world_file else default_world_file(),
        kfs_config_file=Path(kfs_config_file) if kfs_config_file else default_kfs_config_file(),
        include_full_geometry=full_geometry,
    )
    return JSONResponse(manifest)


@app.post("/api/runs")
async def create_run(request: PlannerRunRequest) -> dict[str, Any]:
    graph_file, world_file, kfs_file = request_paths(request)
    manifest = build_scene_manifest(
        team=request.team,
        graph_file=graph_file,
        world_file=world_file,
        kfs_config_file=kfs_file,
        include_full_geometry=True,
    )
    snapshot = run_offline_request(request, manifest)
    run = SimulationRun(request, snapshot)
    RUNS[run.id] = run
    return {
        "runId": run.id,
        "frameCount": len(run.frames),
        "summary": run.summary,
        "state": run.state,
    }


@app.post("/api/runs/{run_id}/control")
async def control_run(run_id: str, request: RunControlRequest) -> dict[str, Any]:
    run = RUNS.get(run_id)
    if run is None:
        raise HTTPException(status_code=404, detail=f"Unknown run id: {run_id}")
    return await run.control(request)


@app.websocket("/api/runs/{run_id}/events")
async def run_events(run_id: str, websocket: WebSocket) -> None:
    run = RUNS.get(run_id)
    if run is None:
        await websocket.close(code=4404, reason="Unknown run id")
        return
    await run.subscribe(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        await run.unsubscribe(websocket)


@app.post("/api/live/start")
async def live_start(request: LiveStartRequest) -> dict[str, Any]:
    return await LIVE_BRIDGE.start(namespace=request.namespace)


@app.websocket("/api/live/events")
async def live_events(websocket: WebSocket) -> None:
    await LIVE_BRIDGE.subscribe(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        await LIVE_BRIDGE.unsubscribe(websocket)


if FRONTEND_DIST.is_dir():
    app.mount("/", StaticFiles(directory=FRONTEND_DIST, html=True), name="topo-sim-viewer")


@app.get("/")
async def root() -> Any:
    if FRONTEND_DIST.is_dir():
        return FileResponse(FRONTEND_DIST / "index.html")
    return {
        "message": "RC26 topo simulation adapter is running",
        "hint": f"Build the frontend in {FRONTEND_ROOT} to serve the viewer from this process",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the rc26_topo_nav 3D simulation adapter")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8796)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
