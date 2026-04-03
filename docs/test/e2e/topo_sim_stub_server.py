#!/usr/bin/env python3
"""Stub backend for sim_viewer browser E2E."""

from __future__ import annotations

import argparse
import asyncio
import time
import uuid
from typing import Any

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field


def make_scene_manifest(team: str) -> dict[str, Any]:
    return {
        "meta": {
            "team": team,
            "graph_file": "stub_graph.yaml",
            "world_file": "stub_world.world",
            "kfs_config_file": "stub_kfs.yaml",
            "full_geometry": True,
        },
        "bounds": {
            "min_x": -2.5,
            "max_x": 2.5,
            "min_y": -2.0,
            "max_y": 2.0,
            "min_z": 0.0,
            "max_z": 1.2,
        },
        "lights": {
            "ambient": [255, 255, 255],
            "background": [235, 238, 240],
            "lights": [
                {
                    "name": "stub_sun",
                    "type": "directional",
                    "pose": {"x": 3.0, "y": -2.5, "z": 5.0, "yaw": 0.0},
                    "direction": [0.0, 0.0, -1.0],
                    "diffuse": [255, 255, 255],
                    "cast_shadows": True,
                }
            ],
        },
        "sceneFeatures": [
            {
                "id": "ground_a",
                "name": "Stub Ground A",
                "material_symbol": "stub-ground",
                "fill": "#dbe6d4",
                "opacity": 0.96,
                "render_class": "world-ground",
                "avg_z": 0.0,
                "z_span": 0.0,
                "area_xy": 8.0,
                "points": [
                    {"x": -2.0, "y": -1.5, "z": 0.0, "yaw": 0.0},
                    {"x": 0.0, "y": -1.5, "z": 0.0, "yaw": 0.0},
                    {"x": 0.0, "y": 1.5, "z": 0.0, "yaw": 0.0},
                    {"x": -2.0, "y": 1.5, "z": 0.0, "yaw": 0.0},
                ],
            },
            {
                "id": "ground_b",
                "name": "Stub Ground B",
                "material_symbol": "stub-ground",
                "fill": "#cfd9ef",
                "opacity": 0.96,
                "render_class": "world-ground",
                "avg_z": 0.0,
                "z_span": 0.0,
                "area_xy": 8.0,
                "points": [
                    {"x": 0.1, "y": -1.5, "z": 0.0, "yaw": 0.0},
                    {"x": 2.0, "y": -1.5, "z": 0.0, "yaw": 0.0},
                    {"x": 2.0, "y": 1.5, "z": 0.0, "yaw": 0.0},
                    {"x": 0.1, "y": 1.5, "z": 0.0, "yaw": 0.0},
                ],
            },
            {
                "id": "platform_center",
                "name": "Stub Platform",
                "material_symbol": "stub-platform",
                "fill": "#f1e7d6",
                "opacity": 0.88,
                "render_class": "world-platform",
                "avg_z": 0.35,
                "z_span": 0.0,
                "area_xy": 0.64,
                "points": [
                    {"x": -0.4, "y": -0.4, "z": 0.35, "yaw": 0.0},
                    {"x": 0.4, "y": -0.4, "z": 0.35, "yaw": 0.0},
                    {"x": 0.4, "y": 0.4, "z": 0.35, "yaw": 0.0},
                    {"x": -0.4, "y": 0.4, "z": 0.35, "yaw": 0.0},
                ],
            },
        ],
        "graphNodes": [
            {
                "id": "stub_start",
                "type": "staging",
                "block_id": 0,
                "base_cost": 0.0,
                "operation_tag": "",
                "pose": {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
            },
            {
                "id": "stub_mid",
                "type": "mf_edge_pose",
                "block_id": 4,
                "base_cost": 1.0,
                "operation_tag": "grab",
                "pose": {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
            },
            {
                "id": "stub_goal",
                "type": "ramp_exit",
                "block_id": 0,
                "base_cost": 2.0,
                "operation_tag": "",
                "pose": {"x": 1.4, "y": 0.9, "z": 0.0, "yaw": 0.0},
            },
        ],
        "graphEdges": [
            {
                "id": "stub_edge_a",
                "from": "stub_start",
                "to": "stub_mid",
                "motion_type": "plane_move",
                "height_change": 0.35,
                "required_mode": "mf_traverse",
                "base_cost": 1.0,
                "points": [
                    {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
                    {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
                ],
            },
            {
                "id": "stub_edge_b",
                "from": "stub_mid",
                "to": "stub_goal",
                "motion_type": "ramp_down",
                "height_change": -0.35,
                "required_mode": "mf_exit",
                "base_cost": 1.5,
                "points": [
                    {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
                    {"x": 1.4, "y": 0.9, "z": 0.0, "yaw": 0.0},
                ],
            },
        ],
        "tasks": [{"task_tag": "stub_task", "candidate_nodes": ["stub_goal"]}],
        "routes": [{"route_tag": "stub_route", "nodes": ["stub_start", "stub_mid", "stub_goal"]}],
        "meilinSlots": [{"block_id": 4, "x": 0.0, "y": 0.0, "z": 0.35}],
        "cameraPresets": [
            {
                "id": "orbit",
                "kind": "perspective",
                "position": {"x": 3.2, "y": -3.1, "z": 2.4, "yaw": 0.0},
                "target": {"x": 0.0, "y": 0.0, "z": 0.4, "yaw": 0.0},
            },
            {
                "id": "follow",
                "kind": "perspective",
                "position": {"x": -1.0, "y": -1.8, "z": 1.3, "yaw": 0.0},
                "target": {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
            },
            {
                "id": "first_person",
                "kind": "perspective",
                "position": {"x": -1.2, "y": -1.0, "z": 0.7, "yaw": 0.0},
                "target": {"x": 0.0, "y": 0.0, "z": 0.4, "yaw": 0.0},
            },
            {
                "id": "top_ortho",
                "kind": "orthographic",
                "position": {"x": 0.0, "y": 0.0, "z": 5.0, "yaw": 0.0},
                "target": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            },
            {
                "id": "side_ortho",
                "kind": "orthographic",
                "position": {"x": 5.0, "y": 0.0, "z": 1.3, "yaw": 0.0},
                "target": {"x": 0.0, "y": 0.0, "z": 0.4, "yaw": 0.0},
            },
            {
                "id": "side_perspective",
                "kind": "perspective",
                "position": {"x": 3.6, "y": -2.2, "z": 1.5, "yaw": 0.0},
                "target": {"x": 0.0, "y": 0.0, "z": 0.4, "yaw": 0.0},
            },
        ],
        "defaults": {"startNode": "stub_start", "goalNode": "stub_goal"},
    }


def make_frames() -> list[dict[str, Any]]:
    best_path = {
        "nodeIds": ["stub_start", "stub_mid", "stub_goal"],
        "points": [
            {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
            {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
            {"x": 1.4, "y": 0.9, "z": 0.0, "yaw": 0.0},
        ],
    }
    return [
        {
            "stepIndex": 0,
            "algorithm": "astar",
            "phase": "init",
            "label": "planner initialized",
            "robotPose": {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
            "openSet": [
                {
                    "nodeId": "stub_start",
                    "pose": {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
                    "gCost": 0.0,
                    "fCost": 2.5,
                }
            ],
            "expandedNodes": [],
            "bestPath": {"nodeIds": [], "points": []},
            "treeSegments": [],
            "candidateTrajectories": [],
            "selectedTrajectory": [],
            "metrics": {"gCost": 0.0, "fCost": 2.5, "stepCost": 0.0},
        },
        {
            "stepIndex": 1,
            "algorithm": "astar",
            "phase": "pop",
            "label": "expanded current best node",
            "robotPose": {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
            "openSet": [
                {
                    "nodeId": "stub_mid",
                    "pose": {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
                    "gCost": 1.0,
                    "fCost": 2.5,
                }
            ],
            "expandedNodes": [{"nodeId": "stub_start", "pose": {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0}}],
            "bestPath": {
                "nodeIds": ["stub_start", "stub_mid"],
                "points": best_path["points"][:2],
            },
            "treeSegments": [
                {
                    "from": {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
                    "to": {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
                }
            ],
            "candidateTrajectories": [],
            "selectedTrajectory": [],
            "metrics": {"gCost": 1.0, "fCost": 2.5, "stepCost": 1.0},
        },
        {
            "stepIndex": 2,
            "algorithm": "astar",
            "phase": "goal",
            "label": "reached stub goal",
            "robotPose": {"x": 1.4, "y": 0.9, "z": 0.0, "yaw": 0.0},
            "openSet": [],
            "expandedNodes": [
                {"nodeId": "stub_start", "pose": {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0}},
                {"nodeId": "stub_mid", "pose": {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0}},
            ],
            "bestPath": best_path,
            "treeSegments": [
                {
                    "from": {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
                    "to": {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
                },
                {
                    "from": {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
                    "to": {"x": 1.4, "y": 0.9, "z": 0.0, "yaw": 0.0},
                },
            ],
            "candidateTrajectories": [],
            "selectedTrajectory": [],
            "metrics": {"gCost": 2.5, "fCost": 2.5, "stepCost": 1.5},
        },
    ]


class PlannerRunRequest(BaseModel):
    algorithm: str = "astar"
    mode: str = "offline-sim"
    team: str = "blue"
    start_node: str
    goal_node: str | None = None
    goal_task: str | None = None
    goal_route: str | None = None
    strict_runtime: bool = True
    animation_speed: float = Field(default=1.0, ge=0.1, le=8.0)
    blocked_nodes: list[str] = Field(default_factory=list)
    blocked_edges: list[str] = Field(default_factory=list)


class RunControlRequest(BaseModel):
    action: str
    cursor: int | None = None
    speed: float | None = Field(default=None, ge=0.1, le=8.0)


class LiveStartRequest(BaseModel):
    namespace: str = ""


class StubRun:
    def __init__(self, request: PlannerRunRequest):
        self.id = uuid.uuid4().hex
        self.request = request
        self.frames = make_frames()
        self.summary = {
            "goalKind": "node" if request.goal_node else "task" if request.goal_task else "route" if request.goal_route else "node",
            "goalValue": request.goal_node or request.goal_task or request.goal_route or "stub_goal",
            "framesCount": len(self.frames),
            "totalCost": 2.5,
            "selectedCandidate": "",
            "candidateResults": [],
        }
        self.state = "paused"
        self.cursor = 0
        self.speed = request.animation_speed
        self.subscribers: set[WebSocket] = set()
        self._send_lock = asyncio.Lock()
        self._ticker_task = asyncio.create_task(self._ticker_loop())

    async def _ticker_loop(self) -> None:
        try:
            while True:
                await asyncio.sleep(max(0.05, 0.18 / max(self.speed, 0.1)))
                if self.state != "playing":
                    continue
                if self.cursor >= len(self.frames) - 1:
                    self.state = "paused"
                    await self.broadcast(self.current_payload())
                    continue
                self.cursor += 1
                await self.broadcast(self.current_payload())
        except asyncio.CancelledError:
            return

    def current_payload(self) -> dict[str, Any]:
        return {
            "type": "frame",
            "runId": self.id,
            "state": self.state,
            "cursor": self.cursor,
            "frameCount": len(self.frames),
            "summary": self.summary,
            "frame": self.frames[self.cursor],
        }

    async def broadcast(self, payload: dict[str, Any]) -> None:
        stale: list[WebSocket] = []
        for websocket in list(self.subscribers):
            try:
                await self._send_json(websocket, payload)
            except Exception:
                stale.append(websocket)
        for websocket in stale:
            self.subscribers.discard(websocket)

    async def _send_json(self, websocket: WebSocket, payload: dict[str, Any]) -> None:
        # CI 上 create-run、subscribe、step 可能高度重叠；串行化同一 run 的发送，避免首帧与 step 更新乱序。
        async with self._send_lock:
            await websocket.send_json(payload)

    async def subscribe(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.subscribers.add(websocket)
        await self._send_json(
            websocket,
            {
                "type": "meta",
                "runId": self.id,
                "state": self.state,
                "cursor": self.cursor,
                "frameCount": len(self.frames),
                "summary": self.summary,
            },
        )
        await self._send_json(websocket, self.current_payload())

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
        elif request.action == "seek" and request.cursor is not None:
            self.cursor = max(0, min(request.cursor, len(self.frames) - 1))
        elif request.action == "step":
            self.state = "paused"
            self.cursor = min(self.cursor + 1, len(self.frames) - 1)
        await self.broadcast(self.current_payload())
        return {"runId": self.id, "state": self.state, "cursor": self.cursor, "speed": self.speed}


class LiveBridge:
    def __init__(self) -> None:
        self.started = False
        self.subscribers: set[WebSocket] = set()
        self._send_lock = asyncio.Lock()
        self.state = {
            "routePath": [],
            "corridorPath": [],
            "activeEdge": "",
            "gateStatus": "",
            "blockOverlay": [],
            "trackingState": None,
            "timestamp": time.time(),
        }

    async def start(self, namespace: str = "") -> dict[str, Any]:
        self.started = True
        self.state = {
            "routePath": [
                {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
                {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
                {"x": 1.4, "y": 0.9, "z": 0.0, "yaw": 0.0},
            ],
            "corridorPath": [
                {"x": -1.2, "y": -0.8, "z": 0.0, "yaw": 0.0},
                {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
                {"x": 1.2, "y": 0.8, "z": 0.0, "yaw": 0.0},
            ],
            "activeEdge": "stub_edge_live",
            "gateStatus": "gate_open",
            "blockOverlay": [{"gridId": 4, "state": 1, "confidence": 0.92, "keepoutActive": True}],
            "trackingState": {
                "corridorId": "stub_corridor",
                "edgeId": "stub_edge_live",
                "status": "tracking",
                "terminal": False,
                "distanceToGoal": 0.48,
                "reason": "",
                "cmd": {"vx": 0.3, "vy": 0.0, "wz": 0.0},
            },
            "timestamp": time.time(),
        }
        snapshot = {"type": "live_state", **self.state}
        await self.broadcast(snapshot)
        return {"status": "starting", "namespace": namespace, "snapshot": snapshot}

    async def subscribe(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.subscribers.add(websocket)
        await self._send_json(websocket, {"type": "live_state", **self.state})

    async def unsubscribe(self, websocket: WebSocket) -> None:
        self.subscribers.discard(websocket)

    async def broadcast(self, payload: dict[str, Any]) -> None:
        stale: list[WebSocket] = []
        for websocket in list(self.subscribers):
            try:
                await self._send_json(websocket, payload)
            except Exception:
                stale.append(websocket)
        for websocket in stale:
            self.subscribers.discard(websocket)

    async def _send_json(self, websocket: WebSocket, payload: dict[str, Any]) -> None:
        async with self._send_lock:
            await websocket.send_json(payload)


app = FastAPI(title="RC26 Topo Sim E2E Stub", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

RUNS: dict[str, StubRun] = {}
LIVE = LiveBridge()


@app.get("/health")
async def health() -> dict[str, Any]:
    return {"ok": True, "runs": len(RUNS)}


@app.get("/api/scene-manifest")
async def scene_manifest(team: str = "blue", full_geometry: bool = True) -> dict[str, Any]:
    manifest = make_scene_manifest(team)
    manifest["meta"]["full_geometry"] = bool(full_geometry)
    return manifest


@app.post("/api/runs")
async def create_run(request: PlannerRunRequest) -> dict[str, Any]:
    run = StubRun(request)
    RUNS[run.id] = run
    return {
        "runId": run.id,
        "frameCount": len(run.frames),
        "summary": run.summary,
        "state": run.state,
    }


@app.post("/api/runs/{run_id}/control")
async def control_run(run_id: str, request: RunControlRequest) -> dict[str, Any]:
    return await RUNS[run_id].control(request)


@app.websocket("/api/runs/{run_id}/events")
async def run_events(run_id: str, websocket: WebSocket) -> None:
    run = RUNS[run_id]
    await run.subscribe(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        await run.unsubscribe(websocket)


@app.post("/api/live/start")
async def live_start(request: LiveStartRequest) -> dict[str, Any]:
    return await LIVE.start(request.namespace)


@app.websocket("/api/live/events")
async def live_events(websocket: WebSocket) -> None:
    await LIVE.subscribe(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        await LIVE.unsubscribe(websocket)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the sim_viewer E2E stub backend")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8877)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
