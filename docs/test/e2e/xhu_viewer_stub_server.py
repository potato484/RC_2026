#!/usr/bin/env python3
"""Stub backend for rc26_xhu_viewer browser E2E."""

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


VIEWER_TITLE = "RC26 全局比赛场地闭环可视化平台"
VIEWER_SUBTITLE = "统一消费 live 运行态、离线路径回放、行为树阶段和机构状态，不再依赖外部 Foxglove。"
LOCAL_PLANNER_SCENARIOS = [
    {
        "name": "pass_straight",
        "label": "直行通过",
        "snapshot_file": "local_planner/pass_straight.yaml",
    }
]


def make_display_catalog() -> list[dict[str, Any]]:
    return [
        {"id": "scene", "label": "场地", "short_label": "场", "group": "primary", "tone": "scene"},
        {"id": "route", "label": "路线", "short_label": "线", "group": "primary", "tone": "path"},
        {"id": "corridor", "label": "走廊", "short_label": "廊", "group": "primary", "tone": "path"},
        {"id": "lookahead", "label": "预瞄", "short_label": "瞄", "group": "primary", "tone": "path"},
        {"id": "robotPose", "label": "机器人", "short_label": "机", "group": "primary", "tone": "state"},
        {"id": "phaseZones", "label": "阶段区", "short_label": "区", "group": "primary", "tone": "risk"},
        {"id": "blocked", "label": "Keepout", "short_label": "禁", "group": "primary", "tone": "risk"},
        {"id": "openSet", "label": "前沿", "short_label": "前", "group": "advanced", "tone": "search"},
        {"id": "expanded", "label": "已探查", "short_label": "展", "group": "advanced", "tone": "search"},
        {"id": "graph", "label": "图结构", "short_label": "图", "group": "advanced", "tone": "search"},
        {"id": "keyNodes", "label": "关键点", "short_label": "点", "group": "advanced", "tone": "path"},
        {"id": "tree", "label": "搜索树", "short_label": "树", "group": "advanced", "tone": "search"},
        {"id": "candidates", "label": "候选轨迹", "short_label": "轨", "group": "advanced", "tone": "search"},
        {"id": "shadows", "label": "阴影", "short_label": "影", "group": "advanced", "tone": "appearance"},
    ]


def make_layout_presets() -> list[dict[str, Any]]:
    return [
        {
            "id": "operator",
            "label": "操作员",
            "description": "优先看场地、机器人、路线、当前阶段和关键风险。",
            "visible_displays": ["scene", "route", "corridor", "lookahead", "robotPose", "phaseZones", "blocked", "shadows"],
        },
        {
            "id": "engineering",
            "label": "工程",
            "description": "打开 graph、搜索回放和更多调试图层。",
            "visible_displays": [
                "scene",
                "route",
                "corridor",
                "lookahead",
                "robotPose",
                "phaseZones",
                "blocked",
                "openSet",
                "expanded",
                "graph",
                "keyNodes",
                "tree",
                "candidates",
                "shadows",
            ],
        },
        {
            "id": "diagnostic",
            "label": "诊断",
            "description": "保留场地主视图，但更关注诊断、定位和机构状态。",
            "visible_displays": ["scene", "route", "robotPose", "phaseZones", "blocked", "shadows"],
        },
    ]


def make_semantic_zones() -> list[dict[str, Any]]:
    return [
        {
            "id": "mf_zone",
            "label": "梅林区",
            "phase_key": "MFAreaTree",
            "color": "#2a9d8f",
            "source": "xhu_viewer_e2e_stub",
            "viewer_only": False,
            "polygon": [
                {"x": -0.8, "y": -0.5, "z": 0.03, "yaw": 0.0},
                {"x": 1.6, "y": -0.5, "z": 0.03, "yaw": 0.0},
                {"x": 1.6, "y": 1.2, "z": 0.03, "yaw": 0.0},
                {"x": -0.8, "y": 1.2, "z": 0.03, "yaw": 0.0},
            ],
        },
        {
            "id": "combat_zone",
            "label": "对抗区",
            "phase_key": "CombatAreaTree",
            "color": "#e76f51",
            "source": "xhu_viewer_e2e_stub",
            "viewer_only": True,
            "polygon": [
                {"x": -2.0, "y": -1.2, "z": 0.03, "yaw": 0.0},
                {"x": -1.0, "y": -1.2, "z": 0.03, "yaw": 0.0},
                {"x": -1.0, "y": 0.2, "z": 0.03, "yaw": 0.0},
                {"x": -2.0, "y": 0.2, "z": 0.03, "yaw": 0.0},
            ],
        },
    ]


def make_scene_manifest(team: str) -> dict[str, Any]:
    return {
        "meta": {
            "team": team,
            "graph_file": "stub_graph.yaml",
            "surface_graph_file": f"stub_surface_graph_{team}.yaml",
            "world_file": "stub_world.world",
            "kfs_config_file": "stub_kfs.yaml",
            "full_geometry": True,
        },
        "viewerMeta": {
            "viewer_title": VIEWER_TITLE,
            "viewer_subtitle": VIEWER_SUBTITLE,
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
        "semanticZones": make_semantic_zones(),
        "displayCatalog": make_display_catalog(),
        "layoutPresets": make_layout_presets(),
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
            "label": "goal reached",
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
            "metrics": {"gCost": 2.5, "fCost": 2.5, "stepCost": 1.5, "traceMode": "surface_route"},
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


class Pose3Request(BaseModel):
    x: float
    y: float
    z: float
    yaw: float = 0.0


class SurfaceRouteRequest(BaseModel):
    team: str = "blue"
    start_pick_world: Pose3Request
    goal_pick_world: Pose3Request
    projection_radius_m: float | None = None


class SurfaceRouteTraceFromNodesRequest(BaseModel):
    team: str = "blue"
    start_node_id: str
    goal_node_id: str
    surface_graph_file: str | None = None
    requested_start: Pose3Request | None = None
    requested_goal: Pose3Request | None = None


class LocalPlannerTraceRequest(BaseModel):
    scenario_name: str | None = None
    snapshot_file: str | None = None


def make_surface_route_segments() -> list[dict[str, Any]]:
    return [
        {
            "segment_id": "stub_seg_a",
            "from_node_id": "sf_start",
            "to_node_id": "stub_mid",
            "motion_type": "plane_move",
            "required_mode": "mf_traverse",
            "point_count": 2,
        },
        {
            "segment_id": "stub_seg_b",
            "from_node_id": "stub_mid",
            "to_node_id": "sf_goal",
            "motion_type": "ramp_down",
            "required_mode": "mf_exit",
            "point_count": 2,
        },
    ]


def make_surface_route_preview_payload(request: SurfaceRouteRequest) -> dict[str, Any]:
    return {
        "success": True,
        "failure_code": "",
        "failure_reason": "",
        "projected_start_node_id": "sf_start",
        "projected_goal_node_id": "sf_goal",
        "projected_start": {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
        "projected_goal": {"x": 1.4, "y": 0.9, "z": 0.0, "yaw": 0.0},
        "path_points": [
            {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
            {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
            {"x": 1.4, "y": 0.9, "z": 0.0, "yaw": 0.0},
        ],
        "segments": make_surface_route_segments(),
        "team": request.team,
        "surface_graph_file": f"stub_surface_graph_{request.team}.yaml",
        "planning_timing_ms": {
            "surfaceProjection": 4.5,
            "surfacePlanning": 12.4,
            "surfacePathExpand": 3.2,
            "surfaceSegmentBuild": 1.6,
            "surfaceCompletePlanning": 21.7,
            "surfaceRouteCli": 24.1,
        },
        "planning_logs": [
            {
                "stage": "request",
                "level": "info",
                "title": "收到路线请求",
                "message": "已准备表面图投影与路径规划输入",
                "elapsed_ms": None,
                "fields": [],
            },
            {
                "stage": "surface_route_cli",
                "level": "info",
                "title": "表面路线预览",
                "message": "已完成点击点投影并生成路线",
                "elapsed_ms": 24.1,
                "fields": [],
            },
        ],
    }


def make_surface_route_trace_from_nodes_payload(request: SurfaceRouteTraceFromNodesRequest) -> dict[str, Any]:
    frames = make_frames()
    return {
        "success": True,
        "failure_code": "",
        "failure_reason": "",
        "projected_start_node_id": request.start_node_id,
        "projected_goal_node_id": request.goal_node_id,
        "team": request.team,
        "surface_graph_file": request.surface_graph_file or f"stub_surface_graph_{request.team}.yaml",
        "planning_timing_ms": {
            "tracePlanning": 18.3,
            "plannerTraceCli": 28.4,
        },
        "planning_logs": [
            {
                "stage": "planner_trace_cli",
                "level": "info",
                "title": "搜索回放生成",
                "message": "已按运行时启发式导出搜索回放",
                "elapsed_ms": 28.4,
                "fields": [],
            },
        ],
        "summary": {
            "goalKind": "node",
            "goalValue": request.goal_node_id,
            "framesCount": len(frames),
            "returnedFramesCount": len(frames),
            "framesSampled": False,
            "totalCost": 2.5,
            "projectedStartNodeId": request.start_node_id,
            "projectedGoalNodeId": request.goal_node_id,
            "requestedStart": request.requested_start.model_dump() if request.requested_start else None,
            "requestedGoal": request.requested_goal.model_dump() if request.requested_goal else None,
            "tracePlanningMs": 18.3,
            "traceElapsedMs": 28.4,
        },
        "node_poses": {
            request.start_node_id: {"x": -1.4, "y": -0.9, "z": 0.0, "yaw": 0.0},
            "stub_mid": {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0},
            request.goal_node_id: {"x": 1.4, "y": 0.9, "z": 0.0, "yaw": 0.0},
        },
        "frames": frames,
    }


def make_surface_route_trace_payload(request: SurfaceRouteRequest) -> dict[str, Any]:
    preview = make_surface_route_preview_payload(request)
    trace = make_surface_route_trace_from_nodes_payload(
        SurfaceRouteTraceFromNodesRequest(
            team=request.team,
            start_node_id=preview["projected_start_node_id"],
            goal_node_id=preview["projected_goal_node_id"],
            surface_graph_file=preview["surface_graph_file"],
            requested_start=request.start_pick_world,
            requested_goal=request.goal_pick_world,
        )
    )
    total_elapsed_ms = round(
        float(preview["planning_timing_ms"]["surfaceRouteCli"]) +
        float(trace["planning_timing_ms"]["plannerTraceCli"]),
        2,
    )
    return {
        **preview,
        "planning_timing_ms": {
            **preview["planning_timing_ms"],
            **trace["planning_timing_ms"],
            "surfaceRouteTraceTotal": total_elapsed_ms,
        },
        "planning_logs": [
            *preview["planning_logs"],
            *trace["planning_logs"],
            {
                "stage": "trace_pipeline",
                "level": "info",
                "title": "回放整合结果",
                "message": "已组合路线预览结果与后台搜索回放输出",
                "elapsed_ms": total_elapsed_ms,
                "fields": [],
            },
        ],
        "summary": {
            **trace["summary"],
            "surfaceProjectionMs": preview["planning_timing_ms"]["surfaceProjection"],
            "surfacePlanningMs": preview["planning_timing_ms"]["surfacePlanning"],
            "surfacePathExpandMs": preview["planning_timing_ms"]["surfacePathExpand"],
            "surfaceSegmentBuildMs": preview["planning_timing_ms"]["surfaceSegmentBuild"],
            "surfaceCompletePlanningMs": preview["planning_timing_ms"]["surfaceCompletePlanning"],
            "previewElapsedMs": preview["planning_timing_ms"]["surfaceRouteCli"],
            "traceElapsedMs": trace["planning_timing_ms"]["plannerTraceCli"],
            "totalElapsedMs": total_elapsed_ms,
        },
        "node_poses": trace["node_poses"],
        "frames": trace["frames"],
    }


def make_local_planner_trace_payload(request: LocalPlannerTraceRequest) -> dict[str, Any]:
    snapshot_label = request.scenario_name or "pass_straight"
    snapshot_file = request.snapshot_file or f"local_planner/{snapshot_label}.yaml"
    return {
        "success": True,
        "snapshotLabel": snapshot_label,
        "snapshot_file": snapshot_file,
        "traceMode": "local_planner",
        "result": {
            "status": "ok",
            "reason": "",
            "hasSolution": True,
            "blockedByKeepout": False,
            "blockedByTerrain": False,
            "shouldRotateRecovery": False,
            "cmd": {"vx": 0.12, "vy": 0.0, "wz": 0.0},
            "bestScore": 1.0,
            "clearanceMarginM": 0.22,
        },
        "summary": {
            "candidateCount": 3,
            "linearLimit": 0.8,
            "angularLimit": 1.2,
            "preferredLinearSpeed": 0.5,
            "currentPathDistance": 1.2,
            "goalHeadingError": 0.1,
            "semanticRevision": 3,
            "finalStatus": "ok",
            "finalReason": "",
        },
        "frames": [],
    }


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
        self.state = self._build_state(started=False)

    def _build_state(self, started: bool) -> dict[str, Any]:
        state: dict[str, Any] = {
            "routePath": [],
            "corridorPath": [],
            "localPlannerPreviewPath": [],
            "controlState": None,
            "activeEdge": "",
            "gateStatus": "",
            "blockOverlay": [],
            "motionModeState": None,
            "trackingState": None,
            "localPlannerState": None,
            "recoveryState": None,
            "semanticSummary": None,
            "localizationHealth": None,
            "localizationBackendStatus": None,
            "operatorStatus": None,
            "visualizationEvents": [],
            "mechanismState": None,
            "btSnapshot": None,
            "btEvents": [],
            "timestamp": time.time(),
        }
        if not started:
            return state

        state.update(
            {
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
                "localPlannerPreviewPath": [
                    {"x": -0.2, "y": -0.05, "z": 0.35, "yaw": 0.0},
                    {"x": 0.35, "y": 0.22, "z": 0.28, "yaw": 0.0},
                    {"x": 0.9, "y": 0.6, "z": 0.1, "yaw": 0.0},
                ],
                "controlState": {
                    "pose": {"x": 0.4, "y": 0.2, "z": 0.1, "yaw": 0.15},
                    "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.2},
                },
                "activeEdge": "stub_edge_live",
                "gateStatus": "gate_open",
                "blockOverlay": [{"gridId": 4, "state": 1, "confidence": 0.92, "keepoutActive": True}],
                "motionModeState": {
                    "activeMode": "surface_route",
                    "reason": "follow corridor",
                    "stopRequired": False,
                    "timedOut": False,
                    "maxLinearSpeed": 0.8,
                    "maxAngularSpeed": 1.2,
                },
                "trackingState": {
                    "corridorId": "stub_corridor",
                    "edgeId": "stub_edge_live",
                    "status": "tracking",
                    "terminal": False,
                    "distanceToGoal": 0.48,
                    "reason": "",
                    "cmd": {"vx": 0.3, "vy": 0.0, "wz": 0.0},
                },
                "localPlannerState": {
                    "corridorId": "stub_corridor",
                    "edgeId": "stub_edge_live",
                    "status": "running",
                    "terminal": False,
                    "observeOnly": False,
                    "semanticRevision": 3,
                    "cmd": {"vx": 0.12, "vy": 0.0, "wz": 0.04},
                    "bestScore": 1.04,
                    "clearanceMarginM": 0.22,
                    "reason": "clear",
                },
                "recoveryState": {
                    "corridorId": "stub_corridor",
                    "edgeId": "stub_edge_live",
                    "recoveryName": "none",
                    "status": "idle",
                    "terminal": False,
                    "elapsedSec": 0.0,
                    "reason": "",
                },
                "semanticSummary": {
                    "revision": 3,
                    "terrainAvailable": True,
                    "keepoutAvailable": True,
                    "blockedCells": 4,
                    "slowCells": 1,
                    "maxObstacleProbability": 0.35,
                    "maxDropProbability": 0.12,
                    "activeSources": ["terrain_grid", "keepout_overlay"],
                    "activeReasons": ["keepout"],
                },
                "localizationHealth": {
                    "level": 1,
                    "reason": "imu warmup",
                    "localizationState": "tracking",
                    "controlDegraded": False,
                    "sigmaXy": 0.03,
                    "sigmaYaw": 0.02,
                },
                "localizationBackendStatus": {
                    "optimizerReady": True,
                    "optimizerState": "healthy",
                    "graphHealth": 0.96,
                    "loopCandidateCount": 2,
                    "acceptedLoopCount": 1,
                    "acceptedAnchorCount": 4,
                    "imuSpike": False,
                },
                "operatorStatus": {
                    "overallLevel": 1,
                    "overallReason": "等待定位收敛",
                    "localizationLevel": 1,
                    "localizationReason": "sigma warmup",
                    "controllerLevel": 0,
                    "navSafetyLevel": 0,
                    "terrainLevel": 0,
                    "keepoutLevel": 1,
                    "mechanismLevel": 0,
                    "activeEventCodes": ["LOC_WARN"],
                    "topicTimeoutCount": 0,
                },
                "visualizationEvents": [
                    {
                        "code": "LOC_WARN",
                        "severity": 2,
                        "title": "定位提醒",
                        "detail": "定位尚未完全收敛",
                        "sourceSignal": "/localization/health",
                        "recommendation": "观察 2 秒",
                        "active": True,
                    }
                ],
                "mechanismState": {
                    "tipState": 1,
                    "halOpen": False,
                    "lockedTipSlot": 2,
                    "assembledCount": 3,
                    "lastErrorCode": 0,
                    "cmdElapsedMs": 18,
                    "ackTimeoutCount": 0,
                    "reconnectCount": 0,
                    "parseErrorCount": 0,
                    "avgRttMs": 4.5,
                    "commHealthLevel": 0,
                },
                "btSnapshot": {
                    "tickSeq": 8,
                    "treeStatus": 1,
                    "tickDurationMs": 12.4,
                    "activeSubtreeId": "MFAreaTree",
                    "runningPathUids": [101, 102],
                },
                "btEvents": [
                    {
                        "uid": 101,
                        "nodeName": "NavigateSurfaceRoute",
                        "fullPath": "Root/Planner/NavigateSurfaceRoute",
                        "status": 1,
                        "prevStatus": 0,
                    }
                ],
                "timestamp": time.time(),
            }
        )
        return state

    async def start(self, namespace: str = "") -> dict[str, Any]:
        self.started = True
        self.state = self._build_state(started=True)
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


app = FastAPI(title="RC26 Visualization E2E Stub", version="0.1.0")
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


@app.post("/api/surface-route/preview")
async def surface_route_preview(request: SurfaceRouteRequest) -> dict[str, Any]:
    return make_surface_route_preview_payload(request)


@app.post("/api/surface-route/trace")
async def surface_route_trace(request: SurfaceRouteRequest) -> dict[str, Any]:
    return make_surface_route_trace_payload(request)


@app.post("/api/surface-route/trace-from-nodes")
async def surface_route_trace_from_nodes(request: SurfaceRouteTraceFromNodesRequest) -> dict[str, Any]:
    return make_surface_route_trace_from_nodes_payload(request)


@app.post("/api/surface-route/execute")
async def surface_route_execute(request: SurfaceRouteRequest) -> dict[str, Any]:
    return {
        "accepted": True,
        "preview": make_surface_route_preview_payload(request),
    }


@app.get("/api/local-planner/scenarios")
async def local_planner_scenarios() -> dict[str, Any]:
    return {"scenarios": LOCAL_PLANNER_SCENARIOS}


@app.post("/api/local-planner/trace")
async def local_planner_trace(request: LocalPlannerTraceRequest) -> dict[str, Any]:
    return make_local_planner_trace_payload(request)


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
    parser = argparse.ArgumentParser(description="Run the visualization viewer E2E stub backend")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8877)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
