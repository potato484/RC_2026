// @vitest-environment jsdom

import React from 'react';
import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';

import { useSimStore } from './store';
import type {
  LiveEvent,
  LocalPlannerTraceResponse,
  SceneManifest,
  SurfaceRoutePreviewResponse,
  SurfaceRouteTraceFromNodesResponse,
} from './types';

let lastSceneCanvasProps: Record<string, unknown> | null = null;

vi.mock('./components/SceneCanvas', () => ({
  SceneCanvas: (props: Record<string, unknown>) => {
    lastSceneCanvasProps = props;
    return <div data-testid="scene-canvas">scene-canvas</div>;
  },
}));

vi.mock('./api', () => ({
  executeSurfaceRoute: vi.fn(),
  fetchLocalPlannerScenarios: vi.fn(),
  fetchSceneManifest: vi.fn(),
  previewSurfaceRoute: vi.fn(),
  startLiveBridge: vi.fn(),
  traceLocalPlannerScenario: vi.fn(),
  traceSurfaceRouteFromNodes: vi.fn(),
}));

import App from './App';
import {
  executeSurfaceRoute,
  fetchLocalPlannerScenarios,
  fetchSceneManifest,
  previewSurfaceRoute,
  startLiveBridge,
  traceLocalPlannerScenario,
  traceSurfaceRouteFromNodes,
} from './api';

const initialState = useSimStore.getState();

function createSceneManifest(): SceneManifest {
  return {
    meta: {
      team: 'blue',
      graph_file: 'graph.yaml',
      surface_graph_file: 'surface.yaml',
      world_file: 'scene.world',
      kfs_config_file: 'kfs.yaml',
      full_geometry: true,
    },
    viewerMeta: {
      viewer_title: 'RC26 全局比赛场地闭环可视化平台',
      viewer_subtitle: '统一消费 live 运行态、离线路线回放、行为树阶段、定位健康和机构状态，不再依赖外部 Foxglove。',
    },
    bounds: {
      min_x: -1,
      max_x: 1,
      min_y: -1,
      max_y: 1,
      min_z: 0,
      max_z: 1,
    },
    lights: {
      ambient: [255, 255, 255],
      background: [240, 240, 240],
      lights: [],
    },
    sceneFeatures: [],
    graphNodes: [
      {
        id: 'node-a',
        type: 'staging',
        block_id: 0,
        base_cost: 0,
        operation_tag: '',
        pose: { x: -0.5, y: -0.5, z: 0, yaw: 0 },
      },
    ],
    graphEdges: [],
    tasks: [],
    routes: [],
    meilinSlots: [{ block_id: 7, x: 0.6, y: 0.2, z: 0 }],
    cameraPresets: [
      {
        id: 'orbit',
        kind: 'perspective',
        position: { x: 1, y: 1, z: 1, yaw: 0 },
        target: { x: 0, y: 0, z: 0, yaw: 0 },
      },
      {
        id: 'follow',
        kind: 'perspective',
        position: { x: 1, y: 1, z: 1, yaw: 0 },
        target: { x: 0, y: 0, z: 0, yaw: 0 },
      },
      {
        id: 'top_ortho',
        kind: 'orthographic',
        position: { x: 1, y: 1, z: 1, yaw: 0 },
        target: { x: 0, y: 0, z: 0, yaw: 0 },
      },
      {
        id: 'side_perspective',
        kind: 'perspective',
        position: { x: 1, y: 1, z: 1, yaw: 0 },
        target: { x: 0, y: 0, z: 0, yaw: 0 },
      },
    ],
    semanticZones: [
      {
        id: 'mf_zone',
        label: '梅林区',
        phase_key: 'MFAreaTree',
        color: '#2a9d8f',
        source: 'viewer',
        viewer_only: false,
        polygon: [
          { x: 0, y: 0, z: 0, yaw: 0 },
          { x: 1, y: 0, z: 0, yaw: 0 },
          { x: 1, y: 1, z: 0, yaw: 0 },
        ],
      },
    ],
    displayCatalog: [
      { id: 'scene', label: '场地', short_label: '场', group: 'primary', tone: 'scene' },
      { id: 'route', label: '路线', short_label: '线', group: 'primary', tone: 'path' },
      { id: 'corridor', label: '走廊', short_label: '廊', group: 'primary', tone: 'path' },
      { id: 'lookahead', label: '预瞄', short_label: '瞄', group: 'primary', tone: 'path' },
      { id: 'robotPose', label: '机器人', short_label: '机', group: 'primary', tone: 'state' },
      { id: 'phaseZones', label: '阶段区', short_label: '区', group: 'primary', tone: 'risk' },
      { id: 'blocked', label: 'Keepout', short_label: '禁', group: 'primary', tone: 'risk' },
      { id: 'openSet', label: '前沿', short_label: '前', group: 'advanced', tone: 'search' },
      { id: 'expanded', label: '已探查', short_label: '展', group: 'advanced', tone: 'search' },
      { id: 'graph', label: '图结构', short_label: '图', group: 'advanced', tone: 'search' },
      { id: 'keyNodes', label: '关键点', short_label: '点', group: 'advanced', tone: 'path' },
      { id: 'tree', label: '搜索树', short_label: '树', group: 'advanced', tone: 'search' },
      { id: 'candidates', label: '候选轨迹', short_label: '轨', group: 'advanced', tone: 'search' },
      { id: 'shadows', label: '阴影', short_label: '影', group: 'advanced', tone: 'appearance' },
    ],
    layoutPresets: [
      {
        id: 'operator',
        label: '操作员',
        description: '优先看场地、机器人、路线、当前阶段和关键风险。',
        visible_displays: ['scene', 'route', 'corridor', 'lookahead', 'robotPose', 'phaseZones', 'blocked', 'shadows'],
      },
      {
        id: 'engineering',
        label: '工程',
        description: '打开 graph、搜索回放和更多调试图层。',
        visible_displays: [
          'scene',
          'route',
          'corridor',
          'lookahead',
          'robotPose',
          'phaseZones',
          'blocked',
          'openSet',
          'expanded',
          'graph',
          'keyNodes',
          'tree',
          'candidates',
          'shadows',
        ],
      },
      {
        id: 'diagnostic',
        label: '诊断',
        description: '保留场地主视图，但更关注诊断、定位和机构状态。',
        visible_displays: ['scene', 'robotPose', 'phaseZones', 'blocked', 'shadows'],
      },
    ],
    defaults: {
      startNode: 'node-a',
      goalNode: 'node-b',
    },
  };
}

function createPreviewResponse(): SurfaceRoutePreviewResponse {
  return {
    success: true,
    failure_code: '',
    failure_reason: '',
    projected_start_node_id: 'sf_start',
    projected_goal_node_id: 'sf_goal',
    projected_start: { x: 0, y: 0, z: 0.1, yaw: 0 },
    projected_goal: { x: 1, y: 1, z: 0.3, yaw: 0 },
    path_points: [
      { x: 0, y: 0, z: 0.1, yaw: 0 },
      { x: 1, y: 1, z: 0.3, yaw: 0 },
    ],
    segments: [
      {
        segment_id: 'seg-1',
        from_node_id: 'sf_start',
        to_node_id: 'sf_goal',
        motion_type: 'ramp_up',
        required_mode: 'normal',
        point_count: 2,
      },
    ],
    team: 'blue',
    surface_graph_file: 'surface.yaml',
    planning_timing_ms: {
      surfaceProjection: 18.5,
      surfacePlanning: 24.25,
      surfacePathExpand: 8.25,
      surfaceSegmentBuild: 4.87,
      surfaceCompletePlanning: 55.87,
      surfaceRouteCli: 44.75,
    },
    planning_logs: [
      {
        stage: 'request',
        level: 'info',
        title: '收到路线请求',
        message: '已准备表面图投影与路径规划输入',
        elapsed_ms: null,
        fields: [],
      },
      {
        stage: 'surface_route_cli',
        level: 'info',
        title: '表面路线预览',
        message: '已完成点击点投影并生成路线',
        elapsed_ms: 44.75,
        fields: [],
      },
    ],
  };
}

function createTraceResponse(): SurfaceRouteTraceFromNodesResponse {
  return {
    success: true,
    failure_code: '',
    failure_reason: '',
    projected_start_node_id: 'sf_start',
    projected_goal_node_id: 'sf_goal',
    team: 'blue',
    surface_graph_file: 'surface.yaml',
    planning_timing_ms: {
      tracePlanning: 45.5,
      plannerTraceCli: 82.25,
    },
    planning_logs: [
      {
        stage: 'planner_trace_cli',
        level: 'info',
        title: '搜索回放生成',
        message: '已按运行时启发式导出搜索回放',
        elapsed_ms: 82.25,
        fields: [],
      },
    ],
    summary: {
      totalCost: 3.25,
      framesCount: 2,
      returnedFramesCount: 2,
      framesSampled: false,
      projectedStartNodeId: 'sf_start',
      projectedGoalNodeId: 'sf_goal',
      tracePlanningMs: 45.5,
      traceElapsedMs: 82.25,
    },
    node_poses: {
      sf_start: { x: 0, y: 0, z: 0.1, yaw: 0 },
      sf_goal: { x: 1, y: 1, z: 0.3, yaw: 0 },
    },
    frames: [
      {
        stepIndex: 0,
        algorithm: 'astar',
        phase: 'init',
        label: 'planner initialized',
        robotPose: null,
        openSet: [{ nodeId: 'sf_start', gCost: 0, fCost: 1 }],
        expandedNodes: [],
        bestPath: { nodeIds: ['sf_start'] },
        treeSegments: [],
        candidateTrajectories: [],
        selectedTrajectory: [],
        metrics: { gCost: 0, fCost: 1, stepCost: 0, traceMode: 'surface_route' },
      },
      {
        stepIndex: 1,
        algorithm: 'astar',
        phase: 'goal',
        label: 'goal reached',
        robotPose: { x: 1, y: 1, z: 0.3, yaw: 0 },
        openSet: [],
        expandedNodes: [{ nodeId: 'sf_goal' }],
        bestPath: { nodeIds: ['sf_start', 'sf_goal'] },
        treeSegments: [],
        candidateTrajectories: [],
        selectedTrajectory: [],
        metrics: { gCost: 1.25, fCost: 1.25, stepCost: 1.25, traceMode: 'surface_route' },
      },
    ],
  };
}

function createLiveSnapshot(): LiveEvent {
  return {
    type: 'live_state',
    timestamp: 1712385600,
    routePath: [{ x: 0, y: 0, z: 0, yaw: 0 }],
    corridorPath: [{ x: 0.2, y: 0.1, z: 0, yaw: 0 }],
    localPlannerPreviewPath: [{ x: 0.3, y: 0.1, z: 0, yaw: 0 }],
    controlState: {
      pose: { x: 0.4, y: 0.2, z: 0.1, yaw: 0.15 },
      linear: { x: 0.1, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0.2 },
    },
    activeEdge: 'edge-a',
    gateStatus: 'open',
    blockOverlay: [{ gridId: 7, state: 2, confidence: 0.9, keepoutActive: true }],
    motionModeState: {
      activeMode: 'surface_route',
      reason: 'follow corridor',
      stopRequired: false,
      timedOut: false,
      maxLinearSpeed: 0.8,
      maxAngularSpeed: 1.2,
    },
    trackingState: {
      corridorId: 'corridor-a',
      edgeId: 'edge-a',
      status: 'tracking',
      terminal: false,
      distanceToGoal: 1.2,
      reason: '',
      cmd: { vx: 0.1, vy: 0, wz: 0.1 },
    },
    localPlannerState: {
      corridorId: 'corridor-a',
      edgeId: 'edge-a',
      status: 'running',
      terminal: false,
      observeOnly: false,
      semanticRevision: 3,
      cmd: { vx: 0.1, vy: 0, wz: 0.1 },
      bestScore: 1.2,
      clearanceMarginM: 0.3,
      reason: 'clear',
    },
    recoveryState: {
      corridorId: 'corridor-a',
      edgeId: 'edge-a',
      recoveryName: 'none',
      status: 'idle',
      terminal: false,
      elapsedSec: 0,
      reason: '',
    },
    semanticSummary: {
      revision: 3,
      terrainAvailable: true,
      keepoutAvailable: true,
      blockedCells: 4,
      slowCells: 2,
      maxObstacleProbability: 0.5,
      maxDropProbability: 0.2,
      activeSources: ['terrain_grid'],
      activeReasons: ['keepout'],
    },
    localizationHealth: {
      level: 1,
      reason: 'imu warmup',
      localizationState: 'tracking',
      controlDegraded: false,
      sigmaXy: 0.03,
      sigmaYaw: 0.02,
    },
    localizationBackendStatus: {
      optimizerReady: true,
      optimizerState: 'healthy',
      graphHealth: 0.96,
      loopCandidateCount: 2,
      acceptedLoopCount: 1,
      acceptedAnchorCount: 4,
      imuSpike: false,
    },
    operatorStatus: {
      overallLevel: 1,
      overallReason: '等待定位收敛',
      localizationLevel: 1,
      localizationReason: 'sigma warmup',
      controllerLevel: 0,
      navSafetyLevel: 0,
      terrainLevel: 0,
      keepoutLevel: 1,
      mechanismLevel: 0,
      activeEventCodes: ['LOC_WARN'],
      topicTimeoutCount: 0,
    },
    visualizationEvents: [
      {
        code: 'LOC_WARN',
        severity: 2,
        title: '事件提醒',
        detail: '定位尚未完全收敛',
        sourceSignal: '/localization/health',
        recommendation: '观察 2 秒',
        active: true,
      },
    ],
    mechanismState: {
      tipState: 1,
      halOpen: false,
      lockedTipSlot: 2,
      assembledCount: 3,
      lastErrorCode: 0,
      cmdElapsedMs: 18,
      ackTimeoutCount: 0,
      reconnectCount: 0,
      parseErrorCount: 0,
      avgRttMs: 4.5,
      commHealthLevel: 0,
    },
    btSnapshot: {
      tickSeq: 8,
      treeStatus: 1,
      tickDurationMs: 12.4,
      activeSubtreeId: 'MFAreaTree',
      runningPathUids: [101, 102],
    },
    btEvents: [
      {
        uid: 101,
        nodeName: 'NavigateSurfaceRoute',
        fullPath: 'Root/Planner/NavigateSurfaceRoute',
        status: 1,
        prevStatus: 0,
      },
    ],
  };
}

function createLocalPlannerTrace(): LocalPlannerTraceResponse {
  return {
    success: true,
    snapshotLabel: 'pass_straight',
    snapshot_file: 'scenario.yaml',
    traceMode: 'local_planner',
    result: {
      status: 'ok',
      reason: '',
      hasSolution: true,
      blockedByKeepout: false,
      blockedByTerrain: false,
      shouldRotateRecovery: false,
      cmd: { vx: 0.1, vy: 0, wz: 0 },
      bestScore: 1.0,
      clearanceMarginM: 0.2,
    },
    summary: {
      candidateCount: 3,
      linearLimit: 0.8,
      angularLimit: 1.2,
      preferredLinearSpeed: 0.5,
      currentPathDistance: 1.2,
      goalHeadingError: 0.1,
      semanticRevision: 3,
      finalStatus: 'ok',
      finalReason: '',
    },
    frames: [],
  };
}

async function pickStartAndGoal() {
  fireEvent.click(screen.getByRole('button', { name: '设起点' }));
  await act(async () => {
    (lastSceneCanvasProps?.onPickWorld as ((pose: { x: number; y: number; z: number; yaw: number }) => void))?.({
      x: 0,
      y: 0,
      z: 0.1,
      yaw: 0,
    });
  });

  fireEvent.click(screen.getByRole('button', { name: '设终点' }));
  await act(async () => {
    (lastSceneCanvasProps?.onPickWorld as ((pose: { x: number; y: number; z: number; yaw: number }) => void))?.({
      x: 1,
      y: 1,
      z: 0.3,
      yaw: 0,
    });
  });
}

describe('App', () => {
  afterEach(() => {
    cleanup();
  });

  beforeEach(() => {
    vi.clearAllMocks();
    lastSceneCanvasProps = null;
    vi.mocked(fetchSceneManifest).mockResolvedValue(createSceneManifest());
    vi.mocked(previewSurfaceRoute).mockResolvedValue(createPreviewResponse());
    vi.mocked(traceSurfaceRouteFromNodes).mockResolvedValue(createTraceResponse());
    vi.mocked(executeSurfaceRoute).mockResolvedValue({
      accepted: true,
      preview: createPreviewResponse(),
    });
    vi.mocked(fetchLocalPlannerScenarios).mockResolvedValue([
      { name: 'pass_straight', label: '直行通过', snapshot_file: 'scenario.yaml' },
    ]);
    vi.mocked(traceLocalPlannerScenario).mockResolvedValue(createLocalPlannerTrace());
    vi.mocked(startLiveBridge).mockResolvedValue({
      status: 'running',
      namespace: '',
      snapshot: createLiveSnapshot(),
    });
    useSimStore.setState({
      ...initialState,
      layers: { ...initialState.layers },
    });
  });

  it('renders the visualization platform layout and live status panels', async () => {
    render(<App />);

    await screen.findByRole('heading', { name: 'RC26 全局比赛场地闭环可视化平台' });
    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });

    expect(screen.getByText('RC26 Visualization')).toBeTruthy();
    expect(screen.getByRole('button', { name: '操作员' })).toBeTruthy();
    expect(screen.getByRole('button', { name: '工程' })).toBeTruthy();
    expect(screen.getByRole('button', { name: '诊断' })).toBeTruthy();
    expect(screen.getByText('当前布局: 操作员')).toBeTruthy();
    expect(screen.getAllByText('平台状态').length).toBeGreaterThan(0);
    expect(screen.getByText('定位健康')).toBeTruthy();
    expect(screen.getAllByText('诊断事件').length).toBeGreaterThan(0);
    expect(screen.getByTestId('summary-strip')).toBeTruthy();
    expect(screen.getByTestId('inspector-grid')).toBeTruthy();
  });

  it('switches layout presets from the manifest', async () => {
    render(<App />);

    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });

    const routeButton = screen.getByRole('button', { name: '路线' });
    expect(routeButton.getAttribute('aria-pressed')).toBe('true');

    fireEvent.click(screen.getByRole('button', { name: '诊断' }));

    await waitFor(() => {
      expect(routeButton.getAttribute('aria-pressed')).toBe('false');
      expect(screen.getByText('当前布局: 诊断')).toBeTruthy();
    });
  });

  it('records picks, generates replay, and hydrates compact trace node poses', async () => {
    render(<App />);

    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });
    await pickStartAndGoal();

    fireEvent.click(screen.getByRole('button', { name: '生成三维路线' }));

    await waitFor(() => {
      expect(previewSurfaceRoute).toHaveBeenCalledTimes(1);
      expect(traceSurfaceRouteFromNodes).toHaveBeenCalledTimes(1);
    });

    expect(screen.getByText('表面起点采样点')).toBeTruthy();
    expect(screen.getByText('表面终点采样点')).toBeTruthy();
    expect(screen.getAllByText('2 / 2').length).toBeGreaterThan(0);
    expect(screen.getAllByText('55.87 毫秒').length).toBeGreaterThan(0);
    expect(screen.getAllByText('82.25 毫秒').length).toBeGreaterThan(0);
    expect(screen.getByRole('button', { name: '执行当前路线' }).getAttribute('disabled')).toBeNull();
    expect((lastSceneCanvasProps?.frame as { bestPath?: { points?: unknown[] } } | null)?.bestPath?.points).toHaveLength(2);
  });

  it('surfaces localized request failures without leaking raw english text', async () => {
    vi.mocked(previewSurfaceRoute).mockRejectedValueOnce(new Error('Failed to fetch'));

    render(<App />);

    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });
    await pickStartAndGoal();

    fireEvent.click(screen.getByRole('button', { name: '生成三维路线' }));

    await waitFor(() => {
      expect(screen.getByText('浏览器到规划服务的请求失败')).toBeTruthy();
    });

    expect(screen.getByText('三维路线生成失败: 浏览器到规划服务的请求失败')).toBeTruthy();
    expect(screen.queryByText(/Failed to fetch/)).toBeNull();
    expect(screen.queryByText(/Error:/)).toBeNull();
  });
});
