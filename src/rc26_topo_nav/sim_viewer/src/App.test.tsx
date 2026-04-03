// @vitest-environment jsdom

import React from 'react';
import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';

import { useSimStore } from './store';
import type { SceneManifest, SurfaceRouteTraceResponse } from './types';

let lastSceneCanvasProps: Record<string, unknown> | null = null;

vi.mock('./components/SceneCanvas', () => ({
  SceneCanvas: (props: Record<string, unknown>) => {
    lastSceneCanvasProps = props;
    return <div data-testid="scene-canvas">scene-canvas</div>;
  },
}));

vi.mock('./api', () => ({
  fetchSceneManifest: vi.fn(),
  traceSurfaceRoute: vi.fn(),
  executeSurfaceRoute: vi.fn(),
}));

import App from './App';
import { executeSurfaceRoute, fetchSceneManifest, traceSurfaceRoute } from './api';

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
    meilinSlots: [],
    cameraPresets: [
      {
        id: 'orbit',
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
    defaults: {
      startNode: 'node-a',
      goalNode: 'node-b',
    },
  };
}

function createTraceResponse(): SurfaceRouteTraceResponse {
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
    summary: {
      totalCost: 3.25,
      framesCount: 2,
      returnedFramesCount: 2,
      framesSampled: false,
      projectedStartNodeId: 'sf_start',
      projectedGoalNodeId: 'sf_goal',
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

describe('App', () => {
  afterEach(() => {
    cleanup();
  });

  beforeEach(() => {
    vi.clearAllMocks();
    lastSceneCanvasProps = null;
    vi.mocked(fetchSceneManifest).mockResolvedValue(createSceneManifest());
    vi.mocked(traceSurfaceRoute).mockResolvedValue(createTraceResponse());
    vi.mocked(executeSurfaceRoute).mockResolvedValue({
      accepted: true,
      preview: createTraceResponse(),
    });
    useSimStore.setState({
      ...initialState,
      layers: { ...initialState.layers },
    });
  });

  it('renders the single-purpose 3d route observer layout', async () => {
    render(<App />);

    await screen.findByRole('heading', { name: '3D 路线观察台' });
    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });

    expect(screen.getByRole('button', { name: '设起点' })).toBeTruthy();
    expect(screen.getByRole('button', { name: '设终点' })).toBeTruthy();
    expect(screen.getByRole('button', { name: '生成 3D 路线' })).toBeTruthy();
    expect(screen.queryByRole('button', { name: '高级 / 调试' })).toBeNull();
    expect(screen.getByRole('button', { name: '场景' })).toBeTruthy();
    expect(screen.getByRole('button', { name: '前沿' })).toBeTruthy();
    expect(screen.getByRole('button', { name: '已探查' })).toBeTruthy();
    expect(screen.queryByRole('button', { name: '搜索树' })).toBeNull();
    expect(screen.getByText('RC26 表面路线')).toBeTruthy();
    expect(screen.getByText('表面 A* 三维路线')).toBeTruthy();
  });

  it('records picks, generates a trace, and allows slider replay', async () => {
    render(<App />);

    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });

    fireEvent.click(screen.getByRole('button', { name: '设起点' }));
    await waitFor(() => {
      expect(lastSceneCanvasProps?.pickMode).toBe('surface_start');
    });
    await act(async () => {
      (lastSceneCanvasProps?.onPickWorld as ((pose: { x: number; y: number; z: number; yaw: number }) => void))?.({
        x: 0,
        y: 0,
        z: 0.1,
        yaw: 0,
      });
    });

    fireEvent.click(screen.getByRole('button', { name: '设终点' }));
    await waitFor(() => {
      expect(lastSceneCanvasProps?.pickMode).toBe('surface_goal');
    });
    await act(async () => {
      (lastSceneCanvasProps?.onPickWorld as ((pose: { x: number; y: number; z: number; yaw: number }) => void))?.({
        x: 1,
        y: 1,
        z: 0.3,
        yaw: 0,
      });
    });

    await waitFor(() => {
      expect(screen.getByRole('button', { name: '生成 3D 路线' }).getAttribute('disabled')).toBeNull();
    });

    fireEvent.click(screen.getByRole('button', { name: '生成 3D 路线' }));

    await waitFor(() => {
      expect(traceSurfaceRoute).toHaveBeenCalledTimes(1);
    });

    expect(screen.getByText('sf_start')).toBeTruthy();
    expect(screen.getByText('sf_goal')).toBeTruthy();
    expect(screen.getByDisplayValue('1')).toBeTruthy();
    expect(screen.getAllByText('2 / 2').length).toBeGreaterThan(0);
    expect((lastSceneCanvasProps?.frame as { bestPath?: { points?: unknown[] } } | null)?.bestPath?.points).toHaveLength(2);

    fireEvent.change(screen.getByLabelText('回放帧'), { target: { value: '0' } });

    await waitFor(() => {
      expect(screen.getAllByText('1 / 2').length).toBeGreaterThan(0);
      expect(screen.getAllByText('初始化前沿').length).toBeGreaterThan(0);
    });
  });

  it('maps raw planner english messages to chinese labels', async () => {
    render(<App />);

    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });

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

    fireEvent.click(screen.getByRole('button', { name: '生成 3D 路线' }));

    await waitFor(() => {
      expect(traceSurfaceRoute).toHaveBeenCalledTimes(1);
    });

    expect(screen.getAllByText('到达目标').length).toBeGreaterThan(0);
    expect(screen.queryByText('goal reached')).toBeNull();
  });

  it('shows sampled trace counts when the adapter returns a compressed replay', async () => {
    const sampledResponse = createTraceResponse();
    sampledResponse.summary.framesCount = 1200;
    sampledResponse.summary.returnedFramesCount = 2;
    sampledResponse.summary.framesSampled = true;
    vi.mocked(traceSurfaceRoute).mockResolvedValue(sampledResponse);

    render(<App />);

    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });

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

    fireEvent.click(screen.getByRole('button', { name: '生成 3D 路线' }));

    await waitFor(() => {
      expect(screen.getByText('已生成 2 / 1200 帧搜索回放')).toBeTruthy();
    });

    expect(screen.getAllByText('2 / 2（原始 1200 帧）').length).toBeGreaterThan(0);
  });
});
