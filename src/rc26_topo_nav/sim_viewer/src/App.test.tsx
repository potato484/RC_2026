// @vitest-environment jsdom

import React from 'react';
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';

import { useSimStore } from './store';
import type { SceneManifest } from './types';

let lastSceneCanvasProps: Record<string, unknown> | null = null;

vi.mock('./components/SceneCanvas', () => ({
  SceneCanvas: (props: Record<string, unknown>) => {
    lastSceneCanvasProps = props;
    return <div data-testid="scene-canvas">blocked:{String((props.blockedGridIds as number[]).join(','))}</div>;
  },
}));

vi.mock('./api', () => ({
  fetchSceneManifest: vi.fn(),
  createRun: vi.fn(),
  controlRun: vi.fn(),
  openLiveSocket: vi.fn(() => ({ close: vi.fn() })),
  openRunSocket: vi.fn(() => ({ close: vi.fn() })),
  startLive: vi.fn(),
  mapGoalPayload: (goalKind: string, goalValue: string) => (
    goalKind === 'task'
      ? { goal_task: goalValue }
      : goalKind === 'route'
        ? { goal_route: goalValue }
        : { goal_node: goalValue }
  ),
}));

import App from './App';
import { fetchSceneManifest } from './api';

const initialState = useSimStore.getState();

function createSceneManifest(): SceneManifest {
  return {
    meta: {
      team: 'blue',
      graph_file: 'graph.yaml',
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
        pose: { x: 0, y: 0, z: 0, yaw: 0 },
      },
      {
        id: 'node-b',
        type: 'mf_edge_pose',
        block_id: 7,
        base_cost: 0,
        operation_tag: '',
        pose: { x: 1, y: 0, z: 0, yaw: 0 },
      },
    ],
    graphEdges: [],
    tasks: [{ task_tag: 'task-a', candidate_nodes: ['node-b'] }],
    routes: [{ route_tag: 'route-a', nodes: ['node-a', 'node-b'] }],
    meilinSlots: [{ block_id: 7, x: 1, y: 0, z: 0 }],
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
        id: 'first_person',
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
      goalNode: 'node-a',
    },
  };
}

describe('App', () => {
  afterEach(() => {
    cleanup();
  });

  beforeEach(() => {
    lastSceneCanvasProps = null;
    vi.mocked(fetchSceneManifest).mockResolvedValue(createSceneManifest());
    useSimStore.setState({
      ...initialState,
      layers: { ...initialState.layers },
    });
  });

  it('defaults to a canvas-first astar layout with only contextual layer controls visible', async () => {
    render(<App />);

    await screen.findByRole('heading', { name: '3D 战术观测沙盘' });
    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });

    expect(screen.getByRole('button', { name: '场景' })).toBeTruthy();
    expect(screen.getByRole('button', { name: '拓扑' })).toBeTruthy();
    expect(screen.getByRole('button', { name: '路径节点' })).toBeTruthy();
    expect(screen.queryByRole('button', { name: '搜索树' })).toBeNull();
    expect(screen.queryByLabelText('起点节点')).toBeNull();
  });

  it('opens the debug panel and maps blocked node input to offline blocked grid ids', async () => {
    render(<App />);

    await waitFor(() => {
      expect(lastSceneCanvasProps).not.toBeNull();
    });

    fireEvent.click(screen.getByRole('button', { name: '高级 / 调试' }));

    const blockedInput = await screen.findByLabelText('阻塞节点');
    fireEvent.change(blockedInput, { target: { value: 'node-b' } });

    await waitFor(() => {
      expect(screen.getByLabelText('起点节点')).toBeTruthy();
      expect(lastSceneCanvasProps?.blockedGridIds).toEqual([7]);
    });
  });
});
