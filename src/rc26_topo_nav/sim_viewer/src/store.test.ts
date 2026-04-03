import { beforeEach, describe, expect, it } from 'vitest';

import { UI_LABELS } from './labels';
import { useSimStore } from './store';
import type { SceneManifest } from './types';

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
    ],
    graphEdges: [],
    tasks: [{ task_tag: 'task-a', candidate_nodes: ['node-a'] }],
    routes: [{ route_tag: 'route-a', nodes: ['node-a'] }],
    meilinSlots: [],
    cameraPresets: [
      {
        id: 'orbit',
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

describe('useSimStore', () => {
  beforeEach(() => {
    useSimStore.setState({
      ...initialState,
      layers: { ...initialState.layers },
    });
  });

  it('toggles layers and resets run state', () => {
    useSimStore.getState().toggleLayer('graph');
    expect(useSimStore.getState().layers.graph).toBe(true);

    useSimStore.setState({
      runId: 'abc123',
      runState: 'playing',
      frameCount: 10,
      cursor: 4,
    });
    useSimStore.getState().resetRun();

    expect(useSimStore.getState().runId).toBeNull();
    expect(useSimStore.getState().frameCount).toBe(0);
    expect(useSimStore.getState().cursor).toBe(0);
  });

  it('starts in a scene-first layout with graph overlays hidden', () => {
    expect(initialState.layers.scene).toBe(true);
    expect(initialState.layers.graph).toBe(false);
    expect(initialState.layers.keyNodes).toBe(false);
    expect(initialState.layers.openSet).toBe(false);
    expect(initialState.layers.expanded).toBe(false);
    expect(initialState.layers.tree).toBe(false);
    expect(initialState.layers.candidates).toBe(false);
  });

  it('sets a manual-idle status message after the scene manifest loads', () => {
    useSimStore.getState().setScene(createSceneManifest());

    expect(useSimStore.getState().startNode).toBe('node-a');
    expect(useSimStore.getState().goalValue).toBe('node-a');
    expect(useSimStore.getState().statusMessage).toBe(UI_LABELS.statusLoaded);
  });

  it('hydrates run state from the create-run response before websocket meta arrives', () => {
    useSimStore.getState().setRunMeta('abc12345', {
      state: 'paused',
      cursor: 0,
      frameCount: 3,
      summary: { framesCount: 3, goalKind: 'node', goalValue: 'node-a' },
    });

    expect(useSimStore.getState().runId).toBe('abc12345');
    expect(useSimStore.getState().runState).toBe('paused');
    expect(useSimStore.getState().cursor).toBe(0);
    expect(useSimStore.getState().frameCount).toBe(3);
    expect(useSimStore.getState().runSummary).toEqual({
      framesCount: 3,
      goalKind: 'node',
      goalValue: 'node-a',
    });
    expect(useSimStore.getState().statusMessage).toContain('手动离线运行已创建');
  });

  it('updates run cursor from control responses even before a websocket frame arrives', () => {
    useSimStore.setState({
      ...useSimStore.getState(),
      runId: 'abc12345',
      runState: 'paused',
      frameCount: 3,
      cursor: 0,
    });

    useSimStore.getState().setRunFrame({
      state: 'paused',
      cursor: 1,
    });

    expect(useSimStore.getState().runState).toBe('paused');
    expect(useSimStore.getState().cursor).toBe(1);
    expect(useSimStore.getState().frameCount).toBe(3);
  });
});
