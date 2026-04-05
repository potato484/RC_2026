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
    graphNodes: [],
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
    ],
    defaults: {
      startNode: 'node-a',
      goalNode: 'node-b',
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

  it('starts in a route-observer layout with frontier overlays enabled', () => {
    expect(initialState.layers.scene).toBe(true);
    expect(initialState.layers.openSet).toBe(true);
    expect(initialState.layers.expanded).toBe(true);
    expect(initialState.layers.graph).toBe(false);
    expect(initialState.layers.tree).toBe(false);
  });

  it('toggles layers independently', () => {
    useSimStore.getState().toggleLayer('shadows');
    expect(useSimStore.getState().layers.shadows).toBe(false);

    useSimStore.getState().toggleLayer('scene');
    expect(useSimStore.getState().layers.scene).toBe(false);
  });

  it('sets the scene-ready status once the manifest arrives', () => {
    useSimStore.getState().setScene(createSceneManifest());

    expect(useSimStore.getState().scene?.meta.surface_graph_file).toBe('surface.yaml');
    expect(useSimStore.getState().statusMessage).toBe(UI_LABELS.statusLoaded);
    expect(useSimStore.getState().loadingScene).toBe(false);
  });

  it('stores explicit status messages from route actions', () => {
    useSimStore.getState().setStatusMessage('3D 路线已生成');
    expect(useSimStore.getState().statusMessage).toBe('3D 路线已生成');
  });
});
