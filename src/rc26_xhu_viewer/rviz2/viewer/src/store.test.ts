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
    viewerMeta: {
      viewer_title: 'RC26 全局比赛场地闭环可视化平台',
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
    displayCatalog: [
      { id: 'scene', label: '场地', short_label: '场', group: 'primary', tone: 'scene' },
      { id: 'route', label: '路线', short_label: '线', group: 'primary', tone: 'path' },
      { id: 'robotPose', label: '机器人', short_label: '机', group: 'primary', tone: 'state' },
      { id: 'phaseZones', label: '阶段区', short_label: '区', group: 'primary', tone: 'risk' },
      { id: 'blocked', label: 'Keepout', short_label: '禁', group: 'primary', tone: 'risk' },
      { id: 'shadows', label: '阴影', short_label: '影', group: 'advanced', tone: 'appearance' },
    ],
    layoutPresets: [
      {
        id: 'operator',
        label: '操作员',
        description: '默认态',
        visible_displays: ['scene', 'route', 'robotPose', 'phaseZones', 'blocked', 'shadows'],
      },
      {
        id: 'diagnostic',
        label: '诊断',
        description: '诊断态',
        visible_displays: ['scene', 'robotPose', 'blocked'],
      },
    ],
    semanticZones: [],
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

  it('starts in an operator layout with live runtime layers enabled', () => {
    expect(initialState.layers.scene).toBe(true);
    expect(initialState.layers.route).toBe(true);
    expect(initialState.layers.robotPose).toBe(true);
    expect(initialState.layers.phaseZones).toBe(true);
    expect(initialState.layers.blocked).toBe(true);
    expect(initialState.layers.graph).toBe(false);
    expect(initialState.layers.tree).toBe(false);
  });

  it('toggles layers independently', () => {
    useSimStore.getState().toggleLayer('shadows');
    expect(useSimStore.getState().layers.shadows).toBe(false);

    useSimStore.getState().toggleLayer('scene');
    expect(useSimStore.getState().layers.scene).toBe(false);
  });

  it('sets the scene-ready status and reapplies the preferred preset once the manifest arrives', () => {
    useSimStore.getState().setScene(createSceneManifest());

    expect(useSimStore.getState().scene?.meta.surface_graph_file).toBe('surface.yaml');
    expect(useSimStore.getState().layoutPresetId).toBe('operator');
    expect(useSimStore.getState().layers.route).toBe(true);
    expect(useSimStore.getState().layers.graph).toBe(false);
    expect(useSimStore.getState().statusMessage).toBe(UI_LABELS.statusLoaded);
    expect(useSimStore.getState().loadingScene).toBe(false);
  });

  it('applies layout presets from the scene manifest', () => {
    useSimStore.getState().setScene(createSceneManifest());
    useSimStore.getState().applyLayoutPreset('diagnostic');

    expect(useSimStore.getState().layoutPresetId).toBe('diagnostic');
    expect(useSimStore.getState().layers.scene).toBe(true);
    expect(useSimStore.getState().layers.route).toBe(false);
    expect(useSimStore.getState().layers.robotPose).toBe(true);
    expect(useSimStore.getState().layers.shadows).toBe(false);
  });

  it('stores explicit status messages from route actions', () => {
    useSimStore.getState().setStatusMessage('3D 路线已生成');
    expect(useSimStore.getState().statusMessage).toBe('3D 路线已生成');
  });

  it('can force a route layer visible without toggling unrelated layers', () => {
    useSimStore.getState().setScene(createSceneManifest());
    useSimStore.getState().applyLayoutPreset('diagnostic');
    useSimStore.getState().setLayerVisible('route', true);

    expect(useSimStore.getState().layers.route).toBe(true);
    expect(useSimStore.getState().layers.robotPose).toBe(true);
    expect(useSimStore.getState().layers.blocked).toBe(true);
    expect(useSimStore.getState().layers.shadows).toBe(false);
  });
});
