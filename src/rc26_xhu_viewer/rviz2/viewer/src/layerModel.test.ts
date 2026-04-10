import { describe, expect, it } from 'vitest';

import { deriveLayerControls } from './layerModel';
import type { SceneManifest } from './types';

const baseLayers = {
  scene: true,
  route: true,
  corridor: true,
  lookahead: true,
  robotPose: true,
  phaseZones: true,
  blocked: true,
  graph: false,
  keyNodes: false,
  openSet: false,
  expanded: false,
  tree: false,
  candidates: false,
  shadows: true,
};

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
    cameraPresets: [],
    displayCatalog: [
      { id: 'scene', label: '场地', short_label: '场', group: 'primary', tone: 'scene' },
      { id: 'route', label: '路线', short_label: '线', group: 'primary', tone: 'path' },
      { id: 'robotPose', label: '机器人', short_label: '机', group: 'primary', tone: 'state' },
      { id: 'phaseZones', label: '阶段区', short_label: '区', group: 'primary', tone: 'risk' },
      { id: 'openSet', label: '前沿', short_label: '前', group: 'advanced', tone: 'search' },
      { id: 'expanded', label: '已探查', short_label: '展', group: 'advanced', tone: 'search' },
      { id: 'shadows', label: '阴影', short_label: '影', group: 'advanced', tone: 'appearance' },
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
    layoutPresets: [],
    defaults: {
      startNode: 'node-a',
      goalNode: 'node-b',
    },
  };
}

describe('layerModel', () => {
  it('uses scene display catalog order and labels', () => {
    const controls = deriveLayerControls({
      layers: baseLayers,
      frame: null,
      scene: createSceneManifest(),
      liveEvent: null,
    });

    expect(controls.map((control) => control.key)).toEqual([
      'scene',
      'route',
      'robotPose',
      'phaseZones',
      'openSet',
      'expanded',
      'shadows',
    ]);
    expect(controls[0]).toMatchObject({ label: '场地', shortLabel: '场', group: 'primary' });
    expect(controls[2]).toMatchObject({ tone: 'state' });
  });

  it('disables frontier and expanded toggles until a trace exists', () => {
    const controls = deriveLayerControls({
      layers: baseLayers,
      frame: null,
      scene: createSceneManifest(),
      liveEvent: null,
    });

    expect(controls.find((control) => control.key === 'openSet')).toMatchObject({ enabled: false });
    expect(controls.find((control) => control.key === 'expanded')).toMatchObject({ enabled: false });
    expect(controls.find((control) => control.key === 'phaseZones')).toMatchObject({ enabled: true });
  });

  it('enables frontier and expanded toggles once trace data is available', () => {
    const controls = deriveLayerControls({
      layers: baseLayers,
      frame: {
        stepIndex: 0,
        algorithm: 'astar',
        phase: 'init',
        label: '初始化前沿',
        robotPose: null,
        openSet: [{ nodeId: 'sf-1', pose: { x: 0, y: 0, z: 0, yaw: 0 }, gCost: 0, fCost: 1 }],
        expandedNodes: [{ nodeId: 'sf-2', pose: { x: 1, y: 0, z: 0, yaw: 0 } }],
        bestPath: { nodeIds: [], points: [] },
        treeSegments: [],
        candidateTrajectories: [],
        selectedTrajectory: [],
        metrics: {},
      },
      scene: createSceneManifest(),
      liveEvent: null,
    });

    expect(controls.find((control) => control.key === 'openSet')).toMatchObject({ enabled: true });
    expect(controls.find((control) => control.key === 'expanded')).toMatchObject({ enabled: true });
  });

  it('keeps frontier and expanded toggles enabled when cumulative trace points exist outside the current frame', () => {
    const controls = deriveLayerControls({
      layers: baseLayers,
      frame: {
        stepIndex: 1,
        algorithm: 'astar',
        phase: 'goal',
        label: '到达目标',
        robotPose: null,
        openSet: [],
        expandedNodes: [],
        bestPath: { nodeIds: [], points: [] },
        treeSegments: [],
        candidateTrajectories: [],
        selectedTrajectory: [],
        metrics: {},
      },
      scene: createSceneManifest(),
      liveEvent: null,
      traceOpenSetCount: 2,
      traceExpandedCount: 3,
    });

    expect(controls.find((control) => control.key === 'openSet')).toMatchObject({ enabled: true });
    expect(controls.find((control) => control.key === 'expanded')).toMatchObject({ enabled: true });
  });
});
