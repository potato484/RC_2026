import { describe, expect, it } from 'vitest';

import { deriveBlockedGridIds, deriveLayerControls, parseBlockedNodes } from './layerModel';
import type { SceneManifest } from './types';

const baseLayers = {
  scene: true,
  graph: false,
  keyNodes: false,
  openSet: false,
  expanded: false,
  tree: false,
  candidates: false,
  shadows: true,
  blocked: true,
};

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
        block_id: 7,
        base_cost: 0,
        operation_tag: '',
        pose: { x: 0, y: 0, z: 0, yaw: 0 },
      },
    ],
    graphEdges: [],
    tasks: [],
    routes: [],
    meilinSlots: [{ block_id: 7, x: 0, y: 0, z: 0 }],
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

describe('layerModel', () => {
  it('parses blocked nodes from comma separated text', () => {
    expect(parseBlockedNodes(' node-a, node-b ,,node-c ')).toEqual(['node-a', 'node-b', 'node-c']);
  });

  it('maps offline blocked node ids to renderable blocked grid ids', () => {
    expect(
      deriveBlockedGridIds({
        scene: createSceneManifest(),
        mode: 'offline-sim',
        liveEvent: null,
        blockedNodeIds: ['node-a'],
      }),
    ).toEqual([7]);
  });

  it('keeps only astar-related overlays visible in offline astar context', () => {
    const controls = deriveLayerControls({
      layers: baseLayers,
      mode: 'offline-sim',
      algorithm: 'astar',
      frame: null,
      blockedGridIds: [],
    });

    expect(controls.filter((control) => control.visible).map((control) => control.key)).toEqual([
      'scene',
      'graph',
      'keyNodes',
      'openSet',
      'expanded',
      'blocked',
      'shadows',
    ]);
    expect(controls.find((control) => control.key === 'tree')?.visible).toBe(false);
    expect(controls.find((control) => control.key === 'candidates')?.visible).toBe(false);
    expect(controls.find((control) => control.key === 'openSet')?.enabled).toBe(false);
  });

  it('switches contextual overlays when the frame algorithm changes to rrt or dwa', () => {
    const rrtControls = deriveLayerControls({
      layers: baseLayers,
      mode: 'offline-sim',
      algorithm: 'astar',
      frame: {
        stepIndex: 0,
        algorithm: 'rrt',
        phase: 'tree_expand',
        label: 'rrt',
        robotPose: null,
        openSet: [],
        expandedNodes: [],
        bestPath: { nodeIds: [], points: [] },
        treeSegments: [{ from: { x: 0, y: 0, z: 0, yaw: 0 }, to: { x: 1, y: 0, z: 0, yaw: 0 } }],
        candidateTrajectories: [],
        selectedTrajectory: [],
        metrics: {},
      },
      blockedGridIds: [],
    });
    const dwaControls = deriveLayerControls({
      layers: baseLayers,
      mode: 'offline-sim',
      algorithm: 'astar',
      frame: {
        stepIndex: 0,
        algorithm: 'dwa',
        phase: 'tracking',
        label: 'dwa',
        robotPose: null,
        openSet: [],
        expandedNodes: [],
        bestPath: { nodeIds: [], points: [] },
        treeSegments: [],
        candidateTrajectories: [{ points: [{ x: 0, y: 0, z: 0, yaw: 0 }], selected: true }],
        selectedTrajectory: [],
        metrics: {},
      },
      blockedGridIds: [],
    });

    expect(rrtControls.find((control) => control.key === 'tree')).toMatchObject({ visible: true, enabled: true });
    expect(rrtControls.find((control) => control.key === 'openSet')).toMatchObject({ visible: false });
    expect(dwaControls.find((control) => control.key === 'candidates')).toMatchObject({ visible: true, enabled: true });
  });

  it('shows blocked layer in live mode when live overlay has blocked cells', () => {
    const controls = deriveLayerControls({
      layers: baseLayers,
      mode: 'live-ros',
      algorithm: 'astar',
      frame: null,
      blockedGridIds: [3],
    });

    expect(controls.filter((control) => control.visible).map((control) => control.key)).toEqual([
      'scene',
      'graph',
      'blocked',
      'shadows',
    ]);
    expect(controls.find((control) => control.key === 'blocked')).toMatchObject({ enabled: true });
  });
});
