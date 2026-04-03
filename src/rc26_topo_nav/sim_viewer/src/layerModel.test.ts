import { describe, expect, it } from 'vitest';

import { deriveLayerControls } from './layerModel';

const baseLayers = {
  scene: true,
  graph: false,
  keyNodes: false,
  openSet: true,
  expanded: true,
  tree: false,
  candidates: false,
  shadows: true,
  blocked: false,
};

describe('layerModel', () => {
  it('shows only the route-observer layers', () => {
    const controls = deriveLayerControls({
      layers: baseLayers,
      frame: null,
    });

    expect(controls.filter((control) => control.visible).map((control) => control.key)).toEqual([
      'scene',
      'openSet',
      'expanded',
      'shadows',
    ]);
  });

  it('disables frontier and expanded toggles until a trace exists', () => {
    const controls = deriveLayerControls({
      layers: baseLayers,
      frame: null,
    });

    expect(controls.find((control) => control.key === 'openSet')).toMatchObject({ enabled: false });
    expect(controls.find((control) => control.key === 'expanded')).toMatchObject({ enabled: false });
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
    });

    expect(controls.find((control) => control.key === 'openSet')).toMatchObject({ enabled: true });
    expect(controls.find((control) => control.key === 'expanded')).toMatchObject({ enabled: true });
  });
});
