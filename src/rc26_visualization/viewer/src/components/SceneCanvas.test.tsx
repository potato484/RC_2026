// @vitest-environment jsdom

import React, { StrictMode } from 'react';
import { act, render, waitFor } from '@testing-library/react';
import { beforeEach, describe, expect, it, vi } from 'vitest';

vi.mock('@babylonjs/core/Engines/WebGPU/Extensions/engine.alpha', () => ({}));

vi.mock('@babylonjs/core', () => {
  const mockState = {
    pendingInitResolvers: [] as Array<() => void>,
    staticMeshNames: [] as string[],
    liveMeshNames: [] as string[],
    pointerObservers: [] as Array<(info: { type: number }) => void>,
    pickResult: null as null | { hit: boolean; pickedPoint: Vector3; pickedMesh: { name: string } },
  };

  class Vector3 {
    public x: number;
    public y: number;
    public z: number;

    constructor(x = 0, y = 0, z = 0) {
      this.x = x;
      this.y = y;
      this.z = z;
    }

    static Zero() {
      return new Vector3(0, 0, 0);
    }
  }

  class Color3 {
    constructor(public r = 1, public g = 1, public b = 1) {}
  }

  class Color4 {
    constructor(public r = 0, public g = 0, public b = 0, public a = 1) {}
  }

  class Engine {
    constructor(public canvas: HTMLCanvasElement) {}

    runRenderLoop(_cb: () => void) {}

    resize() {}

    dispose() {}

    getRenderWidth() {
      return 800;
    }

    getRenderHeight() {
      return 600;
    }
  }

  class WebGPUEngine extends Engine {
    static IsSupportedAsync = Promise.resolve(true);

    async initAsync() {
      await new Promise<void>((resolve) => {
        mockState.pendingInitResolvers.push(resolve);
      });
    }
  }

  class Scene {
    static FOGMODE_LINEAR = 1;

    public meshes: Mesh[] = [];
    public lights: Array<{ name: string; dispose: () => void }> = [];
    public activeCamera: unknown = null;
    public useRightHandedSystem = false;
    public clearColor: Color4 | null = null;
    public fogMode = 0;
    public fogColor: Color3 | null = null;
    public fogStart = 0;
    public fogEnd = 0;
    public pointerX = 0;
    public pointerY = 0;
    public onPointerObservable = {
      add: (callback: (info: { type: number }) => void) => {
        mockState.pointerObservers.push(callback);
        return callback;
      },
      remove: (callback: (info: { type: number }) => void) => {
        mockState.pointerObservers = mockState.pointerObservers.filter((entry) => entry !== callback);
      },
    };

    constructor(public engine: Engine) {}

    render() {}

    dispose() {}

    pick(_x: number, _y: number, predicate?: (mesh: { name?: string }) => boolean) {
      if (!mockState.pickResult) {
        return { hit: false, pickedPoint: null, pickedMesh: null };
      }
      if (predicate && !predicate(mockState.pickResult.pickedMesh)) {
        return { hit: false, pickedPoint: null, pickedMesh: null };
      }
      return mockState.pickResult;
    }
  }

  class HemisphericLight {
    public intensity = 0;
    private sceneRef: Scene;

    constructor(public name: string, _direction: Vector3, scene: Scene) {
      this.sceneRef = scene;
      scene.lights.push(this);
    }

    dispose() {
      this.sceneRef.lights = this.sceneRef.lights.filter((entry) => entry !== this);
    }
  }

  class DirectionalLight {
    public position: Vector3 | null = null;
    public intensity = 0;
    public diffuse: Color3 | null = null;
    private sceneRef: Scene;

    constructor(public name: string, _direction: Vector3, scene: Scene) {
      this.sceneRef = scene;
      scene.lights.push(this);
    }

    dispose() {
      this.sceneRef.lights = this.sceneRef.lights.filter((entry) => entry !== this);
    }
  }

  class TransformNode {
    public parent: TransformNode | null = null;
    public position = new Vector3();
    public rotation = new Vector3();

    constructor(public name: string, _scene: Scene) {}

    dispose() {}
  }

  class ArcRotateCamera {
    public upVector = new Vector3();
    public wheelPrecision = 0;
    public position = new Vector3();
    public target = new Vector3();

    constructor(
      public name: string,
      _alpha: number,
      _beta: number,
      _radius: number,
      _target: Vector3,
      _scene: Scene,
    ) {}

    attachControl(_canvas: HTMLCanvasElement, _attach: boolean) {}

    setPosition(position: Vector3) {
      this.position = position;
    }

    setTarget(target: Vector3) {
      this.target = target;
    }
  }

  class FreeCamera {
    public upVector = new Vector3();
    public mode: number | null = null;
    public position = new Vector3();
    public target = new Vector3();
    public orthoLeft: number | null = null;
    public orthoRight: number | null = null;
    public orthoTop: number | null = null;
    public orthoBottom: number | null = null;

    constructor(public name: string, position: Vector3, _scene: Scene) {
      this.position = position;
    }

    setTarget(target: Vector3) {
      this.target = target;
    }
  }

  class PBRMaterial {
    static MATERIAL_ALPHABLEND = 1;

    public albedoColor: Color3 | null = null;
    public alpha = 1;
    public metallic = 0;
    public roughness = 0;
    public transparencyMode: number | null = null;
    public unlit = false;

    constructor(public name: string, _scene: Scene) {}
  }

  class StandardMaterial {}

  class VertexData {
    public positions: number[] = [];
    public indices: number[] = [];
    public normals: number[] = [];

    static ComputeNormals(_positions: number[], _indices: number[], _normals: number[]) {}

    applyToMesh(mesh: Mesh) {
      mesh.hasVertexData = true;
    }
  }

  class Mesh {
    static CAP_ALL = 1;

    public position = new Vector3();
    public rotation = new Vector3();
    public material: unknown = null;
    public parent: TransformNode | null = null;
    public receiveShadows = false;
    public hasVertexData = false;
    public isPickable = false;
    private sceneRef: Scene;

    constructor(public name: string, scene: Scene) {
      this.sceneRef = scene;
      scene.meshes.push(this);
      mockState.liveMeshNames.push(name);
      if (name.startsWith('static_')) {
        mockState.staticMeshNames.push(name);
      }
    }

    dispose() {
      this.sceneRef.meshes = this.sceneRef.meshes.filter((entry) => entry !== this);
      mockState.liveMeshNames = mockState.liveMeshNames.filter((entry) => entry !== this.name);
    }
  }

  class ShadowGenerator {
    public useBlurExponentialShadowMap = false;

    constructor(_size: number, _light: DirectionalLight) {}

    addShadowCaster(_mesh: Mesh) {}

    dispose() {}
  }

  const MeshBuilder = {
    CreateSphere(name: string, _options: unknown, scene: Scene) {
      return new Mesh(name, scene);
    },
    CreateLines(name: string, _options: unknown, scene: Scene) {
      return new Mesh(name, scene);
    },
    CreateBox(name: string, _options: unknown, scene: Scene) {
      return new Mesh(name, scene);
    },
    CreateTube(name: string, _options: unknown, scene: Scene) {
      return new Mesh(name, scene);
    },
    CreateCylinder(name: string, _options: unknown, scene: Scene) {
      return new Mesh(name, scene);
    },
  };

  return {
    __mockState: mockState,
    ArcRotateCamera,
    Camera: { ORTHOGRAPHIC_CAMERA: 1 },
    Color3,
    Color4,
    DirectionalLight,
    Engine,
    FreeCamera,
    HemisphericLight,
    Mesh,
    MeshBuilder,
    PBRMaterial,
    PointerEventTypes: {
      POINTERMOVE: 1,
      POINTERPICK: 2,
    },
    Scene,
    ShadowGenerator,
    StandardMaterial,
    TransformNode,
    Vector3,
    VertexData,
    WebGPUEngine,
  };
});

import * as BABYLON from '@babylonjs/core';

import { SceneCanvas } from './SceneCanvas';
import type { SceneManifest } from '../types';

const mockState = (BABYLON as typeof BABYLON & {
  __mockState: {
    pendingInitResolvers: Array<() => void>;
    staticMeshNames: string[];
    liveMeshNames: string[];
    pointerObservers: Array<(info: { type: number }) => void>;
    pickResult: null | { hit: boolean; pickedPoint: InstanceType<typeof BABYLON.Vector3>; pickedMesh: { name: string } };
  };
}).__mockState;

const layers = {
  scene: false,
  route: true,
  corridor: true,
  lookahead: true,
  robotPose: true,
  phaseZones: true,
  blocked: false,
  graph: true,
  keyNodes: false,
  openSet: false,
  expanded: false,
  tree: false,
  candidates: false,
  shadows: false,
};

function createSceneManifest(overrides: Partial<SceneManifest> = {}): SceneManifest {
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
        block_id: 1,
        base_cost: 0,
        operation_tag: '',
        pose: { x: 1, y: 0, z: 0, yaw: 0 },
      },
    ],
    graphEdges: [],
    tasks: [],
    routes: [],
    meilinSlots: [],
    semanticZones: [],
    displayCatalog: [],
    layoutPresets: [],
    cameraPresets: [
      {
        id: 'orbit',
        kind: 'perspective',
        position: { x: 1, y: 1, z: 1, yaw: 0 },
        target: { x: 0, y: 0, z: 0, yaw: 0 },
      },
      {
        id: 'side_ortho',
        kind: 'orthographic',
        position: { x: 3, y: 0, z: 1, yaw: 0 },
        target: { x: 0, y: 0, z: 0, yaw: 0 },
      },
    ],
    defaults: {
      startNode: 'node-a',
      goalNode: 'node-a',
    },
    ...overrides,
  };
}

describe('SceneCanvas', () => {
  beforeEach(() => {
    mockState.pendingInitResolvers.length = 0;
    mockState.staticMeshNames.length = 0;
    mockState.liveMeshNames.length = 0;
    mockState.pointerObservers.length = 0;
    mockState.pickResult = null;
  });

  it('ignores stale StrictMode engine init completions and loads the active scene', async () => {
    render(
      <StrictMode>
        <div style={{ width: '640px', height: '360px' }}>
          <SceneCanvas
            scene={createSceneManifest()}
            frame={null}
            liveEvent={null}
            viewMode="orbit"
            layers={layers}
            startPose={null}
            goalPose={null}
            hoverPose={null}
            blockedGridIds={[]}
            pickMode="idle"
          />
        </div>
      </StrictMode>,
    );

    await waitFor(() => {
      expect(mockState.pendingInitResolvers).toHaveLength(2);
    });

    await act(async () => {
      mockState.pendingInitResolvers[0]();
      await Promise.resolve();
    });

    expect(mockState.staticMeshNames).toHaveLength(0);

    await act(async () => {
      mockState.pendingInitResolvers[1]();
      await Promise.resolve();
    });

    await waitFor(() => {
      expect(mockState.staticMeshNames).toContain('static_node_node-a');
    });
  });

  it('initializes once the scene manifest arrives after the placeholder render', async () => {
    const { rerender } = render(
      <div style={{ width: '640px', height: '360px' }}>
        <SceneCanvas
          scene={null}
          frame={null}
          liveEvent={null}
          viewMode="orbit"
          layers={layers}
          startPose={null}
          goalPose={null}
          hoverPose={null}
          blockedGridIds={[]}
          pickMode="idle"
        />
      </div>,
    );

    expect(mockState.pendingInitResolvers).toHaveLength(0);

    rerender(
      <div style={{ width: '640px', height: '360px' }}>
        <SceneCanvas
          scene={createSceneManifest()}
          frame={null}
          liveEvent={null}
          viewMode="orbit"
          layers={layers}
          startPose={null}
          goalPose={null}
          hoverPose={null}
          blockedGridIds={[]}
          pickMode="idle"
        />
      </div>,
    );

    await waitFor(() => {
      expect(mockState.pendingInitResolvers).toHaveLength(1);
    });

    await act(async () => {
      mockState.pendingInitResolvers[0]();
      await Promise.resolve();
    });

    await waitFor(() => {
      expect(mockState.staticMeshNames).toContain('static_node_node-a');
    });
  });

  it('renders structural vertical scene features together with horizontal surfaces', async () => {
    const scene = createSceneManifest({
      sceneFeatures: [
        {
          id: 'platform_top',
          name: '主地图',
          material_symbol: 'ground',
          fill: '#7f8c99',
          opacity: 0.9,
          render_class: 'world-ground',
          avg_z: 0.4,
          z_span: 0,
          area_xy: 0.48,
          points: [
            { x: 0, y: 0, z: 0.4, yaw: 0 },
            { x: 1.2, y: 0, z: 0.4, yaw: 0 },
            { x: 1.2, y: 0.4, z: 0.4, yaw: 0 },
            { x: 0, y: 0.4, z: 0.4, yaw: 0 },
          ],
        },
        {
          id: 'platform_riser',
          name: '主地图',
          material_symbol: 'ground',
          fill: '#7f8c99',
          opacity: 0.9,
          render_class: 'world-ground',
          avg_z: 0.2,
          z_span: 0.4,
          area_xy: 0,
          points: [
            { x: 1.2, y: 0, z: 0, yaw: 0 },
            { x: 1.2, y: 0, z: 0.4, yaw: 0 },
            { x: 1.2, y: 0.4, z: 0.4, yaw: 0 },
            { x: 1.2, y: 0.4, z: 0, yaw: 0 },
          ],
        },
      ],
    });

    render(
      <div style={{ width: '640px', height: '360px' }}>
        <SceneCanvas
          scene={scene}
          frame={null}
          liveEvent={null}
          viewMode="orbit"
          layers={{ ...layers, scene: true, graph: false }}
          startPose={null}
          goalPose={null}
          hoverPose={null}
          blockedGridIds={[]}
          pickMode="idle"
        />
      </div>,
    );

    await waitFor(() => {
      expect(mockState.pendingInitResolvers).toHaveLength(1);
    });

    await act(async () => {
      mockState.pendingInitResolvers[0]();
      await Promise.resolve();
    });

    await waitFor(() => {
      expect(mockState.staticMeshNames).toContain('static_feat_platform_top');
      expect(mockState.staticMeshNames).toContain('static_feat_platform_riser');
    });
  });

  it('removes static topo meshes when the graph layer is toggled off', async () => {
    const { rerender } = render(
      <div style={{ width: '640px', height: '360px' }}>
        <SceneCanvas
          scene={createSceneManifest()}
          frame={null}
          liveEvent={null}
          viewMode="orbit"
          layers={layers}
          startPose={null}
          goalPose={null}
          hoverPose={null}
          blockedGridIds={[]}
          pickMode="idle"
        />
      </div>,
    );

    await waitFor(() => {
      expect(mockState.pendingInitResolvers).toHaveLength(1);
    });

    await act(async () => {
      mockState.pendingInitResolvers[0]();
      await Promise.resolve();
    });

    await waitFor(() => {
      expect(mockState.liveMeshNames).toContain('static_node_node-a');
    });

    rerender(
      <div style={{ width: '640px', height: '360px' }}>
        <SceneCanvas
          scene={createSceneManifest()}
          frame={null}
          liveEvent={null}
          viewMode="orbit"
          layers={{ ...layers, graph: false }}
          startPose={null}
          goalPose={null}
          hoverPose={null}
          blockedGridIds={[]}
          pickMode="idle"
        />
      </div>,
    );

    await waitFor(() => {
      expect(mockState.liveMeshNames).not.toContain('static_node_node-a');
      expect(mockState.liveMeshNames).not.toContain('static_node_node-b');
    });
  });

  it('snaps hover and click interactions to the nearest topo node while pick mode is active', async () => {
    const onHoverNodeChange = vi.fn();
    const onPickNode = vi.fn();

    render(
      <div style={{ width: '640px', height: '360px' }}>
        <SceneCanvas
          scene={createSceneManifest()}
          frame={null}
          liveEvent={null}
          viewMode="orbit"
          layers={layers}
          startPose={null}
          goalPose={null}
          hoverPose={null}
          blockedGridIds={[]}
          pickMode="goal"
          onHoverNodeChange={onHoverNodeChange}
          onPickNode={onPickNode}
        />
      </div>,
    );

    await waitFor(() => {
      expect(mockState.pendingInitResolvers).toHaveLength(1);
    });

    await act(async () => {
      mockState.pendingInitResolvers[0]();
      await Promise.resolve();
    });

    await waitFor(() => {
      expect(mockState.pointerObservers).toHaveLength(1);
    });

    mockState.pickResult = {
      hit: true,
      pickedPoint: new BABYLON.Vector3(0.86, 0.08, 0),
      pickedMesh: { name: 'static_feat_floor' },
    };

    act(() => {
      mockState.pointerObservers[0]({ type: BABYLON.PointerEventTypes.POINTERMOVE });
    });

    expect(onHoverNodeChange).toHaveBeenLastCalledWith('node-b');

    act(() => {
      mockState.pointerObservers[0]({ type: BABYLON.PointerEventTypes.POINTERPICK });
    });

    expect(onPickNode).toHaveBeenCalledWith('node-b');
  });

  it('renders astar dynamic overlays when contextual layers are enabled', async () => {
    render(
      <div style={{ width: '640px', height: '360px' }}>
        <SceneCanvas
          scene={createSceneManifest()}
          frame={{
            stepIndex: 1,
            algorithm: 'astar',
            phase: 'relax',
            label: 'A* frame',
            robotPose: null,
            openSet: [{ nodeId: 'node-a', pose: { x: 0, y: 0, z: 0, yaw: 0 }, gCost: 1, fCost: 2 }],
            expandedNodes: [{ nodeId: 'node-b', pose: { x: 1, y: 0, z: 0, yaw: 0 } }],
            bestPath: {
              nodeIds: ['node-a', 'node-mid', 'node-b'],
              points: [
                { x: 0, y: 0, z: 0, yaw: 0 },
                { x: 0.5, y: 0.2, z: 0, yaw: 0 },
                { x: 1, y: 0, z: 0, yaw: 0 },
              ],
            },
            treeSegments: [],
            candidateTrajectories: [],
            selectedTrajectory: [],
            metrics: {},
          }}
          liveEvent={null}
          viewMode="orbit"
          layers={{ ...layers, keyNodes: true, openSet: true, expanded: true }}
          startPose={null}
          goalPose={null}
          hoverPose={null}
          blockedGridIds={[]}
          pickMode="idle"
        />
      </div>,
    );

    await waitFor(() => {
      expect(mockState.pendingInitResolvers).toHaveLength(1);
    });

    await act(async () => {
      mockState.pendingInitResolvers[0]();
      await Promise.resolve();
    });

    await waitFor(() => {
      expect(mockState.liveMeshNames).toContain('dyn_pathnode_1');
      expect(mockState.liveMeshNames).toContain('dyn_open_node-a');
      expect(mockState.liveMeshNames).toContain('dyn_exp_node-b');
    });
  });

  it('renders blocked overlays from offline blocked grid ids without live data', async () => {
    render(
      <div style={{ width: '640px', height: '360px' }}>
        <SceneCanvas
          scene={createSceneManifest({
            meilinSlots: [{ block_id: 7, x: 0.6, y: 0.2, z: 0 }],
          })}
          frame={null}
          liveEvent={null}
          viewMode="orbit"
          layers={{ ...layers, blocked: true }}
          startPose={null}
          goalPose={null}
          hoverPose={null}
          blockedGridIds={[7]}
          pickMode="idle"
        />
      </div>,
    );

    await waitFor(() => {
      expect(mockState.pendingInitResolvers).toHaveLength(1);
    });

    await act(async () => {
      mockState.pendingInitResolvers[0]();
      await Promise.resolve();
    });

    await waitFor(() => {
      expect(mockState.liveMeshNames).toContain('dyn_blocked_7');
    });
  });

  it('renders semantic zones and live robot pose from the visualization event stream', async () => {
    render(
      <div style={{ width: '640px', height: '360px' }}>
        <SceneCanvas
          scene={createSceneManifest({
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
          })}
          frame={null}
          liveEvent={{
            type: 'live_state',
            controlState: {
              pose: { x: 0.5, y: 0.4, z: 0.2, yaw: 0.1 },
              linear: { x: 0, y: 0, z: 0 },
              angular: { x: 0, y: 0, z: 0 },
            },
            btSnapshot: {
              tickSeq: 1,
              treeStatus: 1,
              tickDurationMs: 12.5,
              activeSubtreeId: 'MFAreaTree',
              runningPathUids: [11, 12],
            },
          }}
          viewMode="orbit"
          layers={{ ...layers, graph: false, phaseZones: true, robotPose: true }}
          startPose={null}
          goalPose={null}
          hoverPose={null}
          blockedGridIds={[]}
          pickMode="idle"
        />
      </div>,
    );

    await waitFor(() => {
      expect(mockState.pendingInitResolvers).toHaveLength(1);
    });

    await act(async () => {
      mockState.pendingInitResolvers[0]();
      await Promise.resolve();
    });

    await waitFor(() => {
      expect(mockState.liveMeshNames).toContain('dyn_zone_mf_zone');
      expect(mockState.liveMeshNames).toContain('robot_cone');
    });
  });
});
