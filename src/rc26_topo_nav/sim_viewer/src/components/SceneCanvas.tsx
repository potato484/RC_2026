import React, { useEffect, useRef, useState } from 'react';
import * as BABYLON from '@babylonjs/core';
import '@babylonjs/core/Engines/WebGPU/Extensions/engine.alpha';

import type { LayerState } from '../store';
import { findNearestGraphNode } from '../scenePicking';
import type { CameraPreset, LiveEvent, PlannerFrame, Pose3, PickMode, SceneManifest, ViewMode } from '../types';

export interface SceneCanvasProps {
  scene: SceneManifest | null;
  frame: PlannerFrame | null;
  liveEvent: LiveEvent | null;
  viewMode: ViewMode;
  layers: LayerState;
  startPose: Pose3 | null;
  goalPose: Pose3 | null;
  hoverPose: Pose3 | null;
  pickMode: PickMode;
  onHoverNodeChange?: (nodeId: string | null) => void;
  onPickNode?: (nodeId: string) => void;
}

function hexToColor3(hex: string): BABYLON.Color3 {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result ? new BABYLON.Color3(
    parseInt(result[1], 16) / 255,
    parseInt(result[2], 16) / 255,
    parseInt(result[3], 16) / 255
  ) : new BABYLON.Color3(1, 1, 1);
}

function vec3(p: Pose3 | {x: number, y: number, z: number, world_z?: number}) {
  return new BABYLON.Vector3(p.x, p.y, p.world_z ?? p.z);
}

function poseZ(p: Pose3) {
  return p.world_z ?? p.z;
}

function normalizeColorChannel(value: number): number {
  return value > 1 ? value / 255 : value;
}

function colorTupleToColor4(values: number[] | undefined, fallback: string): BABYLON.Color4 {
  if (!values || values.length < 3) {
    const fallbackColor = hexToColor3(fallback);
    return new BABYLON.Color4(fallbackColor.r, fallbackColor.g, fallbackColor.b, 1);
  }

  return new BABYLON.Color4(
    normalizeColorChannel(values[0] ?? 0),
    normalizeColorChannel(values[1] ?? 0),
    normalizeColorChannel(values[2] ?? 0),
    normalizeColorChannel(values[3] ?? 1),
  );
}

function scaleColor3(color: BABYLON.Color3, factor: number): BABYLON.Color3 {
  return new BABYLON.Color3(color.r * factor, color.g * factor, color.b * factor);
}

class BabylonSceneManager {
  public engine: any = null;
  public scene: BABYLON.Scene | null = null;
  public isReady: boolean = false;
  
  private orbitCam!: BABYLON.ArcRotateCamera;
  private followCam!: BABYLON.FreeCamera;
  private fpCam!: BABYLON.FreeCamera;
  private topOrthoCam!: BABYLON.FreeCamera;
  private sideOrthoCam!: BABYLON.FreeCamera;

  private shadowGen: BABYLON.ShadowGenerator | null = null;
  private activeManifest: SceneManifest | null = null;
  private pickMode: PickMode = 'idle';
  private pointerObserver: any = null;
  private hoveredNodeId: string | null = null;
  private hoverCallback: ((nodeId: string | null) => void) | null = null;
  private pickCallback: ((nodeId: string) => void) | null = null;
  
  private dynamicParent!: BABYLON.TransformNode;
  private dynamicMeshes: BABYLON.Node[] = [];
  private materials = new Map<string, BABYLON.PBRMaterial | BABYLON.StandardMaterial>();

  public async init(canvas: HTMLCanvasElement, onReady: () => void) {
    const webGpuSupported = await BABYLON.WebGPUEngine.IsSupportedAsync;
    if (webGpuSupported) {
      const engine = new BABYLON.WebGPUEngine(canvas, { antialias: true });
      await engine.initAsync();
      this.engine = engine;
    } else {
      this.engine = new BABYLON.Engine(canvas, true, { preserveDrawingBuffer: true, stencil: true, premultipliedAlpha: false });
    }

    this.scene = new BABYLON.Scene(this.engine);
    this.scene.useRightHandedSystem = true;
    
    // 保持画布为不透明背景，避免部分浏览器把已渲染的 WebGL 图层错误合成为空白。
    this.scene.clearColor = colorTupleToColor4([0.92, 0.93, 0.94, 1], '#e6edf0');
    this.scene.fogMode = BABYLON.Scene.FOGMODE_LINEAR;
    this.scene.fogColor = hexToColor3('#d1d5db');
    this.scene.fogStart = 26;
    this.scene.fogEnd = 72;

    const ambient = new BABYLON.HemisphericLight("ambient", new BABYLON.Vector3(0, 0, 1), this.scene);
    ambient.intensity = 0.8;

    this.dynamicParent = new BABYLON.TransformNode("dynamic", this.scene);

    this.orbitCam = new BABYLON.ArcRotateCamera("orbit", Math.PI / 4, Math.PI / 3, 20, BABYLON.Vector3.Zero(), this.scene);
    this.orbitCam.upVector = new BABYLON.Vector3(0, 0, 1);
    this.orbitCam.attachControl(canvas, true);
    this.orbitCam.wheelPrecision = 50;

    this.followCam = new BABYLON.FreeCamera("follow", BABYLON.Vector3.Zero(), this.scene);
    this.followCam.upVector = new BABYLON.Vector3(0, 0, 1);

    this.fpCam = new BABYLON.FreeCamera("fp", BABYLON.Vector3.Zero(), this.scene);
    this.fpCam.upVector = new BABYLON.Vector3(0, 0, 1);
    
    this.topOrthoCam = new BABYLON.FreeCamera("topOrtho", new BABYLON.Vector3(0, 0, 50), this.scene);
    this.topOrthoCam.upVector = new BABYLON.Vector3(0, 1, 0); 
    this.topOrthoCam.mode = BABYLON.Camera.ORTHOGRAPHIC_CAMERA;

    this.sideOrthoCam = new BABYLON.FreeCamera("sideOrtho", new BABYLON.Vector3(50, 0, 0), this.scene);
    this.sideOrthoCam.upVector = new BABYLON.Vector3(0, 0, 1);
    this.sideOrthoCam.mode = BABYLON.Camera.ORTHOGRAPHIC_CAMERA;

    this.scene.activeCamera = this.orbitCam;

    if (this.engine) {
      this.engine.runRenderLoop(() => {
        this.scene?.render();
      });
    }

    window.addEventListener('resize', this.onResize);

    this.isReady = true;
    onReady();
  }

  private onResize = () => {
    if (this.engine) {
      this.engine.resize();
    }
  };

  public dispose() {
    window.removeEventListener('resize', this.onResize);
    if (this.scene && this.pointerObserver && this.scene.onPointerObservable?.remove) {
      this.scene.onPointerObservable.remove(this.pointerObserver);
      this.pointerObserver = null;
    }
    this.scene?.dispose();
    if (this.engine) {
      this.engine.dispose();
    }
  }

  private setupOrtho(cam: BABYLON.FreeCamera, zoom: number) {
    if (!this.engine) return;
    const aspect = this.engine.getRenderWidth() / this.engine.getRenderHeight();
    const halfWidth = zoom / 2;
    const halfHeight = halfWidth / aspect;
    cam.orthoLeft = -halfWidth;
    cam.orthoRight = halfWidth;
    cam.orthoTop = halfHeight;
    cam.orthoBottom = -halfHeight;
  }

  private applyMeshShadows(mesh: BABYLON.Mesh) {
    if (this.shadowGen) {
      this.shadowGen.addShadowCaster(mesh);
    }
    mesh.receiveShadows = true;
  }

  private createPinMarker(
    name: string,
    pose: Pose3,
    color: string,
    {
      headDiameter,
      stemDiameter,
      lift,
      opacity = 1,
      dynamic = false,
    }: {
      headDiameter: number;
      stemDiameter: number;
      lift: number;
      opacity?: number;
      dynamic?: boolean;
    },
  ) {
    if (!this.scene) return;

    const anchorZ = poseZ(pose);
    const stemHeight = Math.max(lift, 0.08);
    const materialKey = dynamic ? `dyn_pin_${color}` : `static_pin_${color}`;

    const stem = BABYLON.MeshBuilder.CreateCylinder(
      `${name}_stem`,
      { diameter: stemDiameter, height: stemHeight, tessellation: 14 },
      this.scene,
    );
    stem.position = new BABYLON.Vector3(pose.x, pose.y, anchorZ + stemHeight / 2);
    stem.material = this.getMat(`${materialKey}_stem`, color, Math.min(opacity, 0.82), 0.58, 0.02);
    stem.isPickable = !dynamic;

    const head = BABYLON.MeshBuilder.CreateSphere(
      name,
      { diameter: headDiameter, segments: 18 },
      this.scene,
    );
    head.position = new BABYLON.Vector3(pose.x, pose.y, anchorZ + lift);
    head.material = this.getMat(`${materialKey}_head`, color, opacity, 0.34, 0.08);
    head.isPickable = !dynamic;

    if (dynamic) {
      this.addDynamicMesh(stem);
      this.addDynamicMesh(head);
    } else {
      this.applyMeshShadows(stem);
      this.applyMeshShadows(head);
    }
  }

  private setHoveredNode(nodeId: string | null) {
    if (this.hoveredNodeId === nodeId) {
      return;
    }
    this.hoveredNodeId = nodeId;
    this.hoverCallback?.(nodeId);
  }

  private static isPickableStaticMesh(mesh: { name?: string } | null | undefined) {
    if (!mesh?.name) {
      return false;
    }
    return (
      mesh.name.startsWith('static_feat_') ||
      mesh.name.startsWith('static_node_') ||
      mesh.name.startsWith('static_edge_')
    );
  }

  private pickNearestNodeFromPointer() {
    if (!this.scene || !this.activeManifest || typeof this.scene.pick !== 'function') {
      return null;
    }

    const pickInfo = this.scene.pick(
      this.scene.pointerX,
      this.scene.pointerY,
      (mesh: { name?: string }) => BabylonSceneManager.isPickableStaticMesh(mesh),
    );
    if (!pickInfo?.hit || !pickInfo.pickedPoint) {
      return null;
    }

    return findNearestGraphNode(this.activeManifest.graphNodes, {
      x: pickInfo.pickedPoint.x,
      y: pickInfo.pickedPoint.y,
      z: pickInfo.pickedPoint.z,
    });
  }

  private installPointerObserver() {
    if (!this.scene || this.pointerObserver || !this.scene.onPointerObservable?.add) {
      return;
    }

    this.pointerObserver = this.scene.onPointerObservable.add((pointerInfo: { type: number }) => {
      if (this.pickMode === 'idle' || !this.activeManifest) {
        this.setHoveredNode(null);
        return;
      }

      if (pointerInfo.type === BABYLON.PointerEventTypes.POINTERMOVE) {
        this.setHoveredNode(this.pickNearestNodeFromPointer()?.id ?? null);
        return;
      }

      if (pointerInfo.type === BABYLON.PointerEventTypes.POINTERPICK) {
        const nearestNode = this.pickNearestNodeFromPointer();
        if (nearestNode) {
          this.setHoveredNode(nearestNode.id);
          this.pickCallback?.(nearestNode.id);
        }
      }
    });
  }

  public configurePicking(
    manifest: SceneManifest | null,
    pickMode: PickMode,
    onHoverNodeChange?: (nodeId: string | null) => void,
    onPickNode?: (nodeId: string) => void,
  ) {
    this.activeManifest = manifest;
    this.pickMode = pickMode;
    this.hoverCallback = onHoverNodeChange ?? null;
    this.pickCallback = onPickNode ?? null;
    this.installPointerObserver();

    if (pickMode === 'idle') {
      this.setHoveredNode(null);
    }
  }

  private getMat(id: string, colorHex: string, opacity: number = 1.0, roughness = 0.6, metalness = 0.1): BABYLON.PBRMaterial {
    const key = `${id}-${colorHex}-${opacity}`;
    if (this.materials.has(key)) return this.materials.get(key) as BABYLON.PBRMaterial;
    
    const pbr = new BABYLON.PBRMaterial(key, this.scene!);
    const albedoColor = hexToColor3(colorHex);
    const isSceneFeature = id.startsWith('feat-');
    pbr.albedoColor = albedoColor;
    pbr.emissiveColor = isSceneFeature ? scaleColor3(albedoColor, 0.18) : scaleColor3(albedoColor, 0.04);
    pbr.alpha = opacity;
    pbr.metallic = metalness;
    pbr.roughness = roughness;
    pbr.backFaceCulling = !isSceneFeature;
    if (opacity < 1.0) {
      pbr.transparencyMode = BABYLON.PBRMaterial.MATERIAL_ALPHABLEND;
    }
    pbr.unlit = isSceneFeature;
    this.materials.set(key, pbr);
    return pbr;
  }

  public loadStaticScene(manifest: SceneManifest, layers: LayerState) {
    if (!this.scene || !this.isReady) return;
    this.activeManifest = manifest;

    this.scene.clearColor = colorTupleToColor4(manifest.lights.background, '#d6dde3');
    
    this.scene.meshes.forEach(m => {
      if (m.name.startsWith("static_")) m.dispose();
    });
    this.scene.lights.forEach(l => {
      if (l.name.startsWith("static_light_")) l.dispose();
    });

    manifest.lights.lights.forEach((lData, i) => {
      const dir = new BABYLON.Vector3(lData.pose.x, lData.pose.y, lData.pose.z);
      const dirLight = new BABYLON.DirectionalLight(`static_light_${i}`, new BABYLON.Vector3(-dir.x, -dir.y, -dir.z), this.scene!);
      dirLight.position = dir;
      dirLight.intensity = 1.6;
      dirLight.diffuse = new BABYLON.Color3(lData.diffuse[0]/255, lData.diffuse[1]/255, lData.diffuse[2]/255);
      
      if (layers.shadows && lData.cast_shadows) {
        this.shadowGen = new BABYLON.ShadowGenerator(2048, dirLight);
        this.shadowGen.useBlurExponentialShadowMap = true;
      }
    });

    if (layers.scene) {
      manifest.sceneFeatures.forEach(feature => {
        const mat = this.getMat(`feat-${feature.id}`, feature.fill, feature.opacity, 0.64, feature.render_class === 'world-platform' ? 0.12 : 0.04);
        
        const positions: number[] = [];
        const indices: number[] = [];
        const points = feature.points;
        for (let i = 1; i < points.length - 1; i++) {
          const left = points[0];
          const mid = points[i];
          const right = points[i+1];
          const base = (i - 1) * 3;
          positions.push(left.x, left.y, left.world_z ?? left.z);
          positions.push(mid.x, mid.y, mid.world_z ?? mid.z);
          positions.push(right.x, right.y, right.world_z ?? right.z);
          indices.push(base, base + 1, base + 2);
        }
        
        const vertexData = new BABYLON.VertexData();
        vertexData.positions = positions;
        vertexData.indices = indices;
        const normals: number[] = [];
        BABYLON.VertexData.ComputeNormals(positions, indices, normals);
        vertexData.normals = normals;
        
        const mesh = new BABYLON.Mesh(`static_feat_${feature.id}`, this.scene!);
        vertexData.applyToMesh(mesh);
        mesh.material = mat;
        mesh.isPickable = true;
        this.applyMeshShadows(mesh);
      });
    }

    if (layers.graph) {
      manifest.graphNodes.forEach(node => {
        this.createPinMarker(
          `static_node_${node.id}`,
          node.pose,
          node.type.includes('staging') ? '#8ecae6' : '#f4a261',
          {
            headDiameter: 0.16,
            stemDiameter: 0.042,
            lift: 0.09,
          },
        );
      });

      manifest.graphEdges.forEach(edge => {
        const pts = edge.points.map(p => vec3(p));
        if (pts.length < 2) {
          return;
        }
        const tube = BABYLON.MeshBuilder.CreateTube(`static_edge_${edge.id}`, { path: pts, radius: 0.024, cap: BABYLON.Mesh.CAP_ALL }, this.scene!);
        tube.material = this.getMat(
          `edge-${edge.id}`,
          edge.motion_type.includes('ramp') ? '#9c6644' : '#5f7484',
          0.68,
          0.44,
          0.04,
        );
        this.applyMeshShadows(tube);
      });
    }
  }

  private clearDynamic() {
    this.dynamicMeshes.forEach(m => m.dispose());
    this.dynamicMeshes = [];
  }

  private addDynamicMesh(mesh: BABYLON.Mesh | BABYLON.LinesMesh | BABYLON.TransformNode) {
    if (!mesh.parent) {
      mesh.parent = this.dynamicParent;
    }
    this.dynamicMeshes.push(mesh as BABYLON.Node);
    if (this.shadowGen && mesh instanceof BABYLON.Mesh) {
      this.shadowGen.addShadowCaster(mesh);
      mesh.receiveShadows = true;
    }
  }

  public updateDynamic(
    manifest: SceneManifest | null, 
    frame: PlannerFrame | null, 
    liveEvent: LiveEvent | null, 
    layers: LayerState,
    startPose: Pose3 | null,
    goalPose: Pose3 | null,
    hoverPose: Pose3 | null,
    pickMode: PickMode,
  ) {
    if (!this.scene || !this.isReady) return;
    this.clearDynamic();
    if (!manifest) return;

    if (layers.blocked && liveEvent?.blockOverlay) {
      const blockedIds = new Set(liveEvent.blockOverlay.filter(c => c.state === 1).map(c => c.gridId));
      const blockedSlots = manifest.meilinSlots.filter(s => blockedIds.has(s.block_id));
      blockedSlots.forEach(slot => {
        const mesh = BABYLON.MeshBuilder.CreateBox(`dyn_blocked_${slot.block_id}`, { width: 0.72, depth: 0.72, height: 0.42 }, this.scene!);
        mesh.position = new BABYLON.Vector3(slot.x, slot.y, slot.z + 0.22);
        mesh.material = this.getMat('blocked', '#d62828', 0.58);
        this.addDynamicMesh(mesh);
      });
    }

    if (startPose) {
      this.createPinMarker('dyn_start', startPose, '#2a9d8f', {
        headDiameter: 0.28,
        stemDiameter: 0.06,
        lift: 0.18,
        dynamic: true,
      });
    }

    const offlinePath = frame?.bestPath.points ?? [];
    const pathGoal = offlinePath.length > 0 ? offlinePath[offlinePath.length - 1] : goalPose;
    
    if (pathGoal) {
      this.createPinMarker('dyn_goal', pathGoal, '#d62828', {
        headDiameter: 0.28,
        stemDiameter: 0.06,
        lift: 0.18,
        dynamic: true,
      });
    }

    if (layers.keyNodes) {
      offlinePath.forEach((p, i) => {
        this.createPinMarker(`dyn_pathnode_${i}`, p, '#fcbf49', {
          headDiameter: 0.13,
          stemDiameter: 0.036,
          lift: 0.07,
          dynamic: true,
        });
      });
    }

    const createTube = (pts: Pose3[], color: string, radius: number, opacity: number) => {
      if (pts.length < 2) return;
      const vPts = pts.map(p => vec3(p));
      const tube = BABYLON.MeshBuilder.CreateTube("dyn_tube", { path: vPts, radius, cap: BABYLON.Mesh.CAP_ALL }, this.scene!);
      tube.material = this.getMat(`tube_${color}`, color, opacity, 0.28, 0.18);
      this.addDynamicMesh(tube);
    };

    createTube(offlinePath, '#355070', 0.055, 0.92);
    createTube(liveEvent?.routePath ?? [], '#0ea5e9', 0.05, 0.82);
    createTube(liveEvent?.corridorPath ?? [], '#fb8500', 0.04, 0.82);

    if (layers.tree && frame?.treeSegments) {
      frame.treeSegments.forEach((seg, i) => {
        const lines = BABYLON.MeshBuilder.CreateLines(`dyn_tree_${i}`, { points: [vec3(seg.from), vec3(seg.to)] }, this.scene!);
        lines.color = hexToColor3('#4ea8de');
        lines.alpha = 0.42;
        this.addDynamicMesh(lines);
      });
    }

    if (layers.candidates && frame?.candidateTrajectories) {
      frame.candidateTrajectories.forEach((traj, i) => {
        const lines = BABYLON.MeshBuilder.CreateLines(`dyn_cand_${i}`, { points: traj.points.map(p => vec3(p)) }, this.scene!);
        lines.color = hexToColor3(traj.selected ? '#ff7b00' : traj.collision ? '#adb5bd' : '#94d2bd');
        lines.alpha = traj.selected ? 0.95 : 0.34;
        this.addDynamicMesh(lines);
      });
    }

    if (layers.openSet && frame?.openSet) {
      frame.openSet.forEach(entry => {
        this.createPinMarker(`dyn_open_${entry.nodeId}`, entry.pose, '#219ebc', {
          headDiameter: 0.144,
          stemDiameter: 0.032,
          lift: 0.06,
          opacity: 0.92,
          dynamic: true,
        });
      });
    }

    if (layers.expanded && frame?.expandedNodes) {
      frame.expandedNodes.forEach(entry => {
        this.createPinMarker(`dyn_exp_${entry.nodeId}`, entry.pose, '#ffb703', {
          headDiameter: 0.11,
          stemDiameter: 0.03,
          lift: 0.05,
          opacity: 0.72,
          dynamic: true,
        });
      });
    }

    if (pickMode !== 'idle' && hoverPose) {
      this.createPinMarker('dyn_hover', hoverPose, '#2f80ed', {
        headDiameter: 0.18,
        stemDiameter: 0.04,
        lift: 0.14,
        opacity: 0.9,
        dynamic: true,
      });
    }

    if (frame?.robotPose) {
      const p = frame.robotPose;
      const grp = new BABYLON.TransformNode("dyn_robot", this.scene!);
      grp.position = vec3(p);
      grp.rotation = new BABYLON.Vector3(0, 0, p.yaw); 
      
      const cone = BABYLON.MeshBuilder.CreateCylinder("robot_cone", { diameterTop: 0, diameterBottom: 0.28, height: 0.38, tessellation: 18 }, this.scene!);
      cone.rotation.x = Math.PI / 2;
      cone.position.x = 0.19;
      cone.material = this.getMat('robot_color', '#111827', 1.0, 0.3, 0.2);
      cone.parent = grp;
      
      const cyl = BABYLON.MeshBuilder.CreateCylinder("robot_cyl", { diameter: 0.22, height: 0.14, tessellation: 18 }, this.scene!);
      cyl.rotation.x = Math.PI / 2;
      cyl.material = this.getMat('robot_base', '#f5f0e8', 1.0, 0.45);
      cyl.parent = grp;

      this.addDynamicMesh(cone);
      this.addDynamicMesh(cyl);
      this.addDynamicMesh(grp);
    }
  }

  public updateCamera(viewMode: ViewMode, robotPose: Pose3 | null, manifest: SceneManifest) {
    if (!this.scene || !this.isReady) return;
    const presets = manifest.cameraPresets;

    if (viewMode === 'follow' && robotPose) {
      this.scene.activeCamera = this.followCam;
      const tX = robotPose.x;
      const tY = robotPose.y;
      const tZ = (robotPose.world_z ?? robotPose.z) + 0.12;
      const lX = tX - Math.cos(robotPose.yaw) * 1.35;
      const lY = tY - Math.sin(robotPose.yaw) * 1.35;
      const lZ = tZ + 0.68;
      this.followCam.position = new BABYLON.Vector3(lX, lY, lZ);
      this.followCam.setTarget(new BABYLON.Vector3(tX, tY, tZ));
      return;
    }

    if (viewMode === 'first_person' && robotPose) {
      this.scene.activeCamera = this.fpCam;
      const eX = robotPose.x;
      const eY = robotPose.y;
      const eZ = (robotPose.world_z ?? robotPose.z) + 0.42;
      const tX = eX + Math.cos(robotPose.yaw) * 1.8;
      const tY = eY + Math.sin(robotPose.yaw) * 1.8;
      const tZ = eZ + 0.02;
      this.fpCam.position = new BABYLON.Vector3(eX, eY, eZ);
      this.fpCam.setTarget(new BABYLON.Vector3(tX, tY, tZ));
      return;
    }

    const presetMap = Object.fromEntries(presets.map(p => [p.id, p]));
    if (viewMode === 'side_perspective') {
      const sidePreset = presetMap.side_perspective ?? presetMap.side_ortho;
      const spanX = Math.max(manifest.bounds.max_x - manifest.bounds.min_x, 1);
      const spanY = Math.max(manifest.bounds.max_y - manifest.bounds.min_y, 1);
      const spanZ = Math.max(manifest.bounds.max_z - manifest.bounds.min_z, 1);
      const centerX = (manifest.bounds.min_x + manifest.bounds.max_x) / 2;
      const centerY = (manifest.bounds.min_y + manifest.bounds.max_y) / 2;
      const centerZ = (manifest.bounds.min_z + manifest.bounds.max_z) / 2;
      const target = sidePreset
        ? vec3(sidePreset.target)
        : new BABYLON.Vector3(centerX, centerY, centerZ + spanZ * 0.2);

      let position: BABYLON.Vector3;
      if (sidePreset) {
        const source = vec3(sidePreset.position);
        const dirX = source.x - target.x;
        const dirY = source.y - target.y;
        const dirZ = source.z - target.z;
        const dirLen = Math.sqrt((dirX * dirX) + (dirY * dirY) + (dirZ * dirZ)) || 1;
        const distance = Math.max(spanX, spanY) * 1.35 + spanZ * 1.4 + 8;
        position = new BABYLON.Vector3(
          target.x + (dirX / dirLen) * distance,
          target.y + (dirY / dirLen) * distance,
          Math.max(target.z + spanZ * 1.1 + 3.4, target.z + (dirZ / dirLen) * distance),
        );
      } else {
        position = new BABYLON.Vector3(
          manifest.bounds.max_x + spanX * 0.8,
          centerY - spanY * 0.42,
          manifest.bounds.max_z + spanZ * 1.3 + 4.8,
        );
      }

      this.scene.activeCamera = this.orbitCam;
      this.orbitCam.setPosition(position);
      this.orbitCam.setTarget(target);
      return;
    }

    const preset = presetMap[viewMode];
    if (!preset) {
      this.scene.activeCamera = this.orbitCam;
      return;
    }

    if (preset.kind === 'orthographic') {
      const cam = viewMode === 'top_ortho' ? this.topOrthoCam : this.sideOrthoCam;
      this.scene.activeCamera = cam;
      cam.position = vec3(preset.position);
      cam.setTarget(vec3(preset.target));
      this.setupOrtho(cam, 74);
    } else {
      this.scene.activeCamera = this.orbitCam;
      this.orbitCam.setPosition(vec3(preset.position));
      this.orbitCam.setTarget(vec3(preset.target));
    }
  }
}

export function SceneCanvas(props: SceneCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const managerRef = useRef<BabylonSceneManager | null>(null);
  const [engineReady, setEngineReady] = useState(false);
  const hasScene = Boolean(props.scene);

  useEffect(() => {
    if (!hasScene || !canvasRef.current) return;

    let disposed = false;
    setEngineReady(false);
    const manager = new BabylonSceneManager();
    managerRef.current = manager;

    manager
      .init(canvasRef.current, () => {
        // React StrictMode 会触发一次预清理，旧实例不能回写当前 manager 的 ready 状态。
        if (disposed || managerRef.current !== manager) {
          manager.dispose();
          return;
        }
        setEngineReady(true);
      })
      .catch(err => {
        if (!disposed && managerRef.current === manager) {
          console.error("Failed to initialize Babylon.js engine:", err);
        }
      });

    return () => {
      disposed = true;
      if (managerRef.current === manager) {
        managerRef.current = null;
      }
      manager.dispose();
    };
  }, [hasScene]);

  useEffect(() => {
    if (engineReady && managerRef.current && props.scene) {
      managerRef.current.loadStaticScene(props.scene, props.layers);
    }
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [engineReady, props.scene, props.layers.scene, props.layers.graph, props.layers.shadows]);

  useEffect(() => {
    if (engineReady && managerRef.current) {
      managerRef.current.updateDynamic(
        props.scene,
        props.frame,
        props.liveEvent,
        props.layers,
        props.startPose,
        props.goalPose,
        props.hoverPose,
        props.pickMode,
      );
    }
  }, [engineReady, props.scene, props.frame, props.liveEvent, props.layers, props.startPose, props.goalPose, props.hoverPose, props.pickMode]);

  useEffect(() => {
    if (engineReady && managerRef.current && props.scene) {
      managerRef.current.updateCamera(props.viewMode, props.frame?.robotPose ?? null, props.scene);
    }
  }, [engineReady, props.viewMode, props.frame?.robotPose, props.scene]);

  useEffect(() => {
    if (engineReady && managerRef.current) {
      managerRef.current.configurePicking(
        props.scene,
        props.pickMode,
        props.onHoverNodeChange,
        props.onPickNode,
      );
    }
  }, [engineReady, props.scene, props.pickMode, props.onHoverNodeChange, props.onPickNode]);

  if (!props.scene) {
    return <div className="canvas-placeholder">场景未加载，无法渲染 3D 视图。</div>;
  }

  return (
    <canvas 
      ref={canvasRef} 
      style={{ width: '100%', height: '100%', display: 'block', outline: 'none', touchAction: 'none' }}
    />
  );
}
