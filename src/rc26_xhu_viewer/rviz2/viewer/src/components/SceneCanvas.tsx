import { useEffect, useRef, useState } from 'react';

import { BabylonSceneManager } from './scene/BabylonSceneManager';
import type { LayerState } from '../store';
import type {
  ExpandedNode,
  LiveEvent,
  OpenSetEntry,
  PickMode,
  PlannerFrame,
  Pose3,
  SceneManifest,
  ViewMode,
} from '../types';

export interface SceneCanvasProps {
  scene: SceneManifest | null;
  frame: PlannerFrame | null;
  cumulativeOpenSet?: OpenSetEntry[];
  cumulativeExpandedNodes?: ExpandedNode[];
  liveEvent: LiveEvent | null;
  viewMode: ViewMode;
  layers: LayerState;
  startPose: Pose3 | null;
  goalPose: Pose3 | null;
  hoverPose: Pose3 | null;
  manualPath?: Pose3[];
  manualPathRejected?: boolean;
  blockedGridIds: number[];
  pickMode: PickMode;
  onHoverNodeChange?: (nodeId: string | null) => void;
  onPickNode?: (nodeId: string) => void;
  onHoverWorldChange?: (pose: Pose3 | null) => void;
  onPickWorld?: (pose: Pose3) => void;
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
      .catch((err) => {
        if (!disposed && managerRef.current === manager) {
          console.error('Failed to initialize Babylon.js engine:', err);
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
  }, [engineReady, props.scene, props.layers.scene, props.layers.graph, props.layers.shadows]);

  useEffect(() => {
    if (engineReady && managerRef.current) {
      managerRef.current.updateDynamic(
        props.scene,
        props.frame,
        props.cumulativeOpenSet,
        props.cumulativeExpandedNodes,
        props.liveEvent,
        props.layers,
        props.startPose,
        props.goalPose,
        props.hoverPose,
        props.manualPath,
        props.manualPathRejected ?? false,
        props.blockedGridIds,
        props.pickMode,
      );
    }
  }, [
    engineReady,
    props.scene,
    props.frame,
    props.cumulativeOpenSet,
    props.cumulativeExpandedNodes,
    props.liveEvent,
    props.layers,
    props.startPose,
    props.goalPose,
    props.hoverPose,
    props.manualPath,
    props.manualPathRejected,
    props.blockedGridIds,
    props.pickMode,
  ]);

  useEffect(() => {
    if (engineReady && managerRef.current && props.scene) {
      managerRef.current.updateCamera(
        props.viewMode,
        props.liveEvent?.controlState?.pose ?? props.frame?.robotPose ?? null,
        props.scene,
      );
    }
  }, [engineReady, props.viewMode, props.frame?.robotPose, props.liveEvent?.controlState?.pose, props.scene]);

  useEffect(() => {
    if (engineReady && managerRef.current) {
      managerRef.current.configurePicking(
        props.scene,
        props.pickMode,
        props.onHoverNodeChange,
        props.onPickNode,
        props.onHoverWorldChange,
        props.onPickWorld,
      );
    }
  }, [engineReady, props.scene, props.pickMode, props.onHoverNodeChange, props.onPickNode, props.onHoverWorldChange, props.onPickWorld]);

  if (!props.scene) {
    return <div className="canvas-placeholder">场景未加载，无法渲染三维视图。</div>;
  }

  return (
    <canvas
      ref={canvasRef}
      style={{ width: '100%', height: '100%', display: 'block', outline: 'none', touchAction: 'none' }}
    />
  );
}
