import { ALGORITHM_LABELS, LAYER_LABELS, LAYER_SHORT_LABELS } from './labels';
import type { LayerState } from './store';
import type { Algorithm, LiveEvent, PlannerFrame, RunMode, SceneManifest } from './types';

export type LayerKey = keyof LayerState;

export interface LayerControl {
  key: LayerKey;
  label: string;
  shortLabel: string;
  active: boolean;
  enabled: boolean;
  visible: boolean;
  group: 'primary' | 'advanced';
  tone: 'scene' | 'search' | 'path' | 'risk' | 'appearance';
  reason?: string;
}

export function parseBlockedNodes(value: string): string[] {
  return value
    .split(',')
    .map((item) => item.trim())
    .filter(Boolean);
}

export function deriveBlockedGridIds(params: {
  scene: SceneManifest | null;
  mode: RunMode;
  liveEvent: LiveEvent | null;
  blockedNodeIds: string[];
}): number[] {
  const { scene, mode, liveEvent, blockedNodeIds } = params;

  if (mode === 'live-ros') {
    const activeIds = liveEvent?.blockOverlay
      ?.filter((cell) => cell.state === 1)
      .map((cell) => cell.gridId) ?? [];
    return Array.from(new Set(activeIds));
  }

  if (!scene || blockedNodeIds.length === 0) {
    return [];
  }

  const slotIds = new Set(scene.meilinSlots.map((slot) => slot.block_id));
  const blockedNodeIdSet = new Set(blockedNodeIds);
  const blockedGridIds = new Set<number>();
  scene.graphNodes.forEach((node) => {
    if (blockedNodeIdSet.has(node.id) && slotIds.has(node.block_id)) {
      blockedGridIds.add(node.block_id);
    }
  });
  return Array.from(blockedGridIds);
}

function runReason(algorithm: Algorithm): string {
  return `需先生成 ${ALGORITHM_LABELS[algorithm]} 运行`;
}

export function deriveLayerControls(params: {
  layers: LayerState;
  mode: RunMode;
  algorithm: Algorithm;
  frame: PlannerFrame | null;
  blockedGridIds: number[];
}): LayerControl[] {
  const { layers, mode, algorithm, frame, blockedGridIds } = params;
  const effectiveAlgorithm = frame?.algorithm ?? algorithm;
  const offline = mode === 'offline-sim';
  const live = mode === 'live-ros';
  const hasPathNodes = (frame?.bestPath.points.length ?? 0) > 0;
  const hasOpenSet = (frame?.openSet.length ?? 0) > 0;
  const hasExpanded = (frame?.expandedNodes.length ?? 0) > 0;
  const hasTree = (frame?.treeSegments.length ?? 0) > 0;
  const hasCandidates = (frame?.candidateTrajectories.length ?? 0) > 0;
  const hasBlocked = blockedGridIds.length > 0;

  const controls: LayerControl[] = [
    {
      key: 'scene',
      label: LAYER_LABELS.scene,
      shortLabel: LAYER_SHORT_LABELS.scene,
      active: layers.scene,
      enabled: true,
      visible: true,
      group: 'primary',
      tone: 'scene',
    },
    {
      key: 'graph',
      label: LAYER_LABELS.graph,
      shortLabel: LAYER_SHORT_LABELS.graph,
      active: layers.graph,
      enabled: true,
      visible: true,
      group: 'primary',
      tone: 'scene',
    },
    {
      key: 'keyNodes',
      label: LAYER_LABELS.keyNodes,
      shortLabel: LAYER_SHORT_LABELS.keyNodes,
      active: layers.keyNodes,
      enabled: hasPathNodes,
      visible: offline && effectiveAlgorithm === 'astar',
      group: 'primary',
      tone: 'path',
      reason: !hasPathNodes ? runReason('astar') : undefined,
    },
    {
      key: 'openSet',
      label: LAYER_LABELS.openSet,
      shortLabel: LAYER_SHORT_LABELS.openSet,
      active: layers.openSet,
      enabled: hasOpenSet,
      visible: offline && effectiveAlgorithm === 'astar',
      group: 'primary',
      tone: 'search',
      reason: !hasOpenSet ? runReason('astar') : undefined,
    },
    {
      key: 'expanded',
      label: LAYER_LABELS.expanded,
      shortLabel: LAYER_SHORT_LABELS.expanded,
      active: layers.expanded,
      enabled: hasExpanded,
      visible: offline && effectiveAlgorithm === 'astar',
      group: 'primary',
      tone: 'search',
      reason: !hasExpanded ? runReason('astar') : undefined,
    },
    {
      key: 'tree',
      label: LAYER_LABELS.tree,
      shortLabel: LAYER_SHORT_LABELS.tree,
      active: layers.tree,
      enabled: hasTree,
      visible: offline && effectiveAlgorithm === 'rrt',
      group: 'primary',
      tone: 'search',
      reason: !hasTree ? runReason('rrt') : undefined,
    },
    {
      key: 'candidates',
      label: LAYER_LABELS.candidates,
      shortLabel: LAYER_SHORT_LABELS.candidates,
      active: layers.candidates,
      enabled: hasCandidates,
      visible: offline && effectiveAlgorithm === 'dwa',
      group: 'primary',
      tone: 'path',
      reason: !hasCandidates ? runReason('dwa') : undefined,
    },
    {
      key: 'blocked',
      label: LAYER_LABELS.blocked,
      shortLabel: LAYER_SHORT_LABELS.blocked,
      active: layers.blocked,
      enabled: hasBlocked,
      visible: offline || live,
      group: 'primary',
      tone: 'risk',
      reason: !hasBlocked
        ? live
          ? '等待实时阻塞区输入'
          : '高级面板中尚未配置阻塞节点'
        : undefined,
    },
    {
      key: 'shadows',
      label: LAYER_LABELS.shadows,
      shortLabel: LAYER_SHORT_LABELS.shadows,
      active: layers.shadows,
      enabled: true,
      visible: true,
      group: 'advanced',
      tone: 'appearance',
    },
  ];

  return controls;
}
