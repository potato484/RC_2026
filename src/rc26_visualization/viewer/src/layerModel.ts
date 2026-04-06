import { LAYER_LABELS, LAYER_SHORT_LABELS } from './labels';
import type { LayerState } from './store';
import type { DisplayCatalogEntry, DisplayTone, LiveEvent, PlannerFrame, SceneManifest } from './types';

export type LayerKey = keyof LayerState;

export interface LayerControl {
  key: LayerKey;
  label: string;
  shortLabel: string;
  active: boolean;
  enabled: boolean;
  visible: boolean;
  group: 'primary' | 'advanced';
  tone: DisplayTone;
  reason?: string;
}

const FALLBACK_CATALOG: DisplayCatalogEntry[] = [
  { id: 'scene', label: LAYER_LABELS.scene, short_label: LAYER_SHORT_LABELS.scene, group: 'primary', tone: 'scene' },
  { id: 'route', label: LAYER_LABELS.route, short_label: LAYER_SHORT_LABELS.route, group: 'primary', tone: 'path' },
  { id: 'corridor', label: LAYER_LABELS.corridor, short_label: LAYER_SHORT_LABELS.corridor, group: 'primary', tone: 'path' },
  { id: 'lookahead', label: LAYER_LABELS.lookahead, short_label: LAYER_SHORT_LABELS.lookahead, group: 'primary', tone: 'path' },
  { id: 'robotPose', label: LAYER_LABELS.robotPose, short_label: LAYER_SHORT_LABELS.robotPose, group: 'primary', tone: 'state' },
  { id: 'phaseZones', label: LAYER_LABELS.phaseZones, short_label: LAYER_SHORT_LABELS.phaseZones, group: 'primary', tone: 'risk' },
  { id: 'blocked', label: LAYER_LABELS.blocked, short_label: LAYER_SHORT_LABELS.blocked, group: 'primary', tone: 'risk' },
  { id: 'graph', label: LAYER_LABELS.graph, short_label: LAYER_SHORT_LABELS.graph, group: 'advanced', tone: 'search' },
  { id: 'keyNodes', label: LAYER_LABELS.keyNodes, short_label: LAYER_SHORT_LABELS.keyNodes, group: 'advanced', tone: 'path' },
  { id: 'openSet', label: LAYER_LABELS.openSet, short_label: LAYER_SHORT_LABELS.openSet, group: 'advanced', tone: 'search' },
  { id: 'expanded', label: LAYER_LABELS.expanded, short_label: LAYER_SHORT_LABELS.expanded, group: 'advanced', tone: 'search' },
  { id: 'tree', label: LAYER_LABELS.tree, short_label: LAYER_SHORT_LABELS.tree, group: 'advanced', tone: 'search' },
  { id: 'candidates', label: LAYER_LABELS.candidates, short_label: LAYER_SHORT_LABELS.candidates, group: 'advanced', tone: 'search' },
  { id: 'shadows', label: LAYER_LABELS.shadows, short_label: LAYER_SHORT_LABELS.shadows, group: 'advanced', tone: 'appearance' },
];

function fallbackReason(key: LayerKey): string | undefined {
  if (key === 'openSet') {
    return '生成搜索回放后才会出现前沿点';
  }
  if (key === 'expanded') {
    return '生成搜索回放后才会出现已探查点';
  }
  if (key === 'tree') {
    return '当前回放没有搜索树分支';
  }
  if (key === 'candidates') {
    return '当前回放没有局部规划候选轨迹';
  }
  if (key === 'phaseZones') {
    return '场景 manifest 没有提供阶段区定义';
  }
  return undefined;
}

function resolveControlState(
  key: LayerKey,
  frame: PlannerFrame | null,
  scene: SceneManifest | null,
  liveEvent: LiveEvent | null,
  traceOpenSetCount: number,
  traceExpandedCount: number,
): { enabled: boolean; reason?: string } {
  if (key === 'openSet') {
    const enabled = traceOpenSetCount > 0;
    return { enabled, reason: enabled ? undefined : fallbackReason(key) };
  }
  if (key === 'expanded') {
    const enabled = traceExpandedCount > 0;
    return { enabled, reason: enabled ? undefined : fallbackReason(key) };
  }
  if (key === 'tree') {
    const enabled = (frame?.treeSegments.length ?? 0) > 0;
    return { enabled, reason: enabled ? undefined : fallbackReason(key) };
  }
  if (key === 'candidates') {
    const enabled = (frame?.candidateTrajectories.length ?? 0) > 0;
    return { enabled, reason: enabled ? undefined : fallbackReason(key) };
  }
  if (key === 'phaseZones') {
    const enabled = (scene?.semanticZones?.length ?? 0) > 0;
    return { enabled, reason: enabled ? undefined : fallbackReason(key) };
  }
  if (key === 'blocked') {
    const enabled =
      (scene?.meilinSlots.length ?? 0) > 0 ||
      (liveEvent?.blockOverlay?.length ?? 0) > 0;
    return { enabled, reason: enabled ? undefined : '当前没有 keepout 或 block overlay 数据' };
  }
  return { enabled: true };
}

export function deriveLayerControls(params: {
  layers: LayerState;
  frame: PlannerFrame | null;
  scene: SceneManifest | null;
  liveEvent: LiveEvent | null;
  traceOpenSetCount?: number;
  traceExpandedCount?: number;
}): LayerControl[] {
  const {
    layers,
    frame,
    scene,
    liveEvent,
    traceOpenSetCount = frame?.openSet.length ?? 0,
    traceExpandedCount = frame?.expandedNodes.length ?? 0,
  } = params;
  const catalog = scene?.displayCatalog?.length ? scene.displayCatalog : FALLBACK_CATALOG;

  return catalog
    .filter((entry): entry is DisplayCatalogEntry & { id: LayerKey } => entry.id in layers)
    .map((entry) => {
      const status = resolveControlState(entry.id, frame, scene, liveEvent, traceOpenSetCount, traceExpandedCount);
      return {
        key: entry.id,
        label: entry.label,
        shortLabel: entry.short_label,
        active: layers[entry.id],
        enabled: status.enabled,
        visible: true,
        group: entry.group,
        tone: entry.tone,
        reason: status.reason,
      };
    });
}
