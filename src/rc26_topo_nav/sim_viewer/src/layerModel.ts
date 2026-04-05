import { LAYER_LABELS, LAYER_SHORT_LABELS } from './labels';
import type { LayerState } from './store';
import type { PlannerFrame } from './types';

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

export function deriveLayerControls(params: {
  layers: LayerState;
  frame: PlannerFrame | null;
}): LayerControl[] {
  const { layers, frame } = params;
  const hasFrontier = (frame?.openSet.length ?? 0) > 0;
  const hasExpanded = (frame?.expandedNodes.length ?? 0) > 0;

  return [
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
      key: 'openSet',
      label: LAYER_LABELS.openSet,
      shortLabel: LAYER_SHORT_LABELS.openSet,
      active: layers.openSet,
      enabled: hasFrontier,
      visible: true,
      group: 'primary',
      tone: 'search',
      reason: hasFrontier ? undefined : '生成路线后才会出现前沿点',
    },
    {
      key: 'expanded',
      label: LAYER_LABELS.expanded,
      shortLabel: LAYER_SHORT_LABELS.expanded,
      active: layers.expanded,
      enabled: hasExpanded,
      visible: true,
      group: 'primary',
      tone: 'search',
      reason: hasExpanded ? undefined : '生成路线后才会出现已探查点',
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
}
