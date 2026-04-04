import { Edge, MarkerType } from '@xyflow/react';

export type CanvasVisualType =
  | 'control'
  | 'decorator'
  | 'action'
  | 'condition'
  | 'subtree'
  | 'sequence'
  | 'selector';

export const CANVAS_BACKGROUND = {
  color: '#cbd5e1',
  gap: 24,
  size: 2,
};

export const CANVAS_LAYOUT = {
  rankdir: 'LR' as const,
  nodesep: 80,
  ranksep: 160,
  edgesep: 20,
  marginx: 20,
  marginy: 20,
};

export function normalizeCanvasVisualType(type: CanvasVisualType): Exclude<CanvasVisualType, 'sequence' | 'selector'> {
  if (type === 'sequence' || type === 'selector') {
    return 'control';
  }

  return type;
}

export function getCanvasNodeSize(type: CanvasVisualType): { width: number; height: number } {
  const normalizedType = normalizeCanvasVisualType(type);

  switch (normalizedType) {
    case 'control':
      return { width: 140, height: 48 };
    case 'decorator':
      return { width: 180, height: 48 };
    case 'condition':
    case 'action':
    case 'subtree':
    default:
      return { width: 240, height: 80 };
  }
}

export function createCanvasEdgeProps(isActive = false): Partial<Edge> {
  const color = isActive ? '#f59e0b' : '#94a3b8';
  return {
    type: 'smoothstep',
    animated: isActive,
    style: {
      stroke: color,
      strokeWidth: isActive ? 3 : 2,
    },
    markerEnd: {
      type: MarkerType.ArrowClosed,
      width: 15,
      height: 15,
      color,
    },
  };
}
