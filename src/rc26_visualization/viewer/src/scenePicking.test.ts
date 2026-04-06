import { describe, expect, it } from 'vitest';

import { findNearestGraphNode } from './scenePicking';
import type { GraphNode } from './types';

const graphNodes: GraphNode[] = [
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
    pose: { x: 2, y: 0.5, z: 0, yaw: 0 },
  },
];

describe('findNearestGraphNode', () => {
  it('returns the closest node in XY space for an arbitrary scene click', () => {
    const nearestNode = findNearestGraphNode(graphNodes, { x: 1.6, y: 0.4, z: 0 });
    expect(nearestNode?.id).toBe('node-b');
  });

  it('returns null when the graph has no nodes', () => {
    expect(findNearestGraphNode([], { x: 0, y: 0, z: 0 })).toBeNull();
  });
});
