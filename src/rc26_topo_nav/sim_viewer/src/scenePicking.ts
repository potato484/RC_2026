import type { GraphNode } from './types';

export interface PickedWorldPoint {
  x: number;
  y: number;
  z: number;
}

export function squaredDistanceXY(
  point: PickedWorldPoint,
  node: GraphNode,
): number {
  const dx = node.pose.x - point.x;
  const dy = node.pose.y - point.y;
  return dx * dx + dy * dy;
}

export function findNearestGraphNode(
  nodes: GraphNode[],
  point: PickedWorldPoint,
): GraphNode | null {
  if (nodes.length === 0) {
    return null;
  }

  let nearestNode = nodes[0];
  let nearestDistance = squaredDistanceXY(point, nodes[0]);

  for (let index = 1; index < nodes.length; index += 1) {
    const candidate = nodes[index];
    const candidateDistance = squaredDistanceXY(point, candidate);
    if (candidateDistance < nearestDistance) {
      nearestNode = candidate;
      nearestDistance = candidateDistance;
    }
  }

  return nearestNode;
}

export function worldPointToPose(point: PickedWorldPoint) {
  return {
    x: point.x,
    y: point.y,
    z: point.z,
    yaw: 0,
    world_z: point.z,
  };
}
