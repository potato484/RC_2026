import type {
  ExpandedNode,
  OpenSetEntry,
  PlannerFrame,
  PlannerTraceFrame,
  Pose3,
  SceneManifest,
} from '../../types';

export function buildTraceNodePoseMap(
  scene: SceneManifest | null,
  traceNodePoses: Record<string, Pose3>,
): Map<string, Pose3> {
  const nodePoseMap = new Map<string, Pose3>();
  if (!scene) {
    return nodePoseMap;
  }
  scene.graphNodes.forEach((node) => {
    nodePoseMap.set(node.id, node.pose);
  });
  Object.entries(traceNodePoses).forEach(([nodeId, pose]) => {
    nodePoseMap.set(nodeId, pose);
  });
  return nodePoseMap;
}

export function hydrateOpenSetEntries(
  entries: PlannerTraceFrame['openSet'],
  nodePoseMap: Map<string, Pose3>,
): OpenSetEntry[] {
  return entries.flatMap((entry) => {
    const pose = entry.pose ?? nodePoseMap.get(entry.nodeId);
    if (!pose) {
      return [];
    }
    return [{ ...entry, pose }];
  });
}

export function hydrateExpandedEntries(
  entries: PlannerTraceFrame['expandedNodes'],
  nodePoseMap: Map<string, Pose3>,
): ExpandedNode[] {
  return entries.flatMap((entry) => {
    const pose = entry.pose ?? nodePoseMap.get(entry.nodeId);
    if (!pose) {
      return [];
    }
    return [{ ...entry, pose }];
  });
}

export function resolveBestPathPoints(
  bestPath: PlannerTraceFrame['bestPath'],
  nodePoseMap: Map<string, Pose3>,
): Pose3[] {
  if (bestPath.points && bestPath.points.length > 0) {
    return bestPath.points;
  }
  return bestPath.nodeIds.flatMap((nodeId) => {
    const pose = nodePoseMap.get(nodeId);
    return pose ? [pose] : [];
  });
}

export function hydratePlannerFrame(
  frame: PlannerTraceFrame | null,
  nodePoseMap: Map<string, Pose3>,
): PlannerFrame | null {
  if (!frame) {
    return null;
  }

  const openSet = hydrateOpenSetEntries(frame.openSet, nodePoseMap);
  const expandedNodes = hydrateExpandedEntries(frame.expandedNodes, nodePoseMap);
  const bestPathPoints = resolveBestPathPoints(frame.bestPath, nodePoseMap);

  return {
    ...frame,
    openSet,
    expandedNodes,
    bestPath: {
      ...frame.bestPath,
      points: bestPathPoints,
    },
  };
}

export function accumulateOpenSetEntries(
  frames: PlannerTraceFrame[],
  nodePoseMap: Map<string, Pose3>,
): OpenSetEntry[] {
  const seenNodeIds = new Set<string>();
  const accumulated: OpenSetEntry[] = [];
  frames.forEach((frame) => {
    hydrateOpenSetEntries(frame.openSet, nodePoseMap).forEach((entry) => {
      if (seenNodeIds.has(entry.nodeId)) {
        return;
      }
      seenNodeIds.add(entry.nodeId);
      accumulated.push(entry);
    });
  });
  return accumulated;
}

export function accumulateExpandedEntries(
  frames: PlannerTraceFrame[],
  nodePoseMap: Map<string, Pose3>,
): ExpandedNode[] {
  const seenNodeIds = new Set<string>();
  const accumulated: ExpandedNode[] = [];
  frames.forEach((frame) => {
    hydrateExpandedEntries(frame.expandedNodes, nodePoseMap).forEach((entry) => {
      if (seenNodeIds.has(entry.nodeId)) {
        return;
      }
      seenNodeIds.add(entry.nodeId);
      accumulated.push(entry);
    });
  });
  return accumulated;
}
