import { Edge as FlowEdge, Node as FlowNode } from '@xyflow/react';
import dagre from 'dagre';
import { EditorNode, EditorTree } from '../types/editor';
import {
  canNodeAcceptChildren,
  getCompositeSwitchCandidates,
  getBtNodeDefinition,
} from './btRegistry';
import {
  getBehaviorTreeNodeCategoryLabel,
  getBehaviorTreeNodeDisplay,
  summarizeBehaviorTreeAttributes,
} from './btDisplay';

export interface EditorFlowNodeData extends Record<string, unknown> {
  editorNodeId: string;
  tagName: string;
  displayLabel: string;
  displayDesc: string;
  attributeSummary: string;
  categoryLabel: string;
  sourceLabel: string;
  attributes: Record<string, string>;
  uiType: EditorNode['uiType'];
  isRoot?: boolean;
  canAcceptChildren: boolean;
  hasChildren: boolean;
  switchCandidates: string[];
  selected?: boolean;
}

const getNodeSize = (uiType: EditorNode['uiType']) => {
  switch (uiType) {
    case 'control':
      return { width: 240, height: 92 };
    case 'decorator':
      return { width: 230, height: 86 };
    case 'subtree':
      return { width: 250, height: 92 };
    case 'leaf':
    default:
      return { width: 260, height: 96 };
  }
};

const getSourceLabel = (node: EditorNode): string => {
  if (node.source === 'official') {
    return '官方';
  }
  if (node.source === 'robot') {
    return '机器人模块';
  }
  return '未注册';
};

export function projectTreeToFlow(
  tree: EditorTree,
  collapsedNodeIds: Set<string>
): { nodes: FlowNode[]; edges: FlowEdge[] } {
  const nodes: FlowNode[] = [];
  const edges: FlowEdge[] = [];

  const traverse = (node: EditorNode, parentId?: string, isRoot = false) => {
    const definition = getBtNodeDefinition(node.tagName);
    const display = getBehaviorTreeNodeDisplay(node.tagName, node.attributes);
    const sourceLabel = getSourceLabel(node);
    const categoryLabel = getBehaviorTreeNodeCategoryLabel(definition?.category ?? node.nodeKind);

    const nodeData: EditorFlowNodeData = {
      editorNodeId: node.id,
      tagName: node.tagName,
      displayLabel: display.label,
      displayDesc: display.desc,
      attributeSummary: summarizeBehaviorTreeAttributes(node.attributes, node.tagName).join(' · '),
      categoryLabel,
      sourceLabel,
      attributes: { ...node.attributes },
      uiType: node.uiType,
      isRoot,
      canAcceptChildren: canNodeAcceptChildren(node),
      hasChildren: node.children.length > 0,
      switchCandidates: getCompositeSwitchCandidates(node.tagName).map((entry) => entry.labelZh),
    };

    nodes.push({
      id: node.id,
      type: 'editorNode',
      position: { x: 0, y: 0 },
      data: nodeData,
    });

    if (parentId) {
      edges.push({
        id: `e-${parentId}-${node.id}`,
        source: parentId,
        target: node.id,
        type: 'smoothstep',
        style: { stroke: '#94a3b8', strokeWidth: 2 },
      });
    }

    if (!collapsedNodeIds.has(node.id)) {
      node.children.forEach((child) => traverse(child, node.id));
    }
  };

  traverse(tree.rootNode, undefined, true);

  const graph = new dagre.graphlib.Graph();
  graph.setDefaultEdgeLabel(() => ({}));
  graph.setGraph({
    rankdir: 'LR',
    nodesep: 64,
    ranksep: 120,
    edgesep: 18,
    marginx: 24,
    marginy: 24,
  });

  nodes.forEach((node) => {
    const nodeSize = getNodeSize((node.data as EditorFlowNodeData).uiType);
    graph.setNode(node.id, nodeSize);
  });

  edges.forEach((edge) => {
    graph.setEdge(edge.source, edge.target);
  });

  dagre.layout(graph);

  nodes.forEach((node) => {
    const position = graph.node(node.id);
    const nodeSize = getNodeSize((node.data as EditorFlowNodeData).uiType);
    node.position = {
      x: position.x - nodeSize.width / 2,
      y: position.y - nodeSize.height / 2,
    };
  });

  return { nodes, edges };
}
