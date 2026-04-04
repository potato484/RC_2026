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
import { CANVAS_LAYOUT, createCanvasEdgeProps, getCanvasNodeSize } from './btCanvasTheme';

export interface EditorFlowNodeData extends Record<string, unknown> {
  editorNodeId: string;
  tagName: string;
  displayLabel: string;
  displayDesc: string;
  baseDescription: string;
  attributeSummary: string;
  categoryLabel: string;
  sourceLabel: string;
  nodeKind: EditorNode['nodeKind'];
  attributes: Record<string, string>;
  uiType: EditorNode['uiType'];
  isRoot?: boolean;
  canAcceptChildren: boolean;
  hasChildren: boolean;
  collapsed: boolean;
  switchCandidates: string[];
  selected?: boolean;
}

export interface EditorFlowEdgeData extends Record<string, unknown> {
  parentNodeId: string;
  childNodeId: string;
}

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
      baseDescription: definition?.descriptionZh ?? display.desc,
      attributeSummary: summarizeBehaviorTreeAttributes(node.attributes, node.tagName).join(' · '),
      categoryLabel,
      sourceLabel,
      nodeKind: node.nodeKind,
      attributes: { ...node.attributes },
      uiType: node.uiType,
      isRoot,
      canAcceptChildren: canNodeAcceptChildren(node),
      hasChildren: node.children.length > 0,
      collapsed: collapsedNodeIds.has(node.id),
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
        ...createCanvasEdgeProps(),
        type: 'editorInsertEdge',
        data: {
          parentNodeId: parentId,
          childNodeId: node.id,
        } satisfies EditorFlowEdgeData,
      });
    }

    if (!collapsedNodeIds.has(node.id)) {
      node.children.forEach((child) => traverse(child, node.id));
    }
  };

  traverse(tree.rootNode, undefined, true);

  const graph = new dagre.graphlib.Graph();
  graph.setDefaultEdgeLabel(() => ({}));
  graph.setGraph(CANVAS_LAYOUT);

  nodes.forEach((node) => {
    const nodeSize = getCanvasNodeSize((node.data as EditorFlowNodeData).nodeKind);
    graph.setNode(node.id, nodeSize);
  });

  edges.forEach((edge) => {
    graph.setEdge(edge.source, edge.target);
  });

  dagre.layout(graph);

  nodes.forEach((node) => {
    const position = graph.node(node.id);
    const nodeSize = getCanvasNodeSize((node.data as EditorFlowNodeData).nodeKind);
    node.position = {
      x: position.x - 120,
      y: position.y - nodeSize.height / 2,
    };
  });

  return { nodes, edges };
}
