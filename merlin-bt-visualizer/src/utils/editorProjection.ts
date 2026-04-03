import { Node as FlowNode, Edge as FlowEdge } from '@xyflow/react';
import dagre from 'dagre';
import { EditorTree, EditorNode } from '../types/editor';
import { getBehaviorTreeNodeDisplay, summarizeBehaviorTreeAttributes } from './btDisplay';

export interface EditorFlowNodeData extends Record<string, unknown> {
  editorNodeId: string;
  tagName: string;
  displayLabel: string;
  displayDesc: string;
  attributeSummary: string;
  attributes: Record<string, string>;
  uiType: 'control' | 'decorator' | 'leaf' | 'subtree';
  isRoot?: boolean;
}

export function projectTreeToFlow(
  tree: EditorTree,
  collapsedNodeIds: Set<string>
): { nodes: FlowNode[]; edges: FlowEdge[] } {
  const nodes: FlowNode[] = [];
  const edges: FlowEdge[] = [];

  const traverse = (node: EditorNode, parentId?: string, isRoot = false) => {
    // Determine if this node should be visible based on parent's collapsed state
    // Actually, React Flow can handle hiding nodes if we just filter them out,
    // but dagre layout needs to know about visibility to layout correctly.
    
    const display = getBehaviorTreeNodeDisplay(node.tagName, node.attributes);
    const nodeData: EditorFlowNodeData = {
      editorNodeId: node.id,
      tagName: node.tagName,
      displayLabel: display.label,
      displayDesc: display.desc,
      attributeSummary: summarizeBehaviorTreeAttributes(node.attributes).join(' · '),
      attributes: { ...node.attributes },
      uiType: node.uiType,
      isRoot
    };

    nodes.push({
      id: node.id,
      type: 'editorNode', // Custom node type for editor
      position: { x: 0, y: 0 }, // Will be set by layout
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
      node.children.forEach(child => traverse(child, node.id));
    }
  };

  if (tree.rootNode) {
    traverse(tree.rootNode, undefined, true);
  }

  // Layout with dagre
  const dagreGraph = new dagre.graphlib.Graph();
  dagreGraph.setDefaultEdgeLabel(() => ({}));
  
  dagreGraph.setGraph({ 
    rankdir: 'LR',
    nodesep: 50,
    ranksep: 100,
    edgesep: 20,
    marginx: 20,
    marginy: 20
  });

  const getNodeSize = (uiType: string) => {
    switch (uiType) {
      case 'control':
        return { width: 160, height: 50 };
      case 'decorator':
        return { width: 180, height: 50 };
      case 'subtree':
      case 'leaf':
      default:
        return { width: 220, height: 60 };
    }
  };

  nodes.forEach((node) => {
    const { height } = getNodeSize((node.data as unknown as EditorFlowNodeData).uiType);
    dagreGraph.setNode(node.id, { width: 220, height }); // Fixed width for alignment
  });

  edges.forEach((edge) => {
    dagreGraph.setEdge(edge.source, edge.target);
  });

  dagre.layout(dagreGraph);

  nodes.forEach((node) => {
    const nodeWithPosition = dagreGraph.node(node.id);
    const { height } = getNodeSize((node.data as unknown as EditorFlowNodeData).uiType);
    node.position = {
      x: nodeWithPosition.x - 110, // Half of 220
      y: nodeWithPosition.y - height / 2,
    };
  });

  return { nodes, edges };
}
