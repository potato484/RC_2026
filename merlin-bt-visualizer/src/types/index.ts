export type NodeState = 'idle' | 'running' | 'success' | 'failure';

export interface BTNode {
  id: string;
  type: 'action' | 'condition' | 'sequence' | 'selector' | 'decorator' | 'subtree';
  label: string;
  state: NodeState;
  desc: string;
  parentId?: string;
  siblingIndex?: number;
  treeId?: string;
  subTreeId?: string;
  collapsed?: boolean;
}

export interface ParsedTree {
  id: string;
  name?: string;
  nodes: BTNode[];
  edges: { id: string; source: string; target: string }[];
}

export interface ParsedArea {
  mainTreeId: string;
  trees: Record<string, ParsedTree>;
}

export interface TimelineEvent {
  id: string;
  time: string;
  desc: string;
  icon: 'scan' | 'move' | 'grab' | 'wait' | 'check';
  status: 'info' | 'success' | 'warning';
}

export interface BlackboardItem {
  key: string;
  value: string;
  desc: string;
  updatedAt: number;
}
