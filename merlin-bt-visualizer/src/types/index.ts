export type NodeState = 'idle' | 'running' | 'success' | 'failure';

export interface BTNode {
  id: string;
  type: 'action' | 'condition' | 'sequence' | 'selector' | 'decorator' | 'subtree';
  label: string;
  state: NodeState;
  desc: string;
  parentId?: string;
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
