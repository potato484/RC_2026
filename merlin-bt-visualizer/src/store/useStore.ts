import { create } from 'zustand';
import { BTNode, TimelineEvent, BlackboardItem } from '../types';
import { mockNodes } from '../mock/treeData';

interface AppState {
  isSimulating: boolean;
  isPlaying: boolean;
  activePhase: '武馆区' | '梅林区' | '对抗区';
  nodes: BTNode[];
  activeNodeId: string | null;
  timeline: TimelineEvent[];
  blackboard: BlackboardItem[];
  
  toggleSimulate: () => void;
  togglePlay: () => void;
  setActivePhase: (phase: '武馆区' | '梅林区' | '对抗区') => void;
  setActiveNode: (id: string | null) => void;
  addTimelineEvent: (event: TimelineEvent) => void;
  updateBlackboard: (item: BlackboardItem) => void;
  updateNodeState: (id: string, state: BTNode['state']) => void;
}

export const useStore = create<AppState>((set) => ({
  isSimulating: true,
  isPlaying: true,
  activePhase: '梅林区',
  nodes: mockNodes,
  activeNodeId: null,
  timeline: [
    { id: '1', time: '10:00:00', desc: '进入梅林区', icon: 'move', status: 'info' }
  ],
  blackboard: [
    { key: 'target_found', value: '否', desc: '是否发现目标', updatedAt: Date.now() },
    { key: 'distance', value: '未知', desc: '距离目标距离', updatedAt: Date.now() }
  ],

  toggleSimulate: () => set((state) => ({ isSimulating: !state.isSimulating })),
  togglePlay: () => set((state) => ({ isPlaying: !state.isPlaying })),
  setActivePhase: (phase) => set({ activePhase: phase }),
  setActiveNode: (id) => set({ activeNodeId: id }),
  addTimelineEvent: (event) => set((state) => ({ 
    timeline: [event, ...state.timeline].slice(0, 10) 
  })),
  updateBlackboard: (item) => set((state) => {
    const exists = state.blackboard.findIndex(i => i.key === item.key);
    let newBb = [...state.blackboard];
    if (exists >= 0) newBb[exists] = item;
    else newBb = [item, ...newBb];
    return { blackboard: newBb.slice(0, 6) };
  }),
  updateNodeState: (id, nodeState) => set((state) => ({
    nodes: state.nodes.map(n => n.id === id ? { ...n, state: nodeState } : n)
  }))
}));
