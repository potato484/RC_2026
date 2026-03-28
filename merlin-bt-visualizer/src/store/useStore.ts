/*
 * @Author: potato484 2220362462@qq.com
 * @Date: 2026-03-27 23:39:44
 * @LastEditors: potato484 2220362462@qq.com
 * @LastEditTime: 2026-03-28 11:07:57
 * @FilePath: /RC_2026/merlin-bt-visualizer/src/store/useStore.ts
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
import { create } from 'zustand';
import { BTNode, TimelineEvent, BlackboardItem } from '../types';
import { parseBTXml } from '../utils/btParser';
import mfTreeXml from '../../../src/rc26_decision/behavior_trees/mf_tree.xml?raw';
import combatTreeXml from '../../../src/rc26_decision/behavior_trees/combat_tree.xml?raw';
import mcTreeXml from '../../../src/rc26_decision/behavior_trees/mc_tree.xml?raw';

// Parse initial trees
const trees = {
  '梅林区': parseBTXml(mfTreeXml),
  '武馆区': parseBTXml(mcTreeXml),
  '对抗区': parseBTXml(combatTreeXml),
};

interface AppState {
  isSimulating: boolean;
  isPlaying: boolean;
  activePhase: '武馆区' | '梅林区' | '对抗区';
  nodes: BTNode[];
  edges: { id: string; source: string; target: string }[];
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
  toggleNodeCollapse: (id: string) => void;
  resetTreeState: () => void;
}

export const useStore = create<AppState>((set) => ({
  isSimulating: true,
  isPlaying: false,
  activePhase: '梅林区',
  nodes: trees['梅林区'].nodes,
  edges: trees['梅林区'].edges,
  activeNodeId: null,
  timeline: [],
  blackboard: [],

  toggleSimulate: () => set((state) => ({ isSimulating: !state.isSimulating })),
  togglePlay: () => set((state) => ({ isPlaying: !state.isPlaying })),
  setActivePhase: (phase) => set((state) => {
    if (state.activePhase === phase) return {};
    return { 
      activePhase: phase,
      nodes: trees[phase].nodes,
      edges: trees[phase].edges,
      isPlaying: false,
      timeline: [],
      blackboard: []
    };
  }),
  setActiveNode: (id) => set({ activeNodeId: id }),
  addTimelineEvent: (event) => set((state) => ({ 
    timeline: [event, ...state.timeline].slice(0, 50) 
  })),
  updateBlackboard: (item) => set((state) => {
    const exists = state.blackboard.findIndex(i => i.key === item.key);
    let newBb = [...state.blackboard];
    if (exists >= 0) newBb[exists] = item;
    else newBb = [item, ...newBb];
    return { blackboard: newBb.slice(0, 10) };
  }),
  updateNodeState: (id, nodeState) => set((state) => ({
    nodes: state.nodes.map(n => n.id === id ? { ...n, state: nodeState } : n)
  })),
  toggleNodeCollapse: (id) => set((state) => ({
    nodes: state.nodes.map(n => n.id === id ? { ...n, collapsed: !n.collapsed } : n)
  })),
  resetTreeState: () => set((state) => ({
    nodes: state.nodes.map(n => ({ ...n, state: 'idle' as const })),
    activeNodeId: null,
    isPlaying: false
  }))
}));
