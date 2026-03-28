
import { create } from 'zustand';
import { BTNode, TimelineEvent, BlackboardItem, ParsedArea, ParsedTree } from '../types';
import { parseBTXml } from '../utils/btParser';
import mfTreeXml from '../../../src/rc26_decision/behavior_trees/mf_tree.xml?raw';
import combatTreeXml from '../../../src/rc26_decision/behavior_trees/combat_tree.xml?raw';
import mcTreeXml from '../../../src/rc26_decision/behavior_trees/mc_tree.xml?raw';

// Parse initial trees
const areas: Record<'梅林区' | '武馆区' | '对抗区', ParsedArea> = {
  '梅林区': parseBTXml(mfTreeXml),
  '武馆区': parseBTXml(mcTreeXml),
  '对抗区': parseBTXml(combatTreeXml),
};

interface AppState {
  isSimulating: boolean;
  isPlaying: boolean;
  activePhase: '武馆区' | '梅林区' | '对抗区';
  activeTreeId: string;
  trees: Record<string, ParsedTree>;
  nodes: BTNode[];
  edges: { id: string; source: string; target: string }[];
  activeNodeId: string | null;
  timeline: TimelineEvent[];
  blackboard: BlackboardItem[];
  
  toggleSimulate: () => void;
  togglePlay: () => void;
  stopPlay: () => void;
  setActivePhase: (phase: '武馆区' | '梅林区' | '对抗区') => void;
  setActiveTree: (treeId: string) => void;
  setActiveNode: (id: string | null) => void;
  addTimelineEvent: (event: TimelineEvent) => void;
  clearTimeline: () => void;
  updateBlackboard: (item: BlackboardItem) => void;
  updateNodeState: (id: string, state: BTNode['state']) => void;
  toggleNodeCollapse: (id: string) => void;
  resetTreeState: () => void;
}

export const useStore = create<AppState>((set) => {
  const initialPhase = '梅林区';
  const initialArea = areas[initialPhase];
  const initialTreeId = initialArea.mainTreeId || Object.keys(initialArea.trees)[0];
  const initialTree = initialArea.trees[initialTreeId] || { nodes: [], edges: [] };

  return {
    isSimulating: true,
    isPlaying: false,
    activePhase: initialPhase,
    activeTreeId: initialTreeId,
    trees: initialArea.trees,
    nodes: initialTree.nodes,
    edges: initialTree.edges,
    activeNodeId: null,
    timeline: [],
    blackboard: [],

    toggleSimulate: () => set((state) => {
      const nextSim = !state.isSimulating;
      return { 
        isSimulating: nextSim,
        isPlaying: nextSim // 自动在模拟时执行，实机时暂停
      };
    }),
    togglePlay: () => set((state) => ({ isPlaying: !state.isPlaying })),
    stopPlay: () => set({ isPlaying: false }),
    
    setActivePhase: (phase) => set((state) => {
      if (state.activePhase === phase) return {};
      const area = areas[phase];
      const treeId = area.mainTreeId || Object.keys(area.trees)[0];
      const tree = area.trees[treeId] || { nodes: [], edges: [] };
      return { 
        activePhase: phase,
        activeTreeId: treeId,
        trees: area.trees,
        nodes: tree.nodes,
        edges: tree.edges,
        timeline: [],
        blackboard: []
      };
    }),

    setActiveTree: (treeId) => set((state) => {
      if (state.activeTreeId === treeId) return {};
      const tree = state.trees[treeId];
      if (!tree) return {};
      return {
        activeTreeId: treeId,
        nodes: tree.nodes,
        edges: tree.edges,
        activeNodeId: null
      };
    }),

    setActiveNode: (id) => set({ activeNodeId: id }),
    
    addTimelineEvent: (event) => set((state) => ({ 
      timeline: [event, ...state.timeline].slice(0, 50) 
    })),
    
    clearTimeline: () => set({ timeline: [] }),
    
    updateBlackboard: (item) => set((state) => {
      const exists = state.blackboard.findIndex(i => i.key === item.key);
      let newBb = [...state.blackboard];
      if (exists >= 0) newBb[exists] = item;
      else newBb = [item, ...newBb];
      return { blackboard: newBb.slice(0, 10) };
    }),
    
    updateNodeState: (id, nodeState) => set((state) => {
      // Need to update state in the specific tree
      const newTrees = { ...state.trees };
      let updated = false;
      let newNodes = state.nodes;

      for (const treeId in newTrees) {
        const tree = newTrees[treeId];
        const nodeIndex = tree.nodes.findIndex(n => n.id === id);
        if (nodeIndex !== -1) {
          const newTreeNodes = [...tree.nodes];
          newTreeNodes[nodeIndex] = { ...newTreeNodes[nodeIndex], state: nodeState };
          newTrees[treeId] = { ...tree, nodes: newTreeNodes };
          
          if (treeId === state.activeTreeId) {
            newNodes = newTreeNodes;
          }
          updated = true;
          break; // node IDs are unique across trees due to prefix
        }
      }

      if (!updated) return {};
      return { trees: newTrees, nodes: newNodes };
    }),

    toggleNodeCollapse: (id) => set((state) => {
      const newTrees = { ...state.trees };
      let updated = false;
      let newNodes = state.nodes;

      for (const treeId in newTrees) {
        const tree = newTrees[treeId];
        const nodeIndex = tree.nodes.findIndex(n => n.id === id);
        if (nodeIndex !== -1) {
          const newTreeNodes = [...tree.nodes];
          newTreeNodes[nodeIndex] = { ...newTreeNodes[nodeIndex], collapsed: !newTreeNodes[nodeIndex].collapsed };
          newTrees[treeId] = { ...tree, nodes: newTreeNodes };
          
          if (treeId === state.activeTreeId) {
            newNodes = newTreeNodes;
          }
          updated = true;
          break;
        }
      }

      if (!updated) return {};
      return { trees: newTrees, nodes: newNodes };
    }),

    resetTreeState: () => set((state) => {
      const newTrees: Record<string, ParsedTree> = {};
      let newNodes = state.nodes;
      
      for (const treeId in state.trees) {
        const tree = state.trees[treeId];
        const resetNodes = tree.nodes.map(n => ({ ...n, state: 'idle' as const }));
        newTrees[treeId] = { ...tree, nodes: resetNodes };
        if (treeId === state.activeTreeId) {
          newNodes = resetNodes;
        }
      }

      return {
        trees: newTrees,
        nodes: newNodes,
        activeNodeId: null
      };
    })
  };
});
