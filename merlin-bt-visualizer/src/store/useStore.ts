
import { create } from 'zustand';
import { BTNode, TimelineEvent, BlackboardItem, ParsedArea, ParsedTree } from '../types';
import { parseBTXml } from '../utils/btParser';
import {
  BehaviorTreePhase,
  getBehaviorTreeXmlForPhase,
  setBehaviorTreeXmlForPhase
} from '../utils/behaviorTreeSources';

const buildParsedArea = (phase: BehaviorTreePhase): ParsedArea => parseBTXml(getBehaviorTreeXmlForPhase(phase));

const parsedAreas: Record<BehaviorTreePhase, ParsedArea> = {
  '梅林区': buildParsedArea('梅林区'),
  '武馆区': buildParsedArea('武馆区'),
  '对抗区': buildParsedArea('对抗区'),
};

const getParsedArea = (phase: BehaviorTreePhase): ParsedArea => parsedAreas[phase];

const replaceParsedArea = (phase: BehaviorTreePhase, xmlContent: string): ParsedArea => {
  setBehaviorTreeXmlForPhase(phase, xmlContent);
  const nextArea = parseBTXml(xmlContent);
  parsedAreas[phase] = nextArea;
  return nextArea;
};

interface AppState {
  appMode: 'viewer' | 'editor';
  isSimulating: boolean;
  isPlaying: boolean;
  activePhase: BehaviorTreePhase;
  activeTreeId: string;
  trees: Record<string, ParsedTree>;
  nodes: BTNode[];
  edges: { id: string; source: string; target: string }[];
  activeNodeId: string | null;
  timeline: TimelineEvent[];
  blackboard: BlackboardItem[];
  
  setAppMode: (mode: 'viewer' | 'editor') => void;
  toggleSimulate: () => void;
  togglePlay: () => void;
  stopPlay: () => void;
  setActivePhase: (phase: BehaviorTreePhase) => void;
  setActiveTree: (treeId: string) => void;
  setActiveNode: (id: string | null) => void;
  addTimelineEvent: (event: TimelineEvent) => void;
  clearTimeline: () => void;
  updateBlackboard: (item: BlackboardItem) => void;
  updateNodeState: (id: string, state: BTNode['state']) => void;
  toggleNodeCollapse: (id: string) => void;
  replacePhaseXml: (phase: BehaviorTreePhase, xmlContent: string) => void;
  resetTreeState: () => void;
}

export const useStore = create<AppState>((set) => {
  const initialPhase = '梅林区';
  const initialArea = getParsedArea(initialPhase);
  const initialTreeId = initialArea.mainTreeId || Object.keys(initialArea.trees)[0];
  const initialTree = initialArea.trees[initialTreeId] || { nodes: [], edges: [] };

  return {
    appMode: 'viewer',
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

    setAppMode: (mode) => set({ appMode: mode }),
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
      const area = getParsedArea(phase);
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

    replacePhaseXml: (phase, xmlContent) => set((state) => {
      const nextArea = replaceParsedArea(phase, xmlContent);
      if (state.activePhase !== phase) return {};

      const nextActiveTreeId = nextArea.trees[state.activeTreeId]
        ? state.activeTreeId
        : nextArea.mainTreeId || Object.keys(nextArea.trees)[0];
      const nextTree = nextArea.trees[nextActiveTreeId] || { nodes: [], edges: [] };

      return {
        trees: nextArea.trees,
        activeTreeId: nextActiveTreeId,
        nodes: nextTree.nodes,
        edges: nextTree.edges,
        activeNodeId: null,
      };
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
