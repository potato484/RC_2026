import { create } from 'zustand';
import { EditorDocument, EditorNode } from '../types/editor';
import { xmlToEditorDocument } from '../utils/editorParser';
import { editorDocumentToXml } from '../utils/editorSerializer';
import { Node as FlowNode, Edge as FlowEdge } from '@xyflow/react';
import { projectTreeToFlow } from '../utils/editorProjection';
import { BehaviorTreePhase } from '../utils/behaviorTreeSources';

interface EditorPhaseDraft {
  document: EditorDocument;
  activeTreeId: string | null;
  collapsedNodes: Set<string>;
  selectedNodeId: string | null;
}

const createDraftFromDocument = (document: EditorDocument): EditorPhaseDraft => ({
  document,
  activeTreeId: document.trees[0]?.id ?? null,
  collapsedNodes: new Set(),
  selectedNodeId: null,
});

const cloneDraft = (draft: EditorPhaseDraft): EditorPhaseDraft => ({
  ...draft,
  collapsedNodes: new Set(draft.collapsedNodes),
});

interface EditorState {
  currentPhase: BehaviorTreePhase | null;
  phaseDrafts: Partial<Record<BehaviorTreePhase, EditorPhaseDraft>>;

  // Data Model
  document: EditorDocument | null;
  activeTreeId: string | null;
  collapsedNodes: Set<string>;
  
  // React Flow State
  flowNodes: FlowNode[];
  flowEdges: FlowEdge[];
  selectedNodeId: string | null;
  
  // Actions
  ensurePhaseLoaded: (phase: BehaviorTreePhase, xmlContent: string) => void;
  setActiveTree: (treeId: string) => void;
  toggleNodeCollapse: (nodeId: string) => void;
  setSelectedNode: (nodeId: string | null) => void;
  updateNodeAttributes: (nodeId: string, attributes: Record<string, string>) => void;
  addChildNode: (parentId: string, tagName: string) => void;
  deleteNode: (nodeId: string) => void;
  exportXml: () => string | null;
  
  // Internal helper to update flow
  _updateFlow: () => void;
}

const findNodeById = (node: EditorNode, id: string): EditorNode | null => {
  if (node.id === id) return node;
  for (const child of node.children) {
    const found = findNodeById(child, id);
    if (found) return found;
  }
  return null;
};

const syncCurrentDraft = (
  state: Pick<EditorState, 'currentPhase' | 'phaseDrafts' | 'document' | 'activeTreeId' | 'collapsedNodes' | 'selectedNodeId'>,
  overrides: Partial<Pick<EditorPhaseDraft, 'document' | 'activeTreeId' | 'collapsedNodes' | 'selectedNodeId'>> = {}
): Partial<Record<BehaviorTreePhase, EditorPhaseDraft>> => {
  if (!state.currentPhase || !state.document) return state.phaseDrafts;

  const nextDraft: EditorPhaseDraft = {
    document: 'document' in overrides && overrides.document ? overrides.document : state.document,
    activeTreeId: 'activeTreeId' in overrides ? overrides.activeTreeId ?? null : state.activeTreeId,
    collapsedNodes: 'collapsedNodes' in overrides && overrides.collapsedNodes
      ? new Set(overrides.collapsedNodes)
      : new Set(state.collapsedNodes),
    selectedNodeId: 'selectedNodeId' in overrides ? overrides.selectedNodeId ?? null : state.selectedNodeId,
  };

  return {
    ...state.phaseDrafts,
    [state.currentPhase]: nextDraft,
  };
};

export const useEditorStore = create<EditorState>((set, get) => ({
  currentPhase: null,
  phaseDrafts: {},
  document: null,
  activeTreeId: null,
  collapsedNodes: new Set(),
  flowNodes: [],
  flowEdges: [],
  selectedNodeId: null,

  ensurePhaseLoaded: (phase, xmlContent) => {
    const cachedDraft = get().phaseDrafts[phase];
    if (cachedDraft) {
      const nextDraft = cloneDraft(cachedDraft);
      set({
        currentPhase: phase,
        document: nextDraft.document,
        activeTreeId: nextDraft.activeTreeId,
        collapsedNodes: nextDraft.collapsedNodes,
        selectedNodeId: nextDraft.selectedNodeId,
      });
      get()._updateFlow();
      return;
    }

    try {
      const draft = createDraftFromDocument(xmlToEditorDocument(xmlContent));
      set((state) => ({
        currentPhase: phase,
        phaseDrafts: {
          ...state.phaseDrafts,
          [phase]: cloneDraft(draft),
        },
        document: draft.document,
        activeTreeId: draft.activeTreeId,
        collapsedNodes: draft.collapsedNodes,
        selectedNodeId: draft.selectedNodeId,
      }));

      get()._updateFlow();
    } catch (error) {
      console.error('Failed to parse XML:', error);
    }
  },

  setActiveTree: (treeId: string) => {
    set((state) => ({
      activeTreeId: treeId,
      selectedNodeId: null,
      phaseDrafts: syncCurrentDraft(state, { activeTreeId: treeId, selectedNodeId: null }),
    }));
    get()._updateFlow();
  },

  toggleNodeCollapse: (nodeId: string) => {
    const { collapsedNodes } = get();
    const newCollapsed = new Set(collapsedNodes);
    if (newCollapsed.has(nodeId)) {
      newCollapsed.delete(nodeId);
    } else {
      newCollapsed.add(nodeId);
    }
    set((state) => ({
      collapsedNodes: newCollapsed,
      phaseDrafts: syncCurrentDraft(state, { collapsedNodes: newCollapsed }),
    }));
    get()._updateFlow();
  },

  setSelectedNode: (nodeId: string | null) => {
    set((state) => ({
      selectedNodeId: nodeId,
      phaseDrafts: syncCurrentDraft(state, { selectedNodeId: nodeId }),
    }));
  },

  updateNodeAttributes: (nodeId: string, attributes: Record<string, string>) => {
    const { document, activeTreeId } = get();
    if (!document || !activeTreeId) return;

    // Deep clone the document to ensure reactivity
    const newDoc = JSON.parse(JSON.stringify(document)) as EditorDocument;
    
    // Find the node and update
    let updated = false;
    for (const tree of newDoc.trees) {
      const targetNode = findNodeById(tree.rootNode, nodeId);
      if (targetNode) {
        targetNode.attributes = { ...attributes };
        updated = true;
        break;
      }
    }

    if (updated) {
      set((state) => ({
        document: newDoc,
        phaseDrafts: syncCurrentDraft(state, { document: newDoc }),
      }));
      get()._updateFlow();
    }
  },

  addChildNode: (parentId: string, tagName: string) => {
    const { document, activeTreeId } = get();
    if (!document || !activeTreeId) return;

    const newDoc = JSON.parse(JSON.stringify(document)) as EditorDocument;
    
    let updated = false;
    for (const tree of newDoc.trees) {
      const parentNode = findNodeById(tree.rootNode, parentId);
      if (parentNode) {
        // Determine type based on basic heuristic
        let uiType: 'control' | 'decorator' | 'leaf' | 'subtree' = 'leaf';
        if (tagName === 'SubTree') uiType = 'subtree';
        else if (['Sequence', 'Fallback', 'ReactiveSequence', 'ReactiveFallback'].includes(tagName)) uiType = 'control';
        else if (['Inverter', 'RetryUntilSuccessful', 'KeepRunningUntilFailure', 'Delay', 'ForceSuccess', 'ForceFailure'].includes(tagName)) uiType = 'decorator';

        const newNode: EditorNode = {
          id: `node_${Math.random().toString(36).substr(2, 9)}`,
          tagName,
          attributes: {},
          children: [],
          uiType
        };
        
        parentNode.children.push(newNode);
        updated = true;
        break;
      }
    }

    if (updated) {
      set((state) => ({
        document: newDoc,
        phaseDrafts: syncCurrentDraft(state, { document: newDoc }),
      }));
      get()._updateFlow();
    }
  },

  deleteNode: (nodeId: string) => {
    const { document, activeTreeId } = get();
    if (!document || !activeTreeId) return;

    const newDoc = JSON.parse(JSON.stringify(document)) as EditorDocument;
    
    let updated = false;

    const deleteFromParent = (parent: EditorNode, id: string): boolean => {
      const index = parent.children.findIndex(c => c.id === id);
      if (index !== -1) {
        parent.children.splice(index, 1);
        return true;
      }
      for (const child of parent.children) {
        if (deleteFromParent(child, id)) return true;
      }
      return false;
    };

    for (const tree of newDoc.trees) {
      if (tree.rootNode.id === nodeId) {
        // Cannot delete root node directly like this
        console.warn("Cannot delete the root node of a tree");
        break;
      }
      if (deleteFromParent(tree.rootNode, nodeId)) {
        updated = true;
        break;
      }
    }

    if (updated) {
      set((state) => ({
        document: newDoc,
        selectedNodeId: null,
        phaseDrafts: syncCurrentDraft(state, { document: newDoc, selectedNodeId: null }),
      }));
      get()._updateFlow();
    }
  },

  exportXml: () => {
    const { document } = get();
    if (!document) return null;
    return editorDocumentToXml(document);
  },

  _updateFlow: () => {
    const { document, activeTreeId, collapsedNodes } = get();
    if (!document || !activeTreeId) {
      set({ flowNodes: [], flowEdges: [] });
      return;
    }

    const activeTree = document.trees.find(t => t.id === activeTreeId);
    if (!activeTree) {
      set({ flowNodes: [], flowEdges: [] });
      return;
    }

    const { nodes, edges } = projectTreeToFlow(activeTree, collapsedNodes);
    set({ flowNodes: nodes, flowEdges: edges });
  }
}));
