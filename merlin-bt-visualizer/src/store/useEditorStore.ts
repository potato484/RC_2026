import { create } from 'zustand';
import { Edge as FlowEdge, Node as FlowNode } from '@xyflow/react';
import { EditorDocument, EditorNode } from '../types/editor';
import { BehaviorTreePhase } from '../utils/behaviorTreeSources';
import { xmlToEditorDocument } from '../utils/editorParser';
import { editorDocumentToXml } from '../utils/editorSerializer';
import { projectTreeToFlow } from '../utils/editorProjection';
import {
  canNodeAcceptChildren,
  EditorInsertPosition,
  createNodeFromDefinition,
  enrichEditorNode,
  getBtNodeDefinition,
  getCompositeSwitchGroupEntries,
  refreshEditorDocumentNodes,
} from '../utils/btRegistry';

interface EditorPhaseDraft {
  document: EditorDocument;
  activeTreeId: string | null;
  collapsedNodes: Set<string>;
  selectedNodeId: string | null;
}

interface NodeLocation {
  treeIndex: number;
  treeRoot: EditorNode;
  node: EditorNode;
  parent: EditorNode | null;
  childIndex: number;
}

interface EditorState {
  currentPhase: BehaviorTreePhase | null;
  phaseDrafts: Partial<Record<BehaviorTreePhase, EditorPhaseDraft>>;
  document: EditorDocument | null;
  activeTreeId: string | null;
  collapsedNodes: Set<string>;
  flowNodes: FlowNode[];
  flowEdges: FlowEdge[];
  selectedNodeId: string | null;
  ensurePhaseLoaded: (phase: BehaviorTreePhase, xmlContent: string) => void;
  setActiveTree: (treeId: string) => void;
  toggleNodeCollapse: (nodeId: string) => void;
  setSelectedNode: (nodeId: string | null) => void;
  updateNodeAttributes: (nodeId: string, attributes: Record<string, string>) => void;
  updateSingleAttribute: (nodeId: string, key: string, value: string) => void;
  insertNode: (nodeId: string, position: EditorInsertPosition, tagName: string) => void;
  replaceNodeType: (nodeId: string, tagName: string) => void;
  cycleCompositeType: (nodeId: string) => void;
  deleteNode: (nodeId: string) => void;
  exportXml: () => string | null;
  _updateFlow: () => void;
}

const createDraftFromDocument = (document: EditorDocument): EditorPhaseDraft => ({
  document,
  activeTreeId: document.trees[0]?.id ?? null,
  collapsedNodes: new Set(),
  selectedNodeId: null,
});

const cloneDraft = (draft: EditorPhaseDraft): EditorPhaseDraft => ({
  document: JSON.parse(JSON.stringify(draft.document)) as EditorDocument,
  activeTreeId: draft.activeTreeId,
  collapsedNodes: new Set(draft.collapsedNodes),
  selectedNodeId: draft.selectedNodeId,
});

const cloneDocument = (document: EditorDocument): EditorDocument =>
  refreshEditorDocumentNodes(JSON.parse(JSON.stringify(document)) as EditorDocument);

const findNodeLocation = (document: EditorDocument, nodeId: string): NodeLocation | null => {
  const visit = (
    treeIndex: number,
    node: EditorNode,
    parent: EditorNode | null
  ): NodeLocation | null => {
    if (node.id === nodeId) {
      return {
        treeIndex,
        treeRoot: document.trees[treeIndex].rootNode,
        node,
        parent,
        childIndex: parent ? parent.children.findIndex((child) => child.id === nodeId) : 0,
      };
    }

    for (const child of node.children) {
      const found = visit(treeIndex, child, node);
      if (found) {
        return found;
      }
    }

    return null;
  };

  for (let treeIndex = 0; treeIndex < document.trees.length; treeIndex += 1) {
    const found = visit(treeIndex, document.trees[treeIndex].rootNode, null);
    if (found) {
      return found;
    }
  }

  return null;
};

const syncCurrentDraft = (
  state: Pick<
    EditorState,
    'currentPhase' | 'phaseDrafts' | 'document' | 'activeTreeId' | 'collapsedNodes' | 'selectedNodeId'
  >,
  overrides: Partial<Pick<EditorPhaseDraft, 'document' | 'activeTreeId' | 'collapsedNodes' | 'selectedNodeId'>> = {}
): Partial<Record<BehaviorTreePhase, EditorPhaseDraft>> => {
  if (!state.currentPhase || !state.document) {
    return state.phaseDrafts;
  }

  return {
    ...state.phaseDrafts,
    [state.currentPhase]: {
      document: overrides.document ?? state.document,
      activeTreeId: overrides.activeTreeId !== undefined ? overrides.activeTreeId : state.activeTreeId,
      collapsedNodes: new Set(overrides.collapsedNodes ?? state.collapsedNodes),
      selectedNodeId: overrides.selectedNodeId !== undefined ? overrides.selectedNodeId : state.selectedNodeId,
    },
  };
};

const updateDocumentState = (
  set: (partial:
    | Partial<EditorState>
    | ((state: EditorState) => Partial<EditorState>)) => void,
  get: () => EditorState,
  document: EditorDocument,
  selectedNodeId?: string | null
) => {
  set((state) => ({
    document,
    selectedNodeId: selectedNodeId !== undefined ? selectedNodeId : state.selectedNodeId,
    phaseDrafts: syncCurrentDraft(state, {
      document,
      selectedNodeId: selectedNodeId !== undefined ? selectedNodeId : state.selectedNodeId,
    }),
  }));
  get()._updateFlow();
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
      const draft = cloneDraft(cachedDraft);
      set({
        currentPhase: phase,
        document: draft.document,
        activeTreeId: draft.activeTreeId,
        collapsedNodes: draft.collapsedNodes,
        selectedNodeId: draft.selectedNodeId,
      });
      get()._updateFlow();
      return;
    }

    const draft = createDraftFromDocument(xmlToEditorDocument(xmlContent));
    set((state) => ({
      currentPhase: phase,
      document: draft.document,
      activeTreeId: draft.activeTreeId,
      collapsedNodes: draft.collapsedNodes,
      selectedNodeId: draft.selectedNodeId,
      phaseDrafts: {
        ...state.phaseDrafts,
        [phase]: cloneDraft(draft),
      },
    }));
    get()._updateFlow();
  },

  setActiveTree: (treeId) => {
    set((state) => ({
      activeTreeId: treeId,
      selectedNodeId: null,
      phaseDrafts: syncCurrentDraft(state, {
        activeTreeId: treeId,
        selectedNodeId: null,
      }),
    }));
    get()._updateFlow();
  },

  toggleNodeCollapse: (nodeId) => {
    const collapsedNodes = new Set(get().collapsedNodes);
    if (collapsedNodes.has(nodeId)) {
      collapsedNodes.delete(nodeId);
    } else {
      collapsedNodes.add(nodeId);
    }
    set((state) => ({
      collapsedNodes,
      phaseDrafts: syncCurrentDraft(state, { collapsedNodes }),
    }));
    get()._updateFlow();
  },

  setSelectedNode: (nodeId) => {
    set((state) => ({
      selectedNodeId: nodeId,
      phaseDrafts: syncCurrentDraft(state, { selectedNodeId: nodeId }),
    }));
  },

  updateNodeAttributes: (nodeId, attributes) => {
    const { document } = get();
    if (!document) {
      return;
    }

    const nextDocument = cloneDocument(document);
    const location = findNodeLocation(nextDocument, nodeId);
    if (!location) {
      return;
    }

    location.node.attributes = { ...attributes };
    location.node.portBindings = enrichEditorNode(location.node).portBindings;
    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(nextDocument.trees[location.treeIndex].rootNode);
    updateDocumentState(set, get, nextDocument);
  },

  updateSingleAttribute: (nodeId, key, value) => {
    const { document } = get();
    if (!document) {
      return;
    }

    const nextDocument = cloneDocument(document);
    const location = findNodeLocation(nextDocument, nodeId);
    if (!location) {
      return;
    }

    if (!value.trim()) {
      delete location.node.attributes[key];
    } else {
      location.node.attributes[key] = value;
    }
    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(nextDocument.trees[location.treeIndex].rootNode);
    updateDocumentState(set, get, nextDocument);
  },

  insertNode: (nodeId, position, tagName) => {
    const { document } = get();
    if (!document) {
      return;
    }

    const nextDocument = cloneDocument(document);
    const location = findNodeLocation(nextDocument, nodeId);
    if (!location) {
      return;
    }

    const nextNode = createNodeFromDefinition(tagName);

    if (position === 'wrap') {
      nextNode.children = [location.node];
      if (location.parent) {
        location.parent.children.splice(location.childIndex, 1, nextNode);
      } else {
        nextDocument.trees[location.treeIndex].rootNode = nextNode;
      }
      nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(nextDocument.trees[location.treeIndex].rootNode);
      updateDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), nextNode.id);
      return;
    }

    if (position === 'prepend_child' || position === 'append_child') {
      if (!canNodeAcceptChildren(location.node)) {
        if (!location.parent) {
          return;
        }
        const fallbackSiblingIndex = position === 'prepend_child' ? location.childIndex : location.childIndex + 1;
        location.parent.children.splice(fallbackSiblingIndex, 0, nextNode);
        nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(nextDocument.trees[location.treeIndex].rootNode);
        updateDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), nextNode.id);
        return;
      }
      const childIndex = position === 'prepend_child' ? 0 : location.node.children.length;
      location.node.children.splice(childIndex, 0, nextNode);
      nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(nextDocument.trees[location.treeIndex].rootNode);
      updateDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), nextNode.id);
      return;
    }

    if (!location.parent) {
      return;
    }

    const siblingIndex = position === 'before' ? location.childIndex : location.childIndex + 1;
    location.parent.children.splice(siblingIndex, 0, nextNode);
    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(nextDocument.trees[location.treeIndex].rootNode);
    updateDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), nextNode.id);
  },

  replaceNodeType: (nodeId, tagName) => {
    const { document } = get();
    if (!document) {
      return;
    }

    const nextDocument = cloneDocument(document);
    const location = findNodeLocation(nextDocument, nodeId);
    if (!location) {
      return;
    }

    const definition = getBtNodeDefinition(tagName);
    location.node.tagName = tagName;
    location.node.definitionId = definition?.id ?? tagName;
    location.node.attributes = {
      ...(definition?.defaultAttributes ?? {}),
      ...location.node.attributes,
    };

    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(nextDocument.trees[location.treeIndex].rootNode);
    updateDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), location.node.id);
  },

  cycleCompositeType: (nodeId) => {
    const { document } = get();
    if (!document) {
      return;
    }
    const location = findNodeLocation(document, nodeId);
    if (!location) {
      return;
    }

    const candidates = getCompositeSwitchGroupEntries(location.node.tagName);
    if (candidates.length <= 1) {
      return;
    }
    const currentIndex = candidates.findIndex((entry) => entry.tagName === location.node.tagName);
    const nextEntry = candidates[(currentIndex + 1) % candidates.length];
    get().replaceNodeType(nodeId, nextEntry.tagName);
  },

  deleteNode: (nodeId) => {
    const { document } = get();
    if (!document) {
      return;
    }

    const nextDocument = cloneDocument(document);
    const location = findNodeLocation(nextDocument, nodeId);
    if (!location || !location.parent) {
      return;
    }

    location.parent.children.splice(location.childIndex, 1);
    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(nextDocument.trees[location.treeIndex].rootNode);
    updateDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), null);
  },

  exportXml: () => {
    const { document } = get();
    if (!document) {
      return null;
    }
    return editorDocumentToXml(document);
  },

  _updateFlow: () => {
    const { document, activeTreeId, collapsedNodes } = get();
    if (!document || !activeTreeId) {
      set({ flowNodes: [], flowEdges: [] });
      return;
    }

    const activeTree = document.trees.find((tree) => tree.id === activeTreeId);
    if (!activeTree) {
      set({ flowNodes: [], flowEdges: [] });
      return;
    }

    const { nodes, edges } = projectTreeToFlow(activeTree, collapsedNodes);
    set({ flowNodes: nodes, flowEdges: edges });
  },
}));
