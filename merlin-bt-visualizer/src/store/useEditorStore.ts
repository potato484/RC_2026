import { create } from 'zustand';
import { Edge as FlowEdge, Node as FlowNode } from '@xyflow/react';
import {
  EditorDocument,
  EditorHistoryEntry,
  EditorInsertTemplate,
  EditorNode,
} from '../types/editor';
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

const HISTORY_LIMIT = 60;

interface EditorPhaseDraft {
  document: EditorDocument;
  activeTreeId: string | null;
  collapsedNodes: Set<string>;
  selectedNodeId: string | null;
  history: EditorHistoryEntry[];
  future: EditorHistoryEntry[];
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
  canUndo: boolean;
  canRedo: boolean;
  ensurePhaseLoaded: (phase: BehaviorTreePhase, xmlContent: string) => void;
  setActiveTree: (treeId: string) => void;
  toggleNodeCollapse: (nodeId: string) => void;
  setSelectedNode: (nodeId: string | null) => void;
  updateNodeAttributes: (nodeId: string, attributes: Record<string, string>) => void;
  updateSingleAttribute: (nodeId: string, key: string, value: string) => void;
  insertNode: (nodeId: string, position: EditorInsertPosition, tagName: string) => void;
  insertNodeTemplate: (nodeId: string, position: EditorInsertPosition, template: EditorInsertTemplate) => void;
  insertNodeOnEdge: (parentNodeId: string, childNodeId: string, template: EditorInsertTemplate) => void;
  replaceNodeType: (nodeId: string, tagName: string) => void;
  cycleCompositeType: (nodeId: string) => void;
  deleteNode: (nodeId: string) => void;
  undo: () => void;
  redo: () => void;
  exportXml: () => string | null;
  _updateFlow: () => void;
}

const cloneHistoryEntry = (entry: EditorHistoryEntry): EditorHistoryEntry => ({
  document: JSON.parse(JSON.stringify(entry.document)) as EditorDocument,
  selectedNodeId: entry.selectedNodeId,
});

const snapshotDocument = (
  document: EditorDocument,
  selectedNodeId: string | null
): EditorHistoryEntry => ({
  document: JSON.parse(JSON.stringify(document)) as EditorDocument,
  selectedNodeId,
});

const createDraftFromDocument = (document: EditorDocument): EditorPhaseDraft => ({
  document,
  activeTreeId: document.trees[0]?.id ?? null,
  collapsedNodes: new Set(),
  selectedNodeId: null,
  history: [],
  future: [],
});

const cloneDraft = (draft: EditorPhaseDraft): EditorPhaseDraft => ({
  document: JSON.parse(JSON.stringify(draft.document)) as EditorDocument,
  activeTreeId: draft.activeTreeId,
  collapsedNodes: new Set(draft.collapsedNodes),
  selectedNodeId: draft.selectedNodeId,
  history: draft.history.map(cloneHistoryEntry),
  future: draft.future.map(cloneHistoryEntry),
});

const cloneDocument = (document: EditorDocument): EditorDocument =>
  refreshEditorDocumentNodes(JSON.parse(JSON.stringify(document)) as EditorDocument);

const trimHistory = (history: EditorHistoryEntry[]): EditorHistoryEntry[] =>
  history.length > HISTORY_LIMIT ? history.slice(history.length - HISTORY_LIMIT) : history;

const getHistoryFlags = (draft?: Pick<EditorPhaseDraft, 'history' | 'future'> | null) => ({
  canUndo: (draft?.history.length ?? 0) > 0,
  canRedo: (draft?.future.length ?? 0) > 0,
});

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
  overrides: Partial<
    Pick<
      EditorPhaseDraft,
      'document' | 'activeTreeId' | 'collapsedNodes' | 'selectedNodeId' | 'history' | 'future'
    >
  > = {}
): Partial<Record<BehaviorTreePhase, EditorPhaseDraft>> => {
  if (!state.currentPhase || !state.document) {
    return state.phaseDrafts;
  }

  const currentDraft = state.phaseDrafts[state.currentPhase];

  return {
    ...state.phaseDrafts,
    [state.currentPhase]: {
      document: overrides.document ?? state.document,
      activeTreeId: overrides.activeTreeId !== undefined ? overrides.activeTreeId : state.activeTreeId,
      collapsedNodes: new Set(overrides.collapsedNodes ?? state.collapsedNodes),
      selectedNodeId:
        overrides.selectedNodeId !== undefined ? overrides.selectedNodeId : state.selectedNodeId,
      history: (overrides.history ?? currentDraft?.history ?? []).map(cloneHistoryEntry),
      future: (overrides.future ?? currentDraft?.future ?? []).map(cloneHistoryEntry),
    },
  };
};

const createNodeFromTemplate = (template: EditorInsertTemplate): EditorNode => {
  const node = createNodeFromDefinition(template.tagName);
  if (!template.presetAttributes) {
    return node;
  }

  node.attributes = {
    ...node.attributes,
    ...template.presetAttributes,
  };

  return enrichEditorNode(node);
};

const replaceDocumentState = (
  set: (
    partial: Partial<EditorState> | ((state: EditorState) => Partial<EditorState>)
  ) => void,
  get: () => EditorState,
  document: EditorDocument,
  options: {
    selectedNodeId?: string | null;
    history?: EditorHistoryEntry[];
    future?: EditorHistoryEntry[];
    pushHistory?: boolean;
  } = {}
) => {
  set((state) => {
    const selectedNodeId =
      options.selectedNodeId !== undefined ? options.selectedNodeId : state.selectedNodeId;
    const currentDraft = state.currentPhase ? state.phaseDrafts[state.currentPhase] : undefined;
    const history =
      options.history ??
      (options.pushHistory === false
        ? currentDraft?.history ?? []
        : state.document
          ? trimHistory([
              ...(currentDraft?.history ?? []),
              snapshotDocument(state.document, state.selectedNodeId),
            ])
          : currentDraft?.history ?? []);
    const future =
      options.future ?? (options.pushHistory === false ? currentDraft?.future ?? [] : []);
    const nextPhaseDrafts = syncCurrentDraft(state, {
      document,
      selectedNodeId,
      history,
      future,
    });

    return {
      document,
      selectedNodeId,
      phaseDrafts: nextPhaseDrafts,
      canUndo: history.length > 0,
      canRedo: future.length > 0,
    };
  });

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
  canUndo: false,
  canRedo: false,

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
        ...getHistoryFlags(draft),
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
      ...getHistoryFlags(draft),
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
    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(
      nextDocument.trees[location.treeIndex].rootNode
    );
    replaceDocumentState(set, get, refreshEditorDocumentNodes(nextDocument));
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
    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(
      nextDocument.trees[location.treeIndex].rootNode
    );
    replaceDocumentState(set, get, refreshEditorDocumentNodes(nextDocument));
  },

  insertNode: (nodeId, position, tagName) => {
    get().insertNodeTemplate(nodeId, position, { tagName });
  },

  insertNodeTemplate: (nodeId, position, template) => {
    const { document } = get();
    if (!document) {
      return;
    }

    const nextDocument = cloneDocument(document);
    const location = findNodeLocation(nextDocument, nodeId);
    if (!location) {
      return;
    }

    const nextNode = createNodeFromTemplate(template);

    if (position === 'wrap') {
      nextNode.children = [location.node];
      if (location.parent) {
        location.parent.children.splice(location.childIndex, 1, nextNode);
      } else {
        nextDocument.trees[location.treeIndex].rootNode = nextNode;
      }
      nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(
        nextDocument.trees[location.treeIndex].rootNode
      );
      replaceDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), {
        selectedNodeId: nextNode.id,
      });
      return;
    }

    if (position === 'prepend_child' || position === 'append_child') {
      if (!canNodeAcceptChildren(location.node)) {
        if (!location.parent) {
          return;
        }
        const fallbackSiblingIndex =
          position === 'prepend_child' ? location.childIndex : location.childIndex + 1;
        location.parent.children.splice(fallbackSiblingIndex, 0, nextNode);
        nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(
          nextDocument.trees[location.treeIndex].rootNode
        );
        replaceDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), {
          selectedNodeId: nextNode.id,
        });
        return;
      }
      const childIndex = position === 'prepend_child' ? 0 : location.node.children.length;
      location.node.children.splice(childIndex, 0, nextNode);
      nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(
        nextDocument.trees[location.treeIndex].rootNode
      );
      replaceDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), {
        selectedNodeId: nextNode.id,
      });
      return;
    }

    if (!location.parent) {
      return;
    }

    const siblingIndex = position === 'before' ? location.childIndex : location.childIndex + 1;
    location.parent.children.splice(siblingIndex, 0, nextNode);
    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(
      nextDocument.trees[location.treeIndex].rootNode
    );
    replaceDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), {
      selectedNodeId: nextNode.id,
    });
  },

  insertNodeOnEdge: (parentNodeId, childNodeId, template) => {
    const { document } = get();
    if (!document) {
      return;
    }

    const nextDocument = cloneDocument(document);
    const childLocation = findNodeLocation(nextDocument, childNodeId);
    if (!childLocation || !childLocation.parent || childLocation.parent.id !== parentNodeId) {
      return;
    }

    const insertedNode = createNodeFromTemplate(template);
    let selectedNodeId = insertedNode.id;

    if (canNodeAcceptChildren(insertedNode)) {
      insertedNode.children = [childLocation.node];
      childLocation.parent.children.splice(childLocation.childIndex, 1, insertedNode);
    } else {
      const sequenceBridge = createNodeFromDefinition('Sequence');
      sequenceBridge.children = [insertedNode, childLocation.node];
      childLocation.parent.children.splice(childLocation.childIndex, 1, sequenceBridge);
    }

    nextDocument.trees[childLocation.treeIndex].rootNode = enrichEditorNode(
      nextDocument.trees[childLocation.treeIndex].rootNode
    );
    replaceDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), {
      selectedNodeId,
    });
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

    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(
      nextDocument.trees[location.treeIndex].rootNode
    );
    replaceDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), {
      selectedNodeId: location.node.id,
    });
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
    nextDocument.trees[location.treeIndex].rootNode = enrichEditorNode(
      nextDocument.trees[location.treeIndex].rootNode
    );
    replaceDocumentState(set, get, refreshEditorDocumentNodes(nextDocument), {
      selectedNodeId: null,
    });
  },

  undo: () => {
    const state = get();
    if (!state.currentPhase || !state.document) {
      return;
    }

    const draft = state.phaseDrafts[state.currentPhase];
    if (!draft || draft.history.length === 0) {
      return;
    }

    const previous = cloneHistoryEntry(draft.history[draft.history.length - 1]);
    const nextHistory = draft.history.slice(0, -1).map(cloneHistoryEntry);
    const nextFuture = trimHistory([
      ...draft.future.map(cloneHistoryEntry),
      snapshotDocument(state.document, state.selectedNodeId),
    ]);

    replaceDocumentState(set, get, refreshEditorDocumentNodes(previous.document), {
      selectedNodeId: previous.selectedNodeId,
      history: nextHistory,
      future: nextFuture,
      pushHistory: false,
    });
  },

  redo: () => {
    const state = get();
    if (!state.currentPhase || !state.document) {
      return;
    }

    const draft = state.phaseDrafts[state.currentPhase];
    if (!draft || draft.future.length === 0) {
      return;
    }

    const next = cloneHistoryEntry(draft.future[draft.future.length - 1]);
    const nextFuture = draft.future.slice(0, -1).map(cloneHistoryEntry);
    const nextHistory = trimHistory([
      ...draft.history.map(cloneHistoryEntry),
      snapshotDocument(state.document, state.selectedNodeId),
    ]);

    replaceDocumentState(set, get, refreshEditorDocumentNodes(next.document), {
      selectedNodeId: next.selectedNodeId,
      history: nextHistory,
      future: nextFuture,
      pushHistory: false,
    });
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
