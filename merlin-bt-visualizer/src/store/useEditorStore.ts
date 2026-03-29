import { create } from 'zustand';
import { EditorDocument, EditorNode } from '../types/editor';
import { xmlToEditorDocument } from '../utils/editorParser';
import { editorDocumentToXml } from '../utils/editorSerializer';
import { Node as FlowNode, Edge as FlowEdge } from '@xyflow/react';
import { projectTreeToFlow } from '../utils/editorProjection';

interface EditorState {
  // Data Model
  document: EditorDocument | null;
  activeTreeId: string | null;
  collapsedNodes: Set<string>;
  
  // React Flow State
  flowNodes: FlowNode[];
  flowEdges: FlowEdge[];
  selectedNodeId: string | null;
  
  // Actions
  loadXml: (xmlContent: string) => void;
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

export const useEditorStore = create<EditorState>((set, get) => ({
  document: null,
  activeTreeId: null,
  collapsedNodes: new Set(),
  flowNodes: [],
  flowEdges: [],
  selectedNodeId: null,

  loadXml: (xmlContent: string) => {
    try {
      const doc = xmlToEditorDocument(xmlContent);
      const firstTreeId = doc.trees.length > 0 ? doc.trees[0].id : null;
      
      set({ 
        document: doc, 
        activeTreeId: firstTreeId,
        collapsedNodes: new Set(),
        selectedNodeId: null
      });
      
      get()._updateFlow();
    } catch (error) {
      console.error("Failed to parse XML:", error);
    }
  },

  setActiveTree: (treeId: string) => {
    set({ activeTreeId: treeId, selectedNodeId: null });
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
    set({ collapsedNodes: newCollapsed });
    get()._updateFlow();
  },

  setSelectedNode: (nodeId: string | null) => {
    set({ selectedNodeId: nodeId });
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
      set({ document: newDoc });
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
      set({ document: newDoc });
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
      set({ document: newDoc, selectedNodeId: null });
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
