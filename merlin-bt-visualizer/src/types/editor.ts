export type EditorUiType = 'control' | 'decorator' | 'leaf' | 'subtree';

export type EditorNodeKind = 'control' | 'decorator' | 'action' | 'condition' | 'subtree';

export type EditorNodeSource = 'official' | 'robot' | 'unknown';

export type EditorPortBindingMode = 'literal' | 'blackboard' | 'root_blackboard';

export interface EditorPortBinding {
  attributeName: string;
  rawValue: string;
  mode: EditorPortBindingMode;
  bindingValue: string;
}

export interface EditorNode {
  id: string;
  tagName: string;
  definitionId: string;
  nodeKind: EditorNodeKind;
  source: EditorNodeSource;
  attributes: Record<string, string>;
  portBindings: Record<string, EditorPortBinding>;
  children: EditorNode[];
  uiType: EditorUiType;
}

export interface EditorTree {
  id: string;
  name?: string;
  rootNode: EditorNode;
}

export interface EditorDocument {
  rootAttributes: Record<string, string>;
  includes: string[];
  trees: EditorTree[];
}
