export interface EditorNode {
  id: string;
  tagName: string;
  attributes: Record<string, string>;
  children: EditorNode[];
  uiType: 'control' | 'decorator' | 'leaf' | 'subtree';
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
