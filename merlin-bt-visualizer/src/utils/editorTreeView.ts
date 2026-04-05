import { EditorDocument, EditorNode } from '../types/editor';
import { getBehaviorTreeNodeDisplay, getBehaviorTreeTreeName } from './btDisplay';

export interface EditorTreeListItem {
  id: string;
  name: string;
  parentTreeId?: string;
}

function collectSubTreeIds(node: EditorNode, visitor: (treeId: string) => void) {
  if (node.tagName === 'SubTree') {
    const subTreeId = node.attributes.ID;
    if (subTreeId) visitor(subTreeId);
  }

  node.children.forEach((child) => collectSubTreeIds(child, visitor));
}

export function buildEditorTreeList(document: EditorDocument): EditorTreeListItem[] {
  const parentTreeMap = new Map<string, string>();

  document.trees.forEach((tree) => {
    collectSubTreeIds(tree.rootNode, (subTreeId) => {
      parentTreeMap.set(subTreeId, tree.id);
    });
  });

  return document.trees.map((tree) => ({
    id: tree.id,
    name: getBehaviorTreeTreeName(tree.id, tree.name),
    parentTreeId: parentTreeMap.get(tree.id),
  }));
}

export function buildEditorTreePreview(document: EditorDocument, activeTreeId: string | null): string {
  const activeTree = activeTreeId ? document.trees.find((tree) => tree.id === activeTreeId) : null;
  if (!activeTree) return '当前没有可预览的结构';

  const lines = [`当前决策树：${getBehaviorTreeTreeName(activeTree.id, activeTree.name)}`, ''];

  const appendNodeLine = (node: EditorNode, depth: number) => {
    const display = getBehaviorTreeNodeDisplay(node.tagName, node.attributes);
    lines.push(`${'  '.repeat(depth)}- ${display.label}`);
    node.children.forEach((child) => appendNodeLine(child, depth + 1));
  };

  appendNodeLine(activeTree.rootNode, 0);
  return lines.join('\n');
}
