import { EditorDocument, EditorTree, EditorNode } from '../types/editor';
import { enrichEditorNode, refreshEditorDocumentNodes } from './btRegistry';

export function xmlToEditorDocument(xmlString: string): EditorDocument {
  const parser = new DOMParser();
  const xmlDoc = parser.parseFromString(xmlString, 'text/xml');

  const parseError = xmlDoc.getElementsByTagName('parsererror');
  if (parseError.length > 0) {
    throw new Error(`XML 解析失败：${parseError[0].textContent}`);
  }

  const rootElement = xmlDoc.documentElement;
  if (rootElement.tagName !== 'root') {
    throw new Error('非法的行为树 XML：根节点必须是 <root>');
  }

  const rootAttributes: Record<string, string> = {};
  for (let index = 0; index < rootElement.attributes.length; index += 1) {
    const attribute = rootElement.attributes[index];
    rootAttributes[attribute.name] = attribute.value;
  }

  const includes: string[] = [];
  const trees: EditorTree[] = [];

  for (let index = 0; index < rootElement.children.length; index += 1) {
    const child = rootElement.children[index];
    if (child.tagName === 'include') {
      const includePath = child.getAttribute('path');
      if (includePath) {
        includes.push(includePath);
      }
      continue;
    }

    if (child.tagName === 'BehaviorTree') {
      trees.push(parseEditorTree(child));
    }
  }

  return refreshEditorDocumentNodes({
    rootAttributes,
    includes,
    trees,
  });
}

function parseEditorTree(treeElement: Element): EditorTree {
  const id = treeElement.getAttribute('ID') || 'UnknownTree';
  const name = treeElement.getAttribute('name') || undefined;

  const rootNodeElement = Array.from(treeElement.children).find((child) => child.nodeType === Node.ELEMENT_NODE) ?? null;
  if (!rootNodeElement) {
    throw new Error(`行为树 ${id} 没有根节点。`);
  }

  return {
    id,
    name,
    rootNode: parseEditorNode(rootNodeElement),
  };
}

function parseEditorNode(nodeElement: Element): EditorNode {
  const attributes: Record<string, string> = {};
  for (let index = 0; index < nodeElement.attributes.length; index += 1) {
    const attribute = nodeElement.attributes[index];
    attributes[attribute.name] = attribute.value;
  }

  const children = Array.from(nodeElement.children).map((child) => parseEditorNode(child));

  const node: EditorNode = {
    id: `node_${Math.random().toString(36).slice(2, 11)}`,
    tagName: nodeElement.tagName,
    definitionId: nodeElement.tagName,
    nodeKind: 'action',
    source: 'unknown',
    attributes,
    portBindings: {},
    children,
    uiType: 'leaf',
  };

  return enrichEditorNode(node);
}
