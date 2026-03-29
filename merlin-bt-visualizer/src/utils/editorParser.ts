import { EditorDocument, EditorTree, EditorNode } from '../types/editor';

/**
 * Parses a raw XML string into an EditorDocument model.
 * It strictly preserves structural and semantic details for round-tripping.
 */
export function xmlToEditorDocument(xmlString: string): EditorDocument {
  const parser = new DOMParser();
  const xmlDoc = parser.parseFromString(xmlString, 'text/xml');

  // Check for parsing errors
  const parseError = xmlDoc.getElementsByTagName('parsererror');
  if (parseError.length > 0) {
    throw new Error('XML parsing failed: ' + parseError[0].textContent);
  }

  const rootElement = xmlDoc.documentElement;
  if (rootElement.tagName !== 'root') {
    throw new Error('Invalid BehaviorTree XML: Root element must be <root>');
  }

  const rootAttributes: Record<string, string> = {};
  for (let i = 0; i < rootElement.attributes.length; i++) {
    const attr = rootElement.attributes[i];
    rootAttributes[attr.name] = attr.value;
  }

  const includes: string[] = [];
  const trees: EditorTree[] = [];

  // Parse direct children of <root>
  for (let i = 0; i < rootElement.children.length; i++) {
    const child = rootElement.children[i];

    if (child.tagName === 'include') {
      const path = child.getAttribute('path');
      if (path) {
        includes.push(path);
      }
    } else if (child.tagName === 'BehaviorTree') {
      trees.push(parseEditorTree(child));
    }
  }

  return {
    rootAttributes,
    includes,
    trees
  };
}

function parseEditorTree(treeElement: Element): EditorTree {
  const id = treeElement.getAttribute('ID') || 'UnknownTree';
  const name = treeElement.getAttribute('name') || undefined;

  // Find the first actual element child as the root node
  let rootNodeElement = null;
  for (let i = 0; i < treeElement.children.length; i++) {
    rootNodeElement = treeElement.children[i];
    break; // BehaviorTree usually has only one root node child
  }

  if (!rootNodeElement) {
    throw new Error(`BehaviorTree ${id} has no root node.`);
  }

  return {
    id,
    name,
    rootNode: parseEditorNode(rootNodeElement)
  };
}

function parseEditorNode(nodeElement: Element): EditorNode {
  const tagName = nodeElement.tagName;
  const attributes: Record<string, string> = {};
  
  for (let i = 0; i < nodeElement.attributes.length; i++) {
    const attr = nodeElement.attributes[i];
    attributes[attr.name] = attr.value;
  }

  const children: EditorNode[] = [];
  for (let i = 0; i < nodeElement.children.length; i++) {
    children.push(parseEditorNode(nodeElement.children[i]));
  }

  return {
    id: `node_${Math.random().toString(36).substr(2, 9)}`,
    tagName,
    attributes,
    children,
    uiType: determineUiType(tagName, children.length)
  };
}

const CONTROL_NODES = new Set([
  'Sequence', 'ReactiveSequence', 'SequenceStar', 
  'Fallback', 'ReactiveFallback', 'Parallel', 'IfThenElse', 'Switch2', 'Switch3', 'Switch4', 'Switch5', 'Switch6'
]);

const DECORATOR_NODES = new Set([
  'Inverter', 'ForceSuccess', 'ForceFailure', 
  'Repeat', 'RetryUntilSuccessful', 'KeepRunningUntilFailure',
  'Timeout', 'Delay'
]);

function determineUiType(tagName: string, childCount: number): 'control' | 'decorator' | 'leaf' | 'subtree' {
  if (tagName === 'SubTree') {
    return 'subtree';
  }
  if (CONTROL_NODES.has(tagName)) {
    return 'control';
  }
  if (DECORATOR_NODES.has(tagName)) {
    return 'decorator';
  }
  
  // Basic heuristic if tag name is not in predefined lists
  if (childCount > 0) {
    if (childCount === 1) {
      return 'decorator';
    }
    return 'control';
  }
  
  return 'leaf';
}
