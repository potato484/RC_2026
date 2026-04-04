import {
  EditorNode,
  EditorNodeKind,
  EditorPortBinding,
  EditorPortBindingMode,
  EditorUiType,
} from '../types/editor';
import { BtNodeRegistryEntry, btNodeRegistry, btNodeRegistryByTag } from '../generated/btNodeRegistry';

export type EditorInsertPosition = 'before' | 'after' | 'prepend_child' | 'append_child' | 'wrap';
export type EditorBranchInsertPosition = 'before' | 'after';

const alongBranchWrapperTags = [
  'Sequence',
  'SequenceWithMemory',
  'ReactiveSequence',
  'Fallback',
  'ReactiveFallback',
  'RoundRobin',
  'Parallel',
  'ParallelAll',
] as const;

const alongBranchWrapperTagSet = new Set<string>(alongBranchWrapperTags);

const fallbackControlNodes = new Set([
  'Sequence',
  'SequenceWithMemory',
  'SequenceStar',
  'ReactiveSequence',
  'Fallback',
  'ReactiveFallback',
  'Parallel',
  'ParallelAll',
  'IfThenElse',
  'WhileDoElse',
  'RoundRobin',
  'Switch2',
  'Switch3',
  'Switch4',
  'Switch5',
  'Switch6',
]);

const fallbackDecoratorNodes = new Set([
  'Inverter',
  'ForceSuccess',
  'ForceFailure',
  'Repeat',
  'RetryUntilSuccessful',
  'KeepRunningUntilFailure',
  'Delay',
  'Timeout',
]);

export function getBtNodeRegistry(): BtNodeRegistryEntry[] {
  return btNodeRegistry;
}

export function getBtNodeDefinition(tagName: string): BtNodeRegistryEntry | undefined {
  return btNodeRegistryByTag[tagName];
}

export function getCompositeSwitchCandidates(tagName: string): BtNodeRegistryEntry[] {
  const current = getBtNodeDefinition(tagName);
  if (!current?.switchGroup) {
    return [];
  }

  return btNodeRegistry.filter((entry) => entry.switchGroup === current.switchGroup && entry.tagName !== tagName);
}

export function getCompositeSwitchGroupEntries(tagName: string): BtNodeRegistryEntry[] {
  const current = getBtNodeDefinition(tagName);
  if (!current?.switchGroup) {
    return [];
  }

  return btNodeRegistry.filter((entry) => entry.switchGroup === current.switchGroup);
}

export function getAlongBranchWrapperEntries(): BtNodeRegistryEntry[] {
  return alongBranchWrapperTags
    .map((tagName) => getBtNodeDefinition(tagName))
    .filter((entry): entry is BtNodeRegistryEntry => Boolean(entry));
}

export function isAlongBranchWrapperTag(tagName: string): boolean {
  return alongBranchWrapperTagSet.has(tagName);
}

export function getNodeUiType(tagName: string, childCount: number): EditorUiType {
  const definition = getBtNodeDefinition(tagName);
  if (definition) {
    return definition.uiType;
  }
  if (tagName === 'SubTree') {
    return 'subtree';
  }
  if (fallbackControlNodes.has(tagName)) {
    return 'control';
  }
  if (fallbackDecoratorNodes.has(tagName)) {
    return 'decorator';
  }
  if (childCount > 1) {
    return 'control';
  }
  if (childCount === 1) {
    return 'decorator';
  }
  return 'leaf';
}

export function getNodeKind(tagName: string, childCount: number): EditorNodeKind {
  const definition = getBtNodeDefinition(tagName);
  if (definition) {
    return definition.category;
  }
  if (tagName === 'SubTree') {
    return 'subtree';
  }
  if (fallbackControlNodes.has(tagName) || childCount > 1) {
    return 'control';
  }
  if (fallbackDecoratorNodes.has(tagName) || childCount === 1) {
    return 'decorator';
  }
  return 'action';
}

export function getNodeSource(tagName: string): EditorNode['source'] {
  return getBtNodeDefinition(tagName)?.source ?? 'unknown';
}

export function getNodeChildPolicy(tagName: string): { min: number; max: number | null } {
  return getBtNodeDefinition(tagName)?.childPolicy ?? { min: 0, max: null };
}

export function canNodeAcceptChildren(nodeOrTagName: EditorNode | string): boolean {
  const tagName = typeof nodeOrTagName === 'string' ? nodeOrTagName : nodeOrTagName.tagName;
  const policy = getNodeChildPolicy(tagName);
  return policy.max === null || policy.max > 0;
}

export function canNodeAddBranch(nodeOrTagName: EditorNode | string, childCount?: number): boolean {
  const tagName = typeof nodeOrTagName === 'string' ? nodeOrTagName : nodeOrTagName.tagName;
  const nextChildCount =
    childCount ?? (typeof nodeOrTagName === 'string' ? 0 : nodeOrTagName.children.length);
  const policy = getNodeChildPolicy(tagName);
  const kind = typeof nodeOrTagName === 'string' ? getNodeKind(tagName, nextChildCount) : nodeOrTagName.nodeKind;

  if (kind !== 'control') {
    return false;
  }

  if (!canNodeAcceptChildren(tagName) || (policy.max !== null && policy.max <= 1)) {
    return false;
  }

  return policy.max === null || nextChildCount < policy.max;
}

export function parsePortBinding(attributeName: string, rawValue: string): EditorPortBinding {
  const trimmed = rawValue.trim();
  let mode: EditorPortBindingMode = 'literal';
  let bindingValue = rawValue;

  const blackboardMatch = trimmed.match(/^\{(.+)\}$/);
  const rootBlackboardMatch = trimmed.match(/^@([A-Za-z_][A-Za-z0-9_]*)$/);

  if (blackboardMatch) {
    mode = 'blackboard';
    bindingValue = blackboardMatch[1];
  } else if (rootBlackboardMatch) {
    mode = 'root_blackboard';
    bindingValue = rootBlackboardMatch[1];
  }

  return {
    attributeName,
    rawValue,
    mode,
    bindingValue,
  };
}

export function formatPortBindingValue(mode: EditorPortBindingMode, value: string): string {
  if (!value.trim()) {
    return '';
  }
  if (mode === 'blackboard') {
    return `{${value.trim()}}`;
  }
  if (mode === 'root_blackboard') {
    return `@${value.trim()}`;
  }
  return value;
}

export function buildNodePortBindings(
  attributes: Record<string, string>,
  definition?: BtNodeRegistryEntry
): Record<string, EditorPortBinding> {
  const orderedKeys = new Set<string>([
    ...(definition?.portSchemas.map((port) => port.name) ?? []),
    ...Object.keys(attributes),
  ]);

  return Array.from(orderedKeys).reduce<Record<string, EditorPortBinding>>((acc, key) => {
    acc[key] = parsePortBinding(key, attributes[key] ?? '');
    return acc;
  }, {});
}

export function enrichEditorNode(node: EditorNode): EditorNode {
  const definition = getBtNodeDefinition(node.tagName);
  return {
    ...node,
    definitionId: definition?.id ?? node.tagName,
    nodeKind: definition?.category ?? getNodeKind(node.tagName, node.children.length),
    source: definition?.source ?? getNodeSource(node.tagName),
    uiType: definition?.uiType ?? getNodeUiType(node.tagName, node.children.length),
    portBindings: buildNodePortBindings(node.attributes, definition),
    children: node.children.map((child) => enrichEditorNode(child)),
  };
}

export function createNodeFromDefinition(tagName: string): EditorNode {
  const definition = getBtNodeDefinition(tagName);
  const attributes = { ...(definition?.defaultAttributes ?? {}) };
  const node: EditorNode = {
    id: `node_${Math.random().toString(36).slice(2, 11)}`,
    tagName,
    definitionId: definition?.id ?? tagName,
    nodeKind: definition?.category ?? getNodeKind(tagName, 0),
    source: definition?.source ?? 'unknown',
    attributes,
    portBindings: buildNodePortBindings(attributes, definition),
    children: [],
    uiType: definition?.uiType ?? getNodeUiType(tagName, 0),
  };
  return enrichEditorNode(node);
}

export function refreshEditorDocumentNodes<T extends { trees: Array<{ rootNode: EditorNode }> }>(document: T): T {
  document.trees.forEach((tree) => {
    tree.rootNode = enrichEditorNode(tree.rootNode);
  });
  return document;
}
