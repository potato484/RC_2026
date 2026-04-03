import { btNodeRegistryByTag } from '../generated/btNodeRegistry';
import { attributeKeyZhMap, blackboardKeyZhMap, enumValueZhMap, instanceNameZhMap, treeIdZhMap } from '../i18n/btTerms';

export interface BehaviorTreeNodeDisplay {
  label: string;
  desc: string;
}

export interface BehaviorTreeAttributeDisplay {
  rawKey: string;
  rawValue: string;
  label: string;
  value: string;
  summary: string;
}

const containsEnglishLetters = (text: string): boolean => /[A-Za-z]/.test(text);

const isNumericText = (text: string): boolean => /^-?\d+(\.\d+)?$/.test(text);

const normalizeAttributes = (attributes: Record<string, string>): Record<string, string> =>
  Object.entries(attributes).reduce<Record<string, string>>((accumulator, [key, value]) => {
    accumulator[key.toLowerCase()] = value;
    return accumulator;
  }, {});

function translateScriptCode(code: string): string {
  if (!code) {
    return '未配置脚本';
  }

  const trimmed = code.trim();
  if (trimmed.includes("next_action=='GRAB'")) return '判断下一步是否抓取';
  if (trimmed.includes("next_action=='MOVE'")) return '判断下一步是否移动';
  if (trimmed.includes("next_action=='WAIT'")) return '判断下一步是否等待';
  if (trimmed.includes('target_kfs_count:=2')) return '初始化目标 KFS 数量为 2';
  if (trimmed.includes('kfs_on_board:=0')) return '初始化已装载 KFS 数量为 0';
  if (trimmed.includes('current_grid:=')) {
    const value = trimmed.split('current_grid:=')[1]?.split(/[; ]/)[0] ?? '';
    return value ? `写入当前格子为 ${value}` : '更新当前格子';
  }
  return trimmed.replace(/:=/g, ' 设为 ').replace(/==/g, ' 等于 ');
}

export function translateParam(param: string): string {
  if (!param) {
    return param;
  }

  const trimmed = param.trim();
  if (trimmed.startsWith('{') && trimmed.endsWith('}')) {
    const key = trimmed.slice(1, -1);
    return `黑板：${translateBlackboardKey(key)}`;
  }
  if (trimmed.startsWith('@')) {
    return `根黑板：${translateBlackboardKey(trimmed.slice(1))}`;
  }
  if (enumValueZhMap[trimmed]) {
    return enumValueZhMap[trimmed];
  }
  if (trimmed.includes(':=') || trimmed.includes('==')) {
    return translateScriptCode(trimmed);
  }
  if (treeIdZhMap[trimmed]) {
    return treeIdZhMap[trimmed];
  }
  if (instanceNameZhMap[trimmed]) {
    return instanceNameZhMap[trimmed];
  }
  return trimmed;
}

export function translateName(name: string, fallbackLabel: string): string {
  if (!name) {
    return fallbackLabel || '未命名节点';
  }

  if (instanceNameZhMap[name]) {
    return instanceNameZhMap[name];
  }
  if (treeIdZhMap[name]) {
    return treeIdZhMap[name];
  }

  let translated = name
    .replace(/ReactiveSequence/g, '响应式顺序流程')
    .replace(/ReactiveFallback/g, '响应式回退流程')
    .replace(/SequenceWithMemory/g, '记忆顺序流程')
    .replace(/SequenceStar/g, '记忆顺序流程')
    .replace(/Sequence/g, '顺序流程')
    .replace(/Fallback/g, '回退流程')
    .replace(/ParallelAll/g, '全并行流程')
    .replace(/Parallel/g, '并行流程')
    .replace(/Entry/g, '进门')
    .replace(/Exit/g, '离场')
    .replace(/Loop/g, '循环')
    .replace(/Main/g, '主流程')
    .replace(/Combat/g, '对抗')
    .replace(/Merlin/g, '梅林')
    .replace(/Grid/g, '格子')
    .replace(/Move/g, '移动')
    .replace(/Grab/g, '抓取')
    .replace(/Assemble/g, '组装');

  if (containsEnglishLetters(translated)) {
    return fallbackLabel || '已命名节点';
  }

  translated = translated
    .replace(/_/g, '')
    .replace(/\s+/g, ' ')
    .trim();

  return translated || fallbackLabel || '已命名节点';
}

export function getBehaviorTreeTreeName(treeId: string, treeName?: string): string {
  const rawName = treeName || treeId;
  return translateName(rawName, treeIdZhMap[treeId] || '未命名决策树');
}

export function translateBlackboardKey(key: string): string {
  if (!key) {
    return '未命名黑板键';
  }
  return blackboardKeyZhMap[key] || translateName(key, '黑板键');
}

function translateBoolean(rawValue: string): string {
  const normalized = rawValue.trim().toLowerCase();
  if (normalized === 'true' || normalized === '1' || normalized === 'yes') {
    return '是';
  }
  if (normalized === 'false' || normalized === '0' || normalized === 'no') {
    return '否';
  }
  return rawValue;
}

function translateAttributeValue(rawValue: string): string {
  if (!rawValue) {
    return '空值';
  }
  if (rawValue.startsWith('{') && rawValue.endsWith('}')) {
    return `黑板：${translateBlackboardKey(rawValue.slice(1, -1))}`;
  }
  if (rawValue.startsWith('@')) {
    return `根黑板：${translateBlackboardKey(rawValue.slice(1))}`;
  }
  if (enumValueZhMap[rawValue]) {
    return enumValueZhMap[rawValue];
  }
  if (rawValue.includes(':=') || rawValue.includes('==')) {
    return translateScriptCode(rawValue);
  }
  if (treeIdZhMap[rawValue]) {
    return treeIdZhMap[rawValue];
  }
  if (instanceNameZhMap[rawValue]) {
    return instanceNameZhMap[rawValue];
  }
  if (rawValue === 'true' || rawValue === 'false' || rawValue === '1' || rawValue === '0') {
    return translateBoolean(rawValue);
  }
  if (!containsEnglishLetters(rawValue) || isNumericText(rawValue)) {
    return rawValue;
  }
  return translateParam(rawValue) || '已配置值';
}

function getFallbackLabel(tagName: string, normalizedAttributes: Record<string, string>): string {
  if (tagName === 'SubTree') {
    const treeId = normalizedAttributes.id || normalizedAttributes.name || '';
    return translateName(treeId, treeIdZhMap[treeId] || '子树调用节点');
  }

  if (tagName === 'Script') {
    return '脚本赋值节点';
  }
  if (tagName === 'ScriptCondition') {
    return '脚本条件节点';
  }

  return btNodeRegistryByTag[tagName]?.labelZh || '未映射节点';
}

function getDefaultDescription(tagName: string): string {
  return btNodeRegistryByTag[tagName]?.descriptionZh || '当前节点尚未补充说明。';
}

export function getBehaviorTreeNodeCategoryLabel(nodeKind: string): string {
  if (nodeKind === 'control' || nodeKind === 'sequence' || nodeKind === 'selector') return '控制节点';
  if (nodeKind === 'decorator') return '装饰节点';
  if (nodeKind === 'condition') return '条件节点';
  if (nodeKind === 'subtree') return '子树节点';
  return '动作节点';
}

export function getBehaviorTreeAttributeDisplay(key: string, rawValue: string): BehaviorTreeAttributeDisplay {
  const label = attributeKeyZhMap[key] || attributeKeyZhMap[key.toLowerCase()] || '自定义属性';
  const value =
    key === '_autoremap'
      ? translateBoolean(rawValue)
      : key === 'name' || key === 'ID' || key === 'id'
        ? translateName(rawValue, rawValue)
        : translateAttributeValue(rawValue);

  return {
    rawKey: key,
    rawValue,
    label,
    value,
    summary: `${label}：${value}`,
  };
}

export function getBehaviorTreeAttributeDisplays(
  attributes: Record<string, string>,
  tagName?: string
): BehaviorTreeAttributeDisplay[] {
  const normalizedTag = tagName ? btNodeRegistryByTag[tagName] : undefined;
  const orderedKeys = [
    ...(normalizedTag?.portSchemas.map((port) => port.name) ?? []),
    ...Object.keys(attributes).filter(
      (key) => !(normalizedTag?.portSchemas.some((port) => port.name === key))
    ),
  ];

  return orderedKeys
    .filter((key, index) => orderedKeys.indexOf(key) === index)
    .filter((key) => attributes[key] !== undefined)
    .map((key) => getBehaviorTreeAttributeDisplay(key, attributes[key]));
}

export function summarizeBehaviorTreeAttributes(
  attributes: Record<string, string>,
  tagName?: string,
  limit = 2
): string[] {
  return getBehaviorTreeAttributeDisplays(attributes, tagName)
    .map((attribute) => attribute.summary)
    .slice(0, limit);
}

export function getBehaviorTreeNodeDisplay(
  tagName: string,
  attributes: Record<string, string>
): BehaviorTreeNodeDisplay {
  const normalizedAttributes = normalizeAttributes(attributes);
  const nameAttr = normalizedAttributes.name || normalizedAttributes.id || '';
  const fallbackLabel = getFallbackLabel(tagName, normalizedAttributes);

  let label = translateName(nameAttr, fallbackLabel);
  if (!nameAttr || containsEnglishLetters(label)) {
    label = fallbackLabel;
  }

  if (tagName === 'Script') {
    label = translateScriptCode(attributes.code ?? normalizedAttributes.code ?? '');
  } else if (tagName === 'ScriptCondition') {
    label = `脚本条件：${translateScriptCode(attributes.code ?? normalizedAttributes.code ?? '')}`;
  }

  const details = summarizeBehaviorTreeAttributes(attributes, tagName);
  const descBase = getDefaultDescription(tagName);
  const desc = details.length > 0 ? `${descBase} [${details.join('，')}]` : descBase;

  return {
    label: details.length > 0 && tagName !== 'Script' && tagName !== 'ScriptCondition'
      ? `${label}（${details.join('，')}）`
      : label,
    desc,
  };
}
