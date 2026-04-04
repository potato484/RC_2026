import { BtNodeRegistryEntry, BtPortSchema } from '../generated/btNodeRegistry';
import { EditorDocument, EditorInsertTemplate, EditorNodeKind, EditorNodeSource } from '../types/editor';
import { getBehaviorTreeNodeCategoryLabel, getBehaviorTreeTreeName } from './btDisplay';
import { getBtNodeRegistry } from './btRegistry';
import { behaviorTreeXmlByPhase, BehaviorTreePhase } from './behaviorTreeSources';

export interface EditorInsertCatalogItem {
  id: string;
  category: 'common' | 'action' | 'condition' | 'control' | 'subtree';
  label: string;
  description: string;
  tagName: string;
  tagLabel: string;
  groupLabel: string;
  sourceLabel: string;
  template: EditorInsertTemplate;
  searchTokens: string[];
}

export interface EditorInsertCatalogSection {
  id: string;
  title: string;
  items: EditorInsertCatalogItem[];
}

export type EditorInsertCatalogCategory = EditorInsertCatalogItem['category'];

export type EditorKnowledgeBaseCategoryId =
  | 'merlin'
  | 'navigation'
  | 'vision'
  | 'duel'
  | 'martial'
  | 'official'
  | 'subtree';

export interface EditorKnowledgeBaseItem {
  id: string;
  knowledgeBaseCategoryId: EditorKnowledgeBaseCategoryId;
  category: EditorNodeKind;
  label: string;
  description: string;
  tagName: string;
  tagLabel: string;
  groupLabel: string;
  source: EditorNodeSource;
  sourceLabel: string;
  childPolicy: {
    min: number;
    max: number | null;
  };
  portSchemas: BtPortSchema[];
  keywordsZh: string[];
  keywordsEn: string[];
  searchTokens: string[];
}

export interface EditorKnowledgeBaseCategory {
  id: EditorKnowledgeBaseCategoryId;
  title: string;
  description: string;
  items: EditorKnowledgeBaseItem[];
}

interface EditorInsertCatalogOptions {
  includeCategories?: EditorInsertCatalogCategory[];
}

const registryEntries = getBtNodeRegistry();
const subtreeDefinition = registryEntries.find((entry) => entry.tagName === 'SubTree');

const knowledgeBaseCategoryMeta: Record<
  EditorKnowledgeBaseCategoryId,
  { title: string; description: string }
> = {
  merlin: {
    title: '梅林区模块',
    description: '梅林区的抓取、地图状态刷新、退出判断和机构动作。',
  },
  navigation: {
    title: '导航模块',
    description: '拓扑导航、任务位姿跳转和整段路线执行。',
  },
  vision: {
    title: '视觉模块',
    description: '视觉启动、切模、等待目标和识别资源控制。',
  },
  duel: {
    title: '对抗区模块',
    description: '对抗区机构动作、跟随手动机器人和放置行为。',
  },
  martial: {
    title: '武馆区模块',
    description: '武馆区矛头抓取、组装和手动机器人配合判断。',
  },
  official: {
    title: '官方节点',
    description: 'BehaviorTree.CPP 提供的控制、装饰、脚本和结构节点。',
  },
  subtree: {
    title: '当前文档子树',
    description: '当前 XML 文档里可直接引用的子树入口。',
  },
};

const categoryLabels: Record<EditorInsertCatalogItem['category'], string> = {
  common: '常用',
  action: '动作',
  condition: '条件',
  control: '控制',
  subtree: '子树',
};

function getSourceLabel(entry: BtNodeRegistryEntry): string {
  return entry.source === 'robot' ? '机器人模块' : '官方节点';
}

function buildRegistrySearchTokens(entry: BtNodeRegistryEntry): string[] {
  return [
    entry.labelZh,
    entry.descriptionZh,
    entry.group,
    entry.tagName,
    getSourceLabel(entry),
    getBehaviorTreeNodeCategoryLabel(entry.category),
    ...entry.keywordsZh,
    ...entry.keywordsEn,
    ...entry.portSchemas.flatMap((port) => [
      port.name,
      port.labelZh,
      port.descriptionZh,
      port.valueType,
      port.defaultValue ?? '',
    ]),
  ];
}

function buildRegistryItem(
  entry: BtNodeRegistryEntry,
  category: EditorInsertCatalogItem['category']
): EditorInsertCatalogItem {
  return {
    id: `${category}:${entry.tagName}`,
    category,
    label: entry.labelZh,
    description: entry.descriptionZh,
    tagName: entry.tagName,
    tagLabel: entry.tagName,
    groupLabel: entry.group,
    sourceLabel: getSourceLabel(entry),
    template: { tagName: entry.tagName },
    searchTokens: buildRegistrySearchTokens(entry),
  };
}

function resolveKnowledgeBaseCategoryId(
  entry: Pick<BtNodeRegistryEntry, 'source' | 'group' | 'tagName'>
): EditorKnowledgeBaseCategoryId {
  if (entry.tagName === 'SubTree') {
    return 'subtree';
  }

  if (entry.source === 'official') {
    return 'official';
  }

  if (entry.group.includes('梅林区')) {
    return 'merlin';
  }
  if (entry.group.includes('导航')) {
    return 'navigation';
  }
  if (entry.group.includes('视觉')) {
    return 'vision';
  }
  if (entry.group.includes('对抗区')) {
    return 'duel';
  }
  if (entry.group.includes('武馆区')) {
    return 'martial';
  }

  return 'official';
}

function buildRegistryKnowledgeItem(
  entry: BtNodeRegistryEntry,
  knowledgeBaseCategoryId = resolveKnowledgeBaseCategoryId(entry)
): EditorKnowledgeBaseItem {
  return {
    id: `${knowledgeBaseCategoryId}:${entry.tagName}`,
    knowledgeBaseCategoryId,
    category: entry.category,
    label: entry.labelZh,
    description: entry.descriptionZh,
    tagName: entry.tagName,
    tagLabel: entry.tagName,
    groupLabel: entry.group,
    source: entry.source,
    sourceLabel: getSourceLabel(entry),
    childPolicy: entry.childPolicy,
    portSchemas: entry.portSchemas,
    keywordsZh: entry.keywordsZh,
    keywordsEn: entry.keywordsEn,
    searchTokens: buildRegistrySearchTokens(entry),
  };
}

function countTagUsage(): Map<string, number> {
  const counts = new Map<string, number>();
  Object.values(behaviorTreeXmlByPhase).forEach((xmlContent) => {
    const matches = xmlContent.match(/<([A-Za-z_][A-Za-z0-9_]*)\b/g) ?? [];
    matches.forEach((match) => {
      const tagName = match.slice(1);
      if (['root', 'BehaviorTree', 'TreeNodesModel', 'Action', 'Condition', 'Decorator', 'Control', 'include'].includes(tagName)) {
        return;
      }

      counts.set(tagName, (counts.get(tagName) ?? 0) + 1);
    });
  });

  return counts;
}

const tagUsageCounts = countTagUsage();

function getCommonRobotEntries(limit = 8): BtNodeRegistryEntry[] {
  return registryEntries
    .filter((entry) => entry.source === 'robot')
    .sort((left, right) => {
      const countDiff = (tagUsageCounts.get(right.tagName) ?? 0) - (tagUsageCounts.get(left.tagName) ?? 0);
      if (countDiff !== 0) {
        return countDiff;
      }

      return left.labelZh.localeCompare(right.labelZh, 'zh-CN');
    })
    .slice(0, limit);
}

export function getCommonRobotInsertItems(limit = 8): EditorInsertCatalogItem[] {
  return getCommonRobotEntries(limit).map((entry) => buildRegistryItem(entry, 'common'));
}

function filterItems<T extends { searchTokens: string[] }>(items: T[], query: string): T[] {
  const keyword = query.trim().toLowerCase();
  if (!keyword) {
    return items;
  }

  return items.filter((item) =>
    item.searchTokens.some((token) => token.toLowerCase().includes(keyword))
  );
}

function buildSubtreeItems(
  document: EditorDocument | null | undefined,
  activeTreeId: string | null | undefined
): EditorInsertCatalogItem[] {
  if (!document) {
    return [
      {
        id: 'subtree:generic',
        category: 'subtree',
        label: '子树调用节点',
        description: '跳到另一棵行为树继续执行，可自动继承同名端口。',
        tagName: 'SubTree',
        tagLabel: 'SubTree',
        groupLabel: '官方结构节点',
        sourceLabel: '官方节点',
        template: {
          tagName: 'SubTree',
          presetAttributes: { ID: '', _autoremap: 'true' },
        },
        searchTokens: ['子树', 'SubTree', '自动映射'],
      },
    ];
  }

  const treeItems = document.trees
    .filter((tree) => tree.id !== activeTreeId)
    .map<EditorInsertCatalogItem>((tree) => {
      const treeLabel = getBehaviorTreeTreeName(tree.id, tree.name);
      return {
        id: `subtree:${tree.id}`,
        category: 'subtree',
        label: treeLabel,
        description: `切到“${treeLabel}”继续执行，并默认开启自动映射。`,
        tagName: 'SubTree',
        tagLabel: tree.id,
        groupLabel: '当前文档子树',
        sourceLabel: '官方节点',
        template: {
          tagName: 'SubTree',
          presetAttributes: { ID: tree.id, _autoremap: 'true' },
        },
        searchTokens: [treeLabel, tree.id, '子树', '自动映射'],
      };
    });

  return treeItems.length > 0
      ? treeItems
    : [
        {
          id: 'subtree:generic',
          category: 'subtree',
          label: '子树调用节点',
          description: '跳到另一棵行为树继续执行，可自动继承同名端口。',
          tagName: 'SubTree',
          tagLabel: 'SubTree',
          groupLabel: '官方结构节点',
          sourceLabel: '官方节点',
          template: {
            tagName: 'SubTree',
            presetAttributes: { ID: '', _autoremap: 'true' },
          },
          searchTokens: ['子树', 'SubTree', '自动映射'],
        },
      ];
}

function buildSubtreeKnowledgeItems(
  document: EditorDocument | null | undefined,
  activeTreeId: string | null | undefined
): EditorKnowledgeBaseItem[] {
  if (!subtreeDefinition) {
    return [];
  }

  if (!document) {
    return [
      {
        ...buildRegistryKnowledgeItem(subtreeDefinition, 'subtree'),
        id: 'subtree:generic',
        description: '跳到另一棵行为树继续执行，默认自动继承同名端口。',
        groupLabel: '官方结构节点',
        searchTokens: [...buildRegistrySearchTokens(subtreeDefinition), '自动映射'],
      },
    ];
  }

  const treeItems = document.trees
    .filter((tree) => tree.id !== activeTreeId)
    .map<EditorKnowledgeBaseItem>((tree) => {
      const treeLabel = getBehaviorTreeTreeName(tree.id, tree.name);
      return {
        ...buildRegistryKnowledgeItem(subtreeDefinition, 'subtree'),
        id: `subtree:${tree.id}`,
        label: treeLabel,
        description: `跳到“${treeLabel}”继续执行，并默认开启自动映射。`,
        tagLabel: tree.id,
        groupLabel: '当前文档子树',
        searchTokens: [...buildRegistrySearchTokens(subtreeDefinition), treeLabel, tree.id, '自动映射'],
      };
    });

  return treeItems.length > 0
    ? treeItems
      : [
        {
          ...buildRegistryKnowledgeItem(subtreeDefinition, 'subtree'),
          id: 'subtree:generic',
          description: '跳到另一棵行为树继续执行，默认自动继承同名端口。',
          groupLabel: '官方结构节点',
          searchTokens: [...buildRegistrySearchTokens(subtreeDefinition), '自动映射'],
        },
      ];
}

export function buildEditorInsertCatalog(
  document: EditorDocument | null | undefined,
  activeTreeId: string | null | undefined,
  query = '',
  options: EditorInsertCatalogOptions = {}
): EditorInsertCatalogSection[] {
  const registryByCategory = {
    action: registryEntries
      .filter((entry) => entry.category === 'action' && entry.tagName !== 'SubTree')
      .map((entry) => buildRegistryItem(entry, 'action')),
    condition: registryEntries
      .filter((entry) => entry.category === 'condition')
      .map((entry) => buildRegistryItem(entry, 'condition')),
    control: registryEntries
      .filter((entry) => entry.category === 'control' || entry.category === 'decorator')
      .map((entry) => buildRegistryItem(entry, 'control')),
    subtree: buildSubtreeItems(document, activeTreeId),
  };

  const sections: EditorInsertCatalogSection[] = [
    {
      id: 'common',
      title: '常用机器人模块',
      items: filterItems(getCommonRobotInsertItems(), query),
    },
    {
      id: 'action',
      title: '动作节点',
      items: filterItems(registryByCategory.action, query),
    },
    {
      id: 'condition',
      title: '条件节点',
      items: filterItems(registryByCategory.condition, query),
    },
    {
      id: 'control',
      title: '控制节点',
      items: filterItems(registryByCategory.control, query),
    },
    {
      id: 'subtree',
      title: '子树节点',
      items: filterItems(registryByCategory.subtree, query),
    },
  ];

  const allowedCategories = options.includeCategories ? new Set(options.includeCategories) : null;

  return sections
    .filter((section) => !allowedCategories || allowedCategories.has(section.id as EditorInsertCatalogCategory))
    .map((section) => ({
      ...section,
      items:
        section.id === 'common'
          ? section.items
          : section.items.sort((left, right) => left.label.localeCompare(right.label, 'zh-CN')),
    }))
    .filter((section) => section.items.length > 0);
}

export function buildAlongBranchInsertCatalog(
  document: EditorDocument | null | undefined,
  activeTreeId: string | null | undefined,
  query = ''
): EditorInsertCatalogSection[] {
  return buildEditorInsertCatalog(document, activeTreeId, query, {
    includeCategories: ['common', 'action', 'condition', 'subtree'],
  });
}

export function getInsertCategoryLabel(category: EditorInsertCatalogItem['category']): string {
  return categoryLabels[category];
}

export function getDefaultKnowledgeBaseCategoryIdForPhase(
  phase: BehaviorTreePhase
): EditorKnowledgeBaseCategoryId {
  if (phase === '梅林区') {
    return 'merlin';
  }
  if (phase === '武馆区') {
    return 'martial';
  }
  return 'duel';
}

export function filterKnowledgeBaseItems(
  items: EditorKnowledgeBaseItem[],
  query: string
): EditorKnowledgeBaseItem[] {
  return filterItems(items, query);
}

export function buildEditorKnowledgeBaseCatalog(
  document: EditorDocument | null | undefined,
  activeTreeId: string | null | undefined
): EditorKnowledgeBaseCategory[] {
  const grouped = new Map<EditorKnowledgeBaseCategoryId, EditorKnowledgeBaseItem[]>();

  registryEntries
    .filter((entry) => entry.tagName !== 'SubTree')
    .map((entry) => buildRegistryKnowledgeItem(entry))
    .forEach((item) => {
      const items = grouped.get(item.knowledgeBaseCategoryId) ?? [];
      items.push(item);
      grouped.set(item.knowledgeBaseCategoryId, items);
    });

  const subtreeItems = buildSubtreeKnowledgeItems(document, activeTreeId);
  if (subtreeItems.length > 0) {
    grouped.set('subtree', subtreeItems);
  }

  const order: EditorKnowledgeBaseCategoryId[] = [
    'merlin',
    'navigation',
    'vision',
    'duel',
    'martial',
    'official',
    'subtree',
  ];

  return order
    .map((id) => {
      const items = grouped.get(id) ?? [];
      const sortedItems =
        id === 'subtree'
          ? items
          : items.sort((left, right) => left.label.localeCompare(right.label, 'zh-CN'));
      return {
        id,
        title: knowledgeBaseCategoryMeta[id].title,
        description: knowledgeBaseCategoryMeta[id].description,
        items: sortedItems,
      };
    })
    .filter((category) => category.items.length > 0);
}
