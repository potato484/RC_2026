import { BtNodeRegistryEntry } from '../generated/btNodeRegistry';
import { EditorDocument, EditorInsertTemplate } from '../types/editor';
import { getBehaviorTreeTreeName } from './btDisplay';
import { getBtNodeRegistry } from './btRegistry';
import { behaviorTreeXmlByPhase } from './behaviorTreeSources';

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

interface EditorInsertCatalogOptions {
  includeCategories?: EditorInsertCatalogCategory[];
}

const registryEntries = getBtNodeRegistry();

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
    searchTokens: [
      entry.labelZh,
      entry.descriptionZh,
      entry.group,
      entry.tagName,
      ...entry.keywordsZh,
      ...entry.keywordsEn,
    ],
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

export function getCommonRobotInsertItems(limit = 8): EditorInsertCatalogItem[] {
  return registryEntries
    .filter((entry) => entry.source === 'robot')
    .sort((left, right) => {
      const countDiff = (tagUsageCounts.get(right.tagName) ?? 0) - (tagUsageCounts.get(left.tagName) ?? 0);
      if (countDiff !== 0) {
        return countDiff;
      }

      return left.labelZh.localeCompare(right.labelZh, 'zh-CN');
    })
    .slice(0, limit)
    .map((entry) => buildRegistryItem(entry, 'common'));
}

function filterItems(items: EditorInsertCatalogItem[], query: string): EditorInsertCatalogItem[] {
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
