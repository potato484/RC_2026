import { useDeferredValue, useEffect, useMemo, useState } from 'react';
import { BookOpenText, ChevronRight, Info, Search, X } from 'lucide-react';
import { useEditorStore } from '../store/useEditorStore';
import { useStore } from '../store/useStore';
import { getBehaviorTreeNodeCategoryLabel } from '../utils/btDisplay';
import {
  buildEditorKnowledgeBaseCatalog,
  EditorKnowledgeBaseCategoryId,
  EditorKnowledgeBaseItem,
  filterKnowledgeBaseItems,
  getDefaultKnowledgeBaseCategoryIdForPhase,
} from '../utils/editorInsertCatalog';

interface EditorKnowledgeBaseProps {
  className?: string;
  mode?: 'workspace' | 'sheet';
  onRequestClose: () => void;
}

function formatChildPolicy(policy: EditorKnowledgeBaseItem['childPolicy']): string {
  if (policy.max === 0) {
    return '不接子节点';
  }

  if (policy.max === null) {
    return policy.min <= 1 ? '至少 1 条子节点，无上限' : `至少 ${policy.min} 条子节点，无上限`;
  }

  if (policy.min === policy.max) {
    return `固定 ${policy.max} 条子节点`;
  }

  return `${policy.min} 到 ${policy.max} 条子节点`;
}

function getDirectionLabel(direction: 'input' | 'output' | 'inout'): string {
  if (direction === 'input') {
    return '输入';
  }
  if (direction === 'output') {
    return '输出';
  }
  return '双向';
}

function getSourceStyle(source: EditorKnowledgeBaseItem['source']): string {
  if (source === 'robot') {
    return 'bg-emerald-100 text-emerald-700';
  }
  if (source === 'official') {
    return 'bg-sky-100 text-sky-700';
  }
  return 'bg-slate-100 text-slate-600';
}

function getCategoryTint(categoryId: EditorKnowledgeBaseCategoryId, active: boolean): string {
  if (!active) {
    return 'border-slate-200 bg-white text-slate-600 hover:border-slate-300 hover:bg-slate-50';
  }

  switch (categoryId) {
    case 'merlin':
      return 'border-sky-300 bg-sky-50 text-sky-700';
    case 'navigation':
      return 'border-cyan-300 bg-cyan-50 text-cyan-700';
    case 'vision':
      return 'border-amber-300 bg-amber-50 text-amber-700';
    case 'duel':
      return 'border-rose-300 bg-rose-50 text-rose-700';
    case 'martial':
      return 'border-emerald-300 bg-emerald-50 text-emerald-700';
    case 'subtree':
      return 'border-violet-300 bg-violet-50 text-violet-700';
    default:
      return 'border-slate-300 bg-slate-100 text-slate-700';
  }
}

export const EditorKnowledgeBase = ({
  className = '',
  mode = 'workspace',
  onRequestClose,
}: EditorKnowledgeBaseProps) => {
  const [query, setQuery] = useState('');
  const [activeCategoryId, setActiveCategoryId] = useState('');
  const [selectedItemId, setSelectedItemId] = useState('');
  const deferredQuery = useDeferredValue(query);
  const document = useEditorStore((state) => state.document);
  const activeTreeId = useEditorStore((state) => state.activeTreeId);
  const activePhase = useStore((state) => state.activePhase);

  const categories = useMemo(
    () => buildEditorKnowledgeBaseCatalog(document, activeTreeId),
    [activeTreeId, document]
  );

  const activeCategory = useMemo(
    () => categories.find((category) => category.id === activeCategoryId) ?? null,
    [activeCategoryId, categories]
  );

  const filteredItems = useMemo(
    () => (activeCategory ? filterKnowledgeBaseItems(activeCategory.items, deferredQuery) : []),
    [activeCategory, deferredQuery]
  );

  const selectedItem = useMemo(
    () => filteredItems.find((item) => item.id === selectedItemId) ?? null,
    [filteredItems, selectedItemId]
  );

  const keywords = useMemo(
    () =>
      selectedItem
        ? Array.from(new Set([...selectedItem.keywordsZh, ...selectedItem.keywordsEn])).filter(Boolean)
        : [],
    [selectedItem]
  );

  useEffect(() => {
    if (categories.length === 0) {
      setActiveCategoryId('');
      return;
    }

    const defaultCategoryId = getDefaultKnowledgeBaseCategoryIdForPhase(activePhase);
    const fallbackId =
      categories.find((category) => category.id === defaultCategoryId)?.id ??
      categories.find((category) => category.id === 'official')?.id ??
      categories[0].id;

    if (!categories.some((category) => category.id === activeCategoryId)) {
      setActiveCategoryId(fallbackId);
    }
  }, [activeCategoryId, activePhase, categories]);

  useEffect(() => {
    if (filteredItems.length === 0) {
      setSelectedItemId('');
      return;
    }

    if (!filteredItems.some((item) => item.id === selectedItemId)) {
      setSelectedItemId(filteredItems[0].id);
    }
  }, [filteredItems, selectedItemId]);

  return (
    <div
      className={`flex h-full min-h-0 flex-col overflow-hidden bg-slate-50 ${
        mode === 'sheet' ? 'rounded-[24px]' : 'rounded-[28px]'
      } ${className}`}
      data-testid="editor-knowledge-base"
    >
      <div className="shrink-0 border-b border-slate-200 bg-white/96 px-4 py-4">
        <div className="flex items-start justify-between gap-3">
          <div className="min-w-0">
            <div className="flex items-center gap-2 text-slate-800">
              <BookOpenText className="h-5 w-5 text-sky-600" />
              <h2 className="text-base font-bold">节点知识库</h2>
            </div>
            <p className="mt-1 text-xs leading-5 text-slate-500">
              这里只负责查节点作用、参数和约束。先选业务类目，再看该类里的具体节点说明。
            </p>
          </div>

          <button
            type="button"
            onClick={onRequestClose}
            className="rounded-xl p-2 text-slate-400 transition hover:bg-slate-100 hover:text-slate-700"
            aria-label="关闭节点知识库"
          >
            <X className="h-4 w-4" />
          </button>
        </div>

        <div className="mt-4 flex items-center gap-2 rounded-2xl border border-slate-200 bg-white px-3 py-2">
          <Search className="h-4 w-4 text-slate-400" />
          <input
            value={query}
            onChange={(event) => setQuery(event.target.value)}
            placeholder={
              activeCategory ? `在“${activeCategory.title}”里搜索节点、说明或参数名` : '先选择一个业务类目'
            }
            className="w-full bg-transparent text-sm text-slate-700 outline-none placeholder:text-slate-400"
            data-testid="editor-knowledge-base-search"
          />
        </div>
      </div>

      <div className="min-h-0 flex flex-1 flex-col overflow-hidden lg:flex-row">
        <div className="shrink-0 border-b border-slate-200 bg-white/94 lg:flex lg:min-h-0 lg:w-[228px] lg:flex-col lg:border-b-0 lg:border-r">
          <div className="px-4 pb-2 pt-4 text-xs font-bold tracking-wide text-slate-400 lg:shrink-0">业务分类</div>
          <div className="flex gap-2 overflow-x-auto px-4 pb-4 lg:min-h-0 lg:flex-1 lg:flex-col lg:overflow-x-visible lg:overflow-y-auto">
            {categories.map((category) => {
              const isActive = category.id === activeCategory?.id;
              return (
                <button
                  key={category.id}
                  type="button"
                  onClick={() => setActiveCategoryId(category.id)}
                  data-testid={`editor-knowledge-category-${category.id}`}
                  className={`min-w-[148px] rounded-2xl border px-3 py-3 text-left transition lg:min-w-0 ${getCategoryTint(
                    category.id,
                    isActive
                  )}`}
                >
                  <div className="flex items-center justify-between gap-3">
                    <span className="text-sm font-bold">{category.title}</span>
                    <span className="rounded-full bg-white/90 px-2 py-0.5 text-[10px] font-semibold text-slate-500 ring-1 ring-slate-200">
                      {category.items.length}
                    </span>
                  </div>
                  <div className="mt-1 text-[11px] leading-5 text-slate-500">{category.description}</div>
                </button>
              );
            })}
          </div>
        </div>

        <div className="min-h-0 flex flex-1 flex-col overflow-hidden lg:flex-row">
          <div className="min-h-0 flex flex-1 flex-col border-b border-slate-200 bg-white/96 lg:w-[360px] lg:flex-none lg:border-b-0 lg:border-r">
            <div className="shrink-0 flex items-center justify-between gap-3 px-4 py-4">
              <div>
                <div
                  className="text-sm font-bold text-slate-800"
                  data-testid="editor-knowledge-active-category"
                >
                  {activeCategory?.title ?? '未选择类目'}
                </div>
                <div className="mt-1 text-xs leading-5 text-slate-500">
                  {activeCategory?.description ?? '请选择一个业务类目查看节点信息。'}
                </div>
              </div>
              <span className="rounded-full bg-slate-100 px-2 py-1 text-[11px] font-semibold text-slate-500">
                {filteredItems.length} 条
              </span>
            </div>

            <div
              className="min-h-0 flex-1 overflow-y-auto overscroll-contain px-4 pb-4"
              data-testid="editor-knowledge-list"
            >
              {filteredItems.length === 0 ? (
                <div
                  className="rounded-2xl border border-dashed border-slate-200 px-4 py-10 text-center text-sm text-slate-400"
                  data-testid="editor-knowledge-empty"
                >
                  {activeCategory
                    ? `“${activeCategory.title}” 当前筛选条件下没有匹配的节点说明。`
                    : '当前没有可浏览的知识库类目。'}
                </div>
              ) : (
                <div className="space-y-2">
                  {filteredItems.map((item) => {
                    const isSelected = item.id === selectedItem?.id;
                    return (
                      <button
                        key={item.id}
                        type="button"
                        onClick={() => setSelectedItemId(item.id)}
                        className={`flex w-full items-start gap-3 rounded-2xl border px-3 py-3 text-left transition ${
                          isSelected
                            ? 'border-sky-300 bg-sky-50/80 shadow-sm'
                            : 'border-slate-200 bg-white hover:border-slate-300 hover:bg-slate-50'
                        }`}
                      >
                        <div className="min-w-0 flex-1">
                          <div className="flex flex-wrap items-center gap-2">
                            <span className="text-sm font-bold text-slate-800">{item.label}</span>
                            <span className={`rounded-full px-2 py-0.5 text-[10px] font-semibold ${getSourceStyle(item.source)}`}>
                              {item.sourceLabel}
                            </span>
                            <span className="rounded-full bg-slate-100 px-2 py-0.5 text-[10px] font-semibold text-slate-500">
                              {getBehaviorTreeNodeCategoryLabel(item.category)}
                            </span>
                          </div>
                          <p className="mt-1 line-clamp-2 text-xs leading-5 text-slate-500">{item.description}</p>
                          <div className="mt-2 flex flex-wrap items-center gap-2 text-[11px] text-slate-400">
                            <span>{item.groupLabel}</span>
                            <span className="rounded-full bg-white px-2 py-0.5 ring-1 ring-slate-200">
                              {item.tagLabel}
                            </span>
                          </div>
                        </div>
                        <ChevronRight className="mt-0.5 h-4 w-4 shrink-0 text-slate-300" />
                      </button>
                    );
                  })}
                </div>
              )}
            </div>
          </div>

          <div
            className="min-h-0 flex-1 overflow-y-auto overscroll-contain bg-slate-50/92 px-4 py-4 lg:px-5"
            data-testid="editor-knowledge-detail"
          >
            {!selectedItem ? (
              <div className="flex h-full min-h-[220px] flex-col items-center justify-center rounded-3xl border border-dashed border-slate-200 bg-white/80 px-6 text-center text-slate-400">
                <Info className="mb-3 h-12 w-12 opacity-60" />
                <p>先从当前类目的节点列表里选一个节点，右边会展开它的功能解释和参数说明。</p>
              </div>
            ) : (
              <div className="space-y-4">
                <section className="rounded-3xl border border-slate-200 bg-white/92 p-5">
                  <div className="flex flex-wrap items-start justify-between gap-3">
                    <div className="min-w-0">
                      <div className="text-xl font-bold text-slate-900">{selectedItem.label}</div>
                      <div className="mt-2 flex flex-wrap items-center gap-2 text-xs">
                        <span className={`rounded-full px-2 py-1 font-semibold ${getSourceStyle(selectedItem.source)}`}>
                          {selectedItem.sourceLabel}
                        </span>
                        <span className="rounded-full bg-slate-100 px-2 py-1 font-semibold text-slate-600">
                          {getBehaviorTreeNodeCategoryLabel(selectedItem.category)}
                        </span>
                        <span className="rounded-full bg-white px-2 py-1 font-semibold text-slate-500 ring-1 ring-slate-200">
                          {selectedItem.tagName}
                        </span>
                      </div>
                    </div>
                    <div className="rounded-2xl bg-slate-50 px-3 py-2 text-right text-xs text-slate-500">
                      <div>业务域：{activeCategory?.title}</div>
                      <div className="mt-1">分组：{selectedItem.groupLabel}</div>
                      <div className="mt-1">子节点约束：{formatChildPolicy(selectedItem.childPolicy)}</div>
                    </div>
                  </div>

                  <p className="mt-4 text-sm leading-7 text-slate-600">{selectedItem.description}</p>
                </section>

                <section className="rounded-3xl border border-slate-200 bg-white/92 p-5">
                  <div className="mb-3 text-sm font-bold text-slate-800">声明端口</div>
                  {selectedItem.portSchemas.length === 0 ? (
                    <div className="rounded-2xl bg-slate-50 px-4 py-4 text-sm text-slate-500">
                      这个节点当前没有声明端口，通常只通过子节点结构表达行为。
                    </div>
                  ) : (
                    <div className="space-y-3">
                      {selectedItem.portSchemas.map((port) => (
                        <div key={port.name} className="rounded-2xl border border-slate-200 bg-slate-50/80 p-4">
                          <div className="flex flex-wrap items-start justify-between gap-3">
                            <div>
                              <div className="text-sm font-semibold text-slate-800">{port.labelZh}</div>
                              <div className="mt-1 text-xs text-slate-500">{port.name}</div>
                            </div>
                            <div className="flex flex-wrap items-center gap-2 text-[11px]">
                              <span className="rounded-full bg-white px-2 py-0.5 font-semibold text-slate-500 ring-1 ring-slate-200">
                                {getDirectionLabel(port.direction)}
                              </span>
                              <span className="rounded-full bg-white px-2 py-0.5 font-semibold text-slate-500 ring-1 ring-slate-200">
                                {port.valueType}
                              </span>
                              {port.defaultValue && (
                                <span className="rounded-full bg-emerald-50 px-2 py-0.5 font-semibold text-emerald-700">
                                  默认：{port.defaultValue}
                                </span>
                              )}
                            </div>
                          </div>
                          <p className="mt-2 text-sm leading-6 text-slate-600">{port.descriptionZh}</p>
                        </div>
                      ))}
                    </div>
                  )}
                </section>

                <section className="rounded-3xl border border-slate-200 bg-white/92 p-5">
                  <div className="mb-3 text-sm font-bold text-slate-800">检索关键词</div>
                  {keywords.length === 0 ? (
                    <div className="rounded-2xl bg-slate-50 px-4 py-4 text-sm text-slate-500">
                      当前节点还没有额外关键词，主要靠中文名、说明和端口名搜索。
                    </div>
                  ) : (
                    <div className="flex flex-wrap gap-2">
                      {keywords.map((keyword) => (
                        <span
                          key={keyword}
                          className="rounded-full bg-slate-100 px-3 py-1 text-xs font-semibold text-slate-600"
                        >
                          {keyword}
                        </span>
                      ))}
                    </div>
                  )}
                </section>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};
