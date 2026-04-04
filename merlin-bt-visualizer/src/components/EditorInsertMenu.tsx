import { useDeferredValue, useMemo, useState } from 'react';
import { Bot, FolderTree, GitBranch, Search, Sparkles, Waypoints, X } from 'lucide-react';
import { EditorDocument, EditorInsertTemplate } from '../types/editor';
import {
  buildEditorInsertCatalog,
  EditorInsertCatalogItem,
  getInsertCategoryLabel,
} from '../utils/editorInsertCatalog';

interface EditorInsertMenuProps {
  document: EditorDocument | null;
  activeTreeId: string | null;
  mode: 'floating' | 'sheet';
  onInsert: (template: EditorInsertTemplate) => void;
  onClose: () => void;
}

const iconByCategory: Record<EditorInsertCatalogItem['category'], typeof Sparkles> = {
  common: Sparkles,
  action: Bot,
  condition: Waypoints,
  control: GitBranch,
  subtree: FolderTree,
};

export const EditorInsertMenu = ({
  document,
  activeTreeId,
  mode,
  onInsert,
  onClose,
}: EditorInsertMenuProps) => {
  const [query, setQuery] = useState('');
  const deferredQuery = useDeferredValue(query);

  const sections = useMemo(
    () => buildEditorInsertCatalog(document, activeTreeId, deferredQuery),
    [activeTreeId, deferredQuery, document]
  );

  return (
    <div
      className={`w-full overflow-hidden rounded-[24px] border border-slate-200/90 bg-white/96 shadow-2xl backdrop-blur ${
        mode === 'sheet' ? 'max-h-[72vh]' : 'max-h-[560px] min-w-[360px] max-w-[440px]'
      }`}
      data-testid="editor-insert-menu"
    >
      <div className="border-b border-slate-200 px-4 py-3">
        <div className="flex items-center justify-between gap-3">
          <div>
            <div className="text-sm font-bold text-slate-800">在线条中插入节点</div>
            <p className="mt-1 text-xs leading-5 text-slate-500">
              选一个节点后会自动接好前后结构，不需要再手动拖线。
            </p>
          </div>
          <button
            type="button"
            onClick={onClose}
            className="rounded-xl p-2 text-slate-400 transition hover:bg-slate-100 hover:text-slate-700"
            aria-label="关闭插入菜单"
          >
            <X className="h-4 w-4" />
          </button>
        </div>

        <div className="mt-3 flex items-center gap-2 rounded-2xl border border-slate-200 bg-white px-3 py-2">
          <Search className="h-4 w-4 text-slate-400" />
          <input
            value={query}
            onChange={(event) => setQuery(event.target.value)}
            placeholder="搜索节点、模块、子树或英文标识"
            className="w-full bg-transparent text-sm text-slate-700 outline-none placeholder:text-slate-400"
          />
        </div>
      </div>

      <div className="max-h-[calc(72vh-5.25rem)] overflow-y-auto px-4 py-3">
        <div className="space-y-5">
          {sections.map((section) => (
            <section key={section.id}>
              <div className="mb-2 flex items-center gap-2 text-xs font-bold tracking-wide text-slate-400">
                <span>{section.title}</span>
                <span className="rounded-full bg-slate-100 px-2 py-0.5 text-[10px] text-slate-500">
                  {section.items.length}
                </span>
              </div>

              <div className="space-y-2">
                {section.items.map((item) => {
                  const Icon = iconByCategory[item.category];
                  return (
                    <button
                      key={item.id}
                      type="button"
                      onClick={() => onInsert(item.template)}
                      className="flex w-full items-start gap-3 rounded-2xl border border-slate-200 bg-white px-3 py-3 text-left transition hover:border-slate-300 hover:bg-slate-50"
                    >
                      <div className="mt-0.5 rounded-xl bg-slate-100 p-2 text-slate-600">
                        <Icon className="h-4 w-4" />
                      </div>
                      <div className="min-w-0 flex-1">
                        <div className="flex flex-wrap items-center gap-2">
                          <span className="text-sm font-bold text-slate-800">{item.label}</span>
                          <span className="rounded-full bg-slate-100 px-2 py-0.5 text-[10px] font-semibold text-slate-500">
                            {getInsertCategoryLabel(item.category)}
                          </span>
                          <span className="rounded-full bg-white px-2 py-0.5 text-[10px] font-semibold text-slate-400 ring-1 ring-slate-200">
                            {item.sourceLabel}
                          </span>
                        </div>
                        <p className="mt-1 line-clamp-2 text-xs leading-5 text-slate-500">
                          {item.description}
                        </p>
                        <div className="mt-2 flex flex-wrap items-center gap-2 text-[11px] text-slate-400">
                          <span>{item.groupLabel}</span>
                          <span className="rounded-full bg-slate-100 px-2 py-0.5 font-semibold text-slate-500">
                            {item.tagLabel}
                          </span>
                        </div>
                      </div>
                    </button>
                  );
                })}
              </div>
            </section>
          ))}

          {sections.length === 0 && (
            <div className="rounded-2xl border border-dashed border-slate-200 px-4 py-10 text-center text-sm text-slate-400">
              当前筛选条件下没有可插入的节点。
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
