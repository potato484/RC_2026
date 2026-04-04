import { useDeferredValue, useMemo, useState } from 'react';
import { Boxes, Search, X } from 'lucide-react';
import { useEditorStore } from '../store/useEditorStore';
import { buildEditorInsertCatalog, getInsertCategoryLabel } from '../utils/editorInsertCatalog';

interface EditorPaletteProps {
  className?: string;
  mode?: 'floating' | 'sheet';
  onRequestClose?: () => void;
}

export const EditorPalette = ({
  className = '',
  mode = 'floating',
  onRequestClose,
}: EditorPaletteProps) => {
  const [query, setQuery] = useState('');
  const deferredQuery = useDeferredValue(query);
  const document = useEditorStore((state) => state.document);
  const activeTreeId = useEditorStore((state) => state.activeTreeId);
  const selectedNodeId = useEditorStore((state) => state.selectedNodeId);
  const insertNodeTemplate = useEditorStore((state) => state.insertNodeTemplate);

  const sections = useMemo(
    () => buildEditorInsertCatalog(document, activeTreeId, deferredQuery),
    [activeTreeId, deferredQuery, document]
  );

  return (
    <div
      className={`glass-panel flex min-h-0 flex-col overflow-hidden border border-white/60 bg-white/88 ${className}`}
    >
      <div className="border-b border-slate-200 px-4 py-3">
        <div className="flex items-center justify-between gap-3">
          <div className="flex items-center gap-2 text-slate-800">
            <Boxes className="h-5 w-5 text-sky-600" />
            <h2 className="text-base font-bold">节点库</h2>
          </div>
          {mode === 'sheet' && onRequestClose && (
            <button
              type="button"
              onClick={onRequestClose}
              className="rounded-xl p-2 text-slate-400 transition hover:bg-slate-100 hover:text-slate-700"
              aria-label="关闭节点库"
            >
              <X className="h-4 w-4" />
            </button>
          )}
        </div>
        <p className="mt-1 text-xs text-slate-500">
          直接插到当前选中节点下方，或拖到节点插槽和连线中点快速接入。
        </p>
      </div>

      <div className="border-b border-slate-200 px-4 py-3">
        <div className="flex items-center gap-2 rounded-2xl border border-slate-200 bg-white px-3 py-2">
          <Search className="h-4 w-4 text-slate-400" />
          <input
            value={query}
            onChange={(event) => setQuery(event.target.value)}
            placeholder="搜索节点、模块、子树或英文标识"
            className="w-full bg-transparent text-sm text-slate-700 outline-none placeholder:text-slate-400"
          />
        </div>
      </div>

      <div className="min-h-0 flex-1 overflow-y-auto px-4 py-3">
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
                {section.items.map((item) => (
                  <div
                    key={item.id}
                    draggable
                    onDragStart={(event) => {
                      event.dataTransfer.setData(
                        'application/x-bt-node-template',
                        JSON.stringify(item.template)
                      );
                      event.dataTransfer.effectAllowed = 'copy';
                    }}
                    className="rounded-2xl border border-slate-200 bg-white/92 p-3 shadow-sm transition hover:border-slate-300"
                  >
                    <div className="flex items-start justify-between gap-3">
                      <div className="min-w-0 flex-1">
                        <div className="flex flex-wrap items-center gap-2">
                          <div className="text-sm font-bold text-slate-800">{item.label}</div>
                          <span className="rounded-full bg-slate-100 px-2 py-0.5 text-[10px] font-semibold text-slate-500">
                            {getInsertCategoryLabel(item.category)}
                          </span>
                        </div>
                        <div className="mt-1 text-xs leading-5 text-slate-500">{item.description}</div>
                        <div className="mt-2 flex flex-wrap items-center gap-2 text-[11px] text-slate-400">
                          <span>{item.groupLabel}</span>
                          <span className="rounded-full bg-white px-2 py-0.5 ring-1 ring-slate-200">
                            {item.tagLabel}
                          </span>
                        </div>
                      </div>
                    </div>

                    <button
                      type="button"
                      disabled={!selectedNodeId}
                      onClick={() => {
                        if (!selectedNodeId) {
                          return;
                        }

                        insertNodeTemplate(selectedNodeId, 'append_child', item.template);
                        if (onRequestClose) {
                          onRequestClose();
                        }
                      }}
                      className="mt-3 w-full rounded-xl bg-slate-100 px-3 py-2 text-sm font-semibold text-slate-700 transition hover:bg-slate-200 disabled:cursor-not-allowed disabled:opacity-40"
                    >
                      插入到当前选中节点
                    </button>
                  </div>
                ))}
              </div>
            </section>
          ))}

          {sections.length === 0 && (
            <div className="rounded-2xl border border-dashed border-slate-200 px-4 py-8 text-center text-sm text-slate-400">
              当前筛选条件下没有可用节点。
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
