import { useDeferredValue, useMemo, useState } from 'react';
import { Bot, FolderTree, GitBranch, Sparkles, Waypoints } from 'lucide-react';
import { EditorAlongBranchInsertRequest, EditorDocument, EditorInsertTemplate } from '../types/editor';
import {
  buildAlongBranchInsertCatalog,
  EditorInsertCatalogItem,
  getInsertCategoryLabel,
} from '../utils/editorInsertCatalog';
import { EditorBranchInsertPosition, getAlongBranchWrapperEntries } from '../utils/btRegistry';
import {
  EditorOverlayPanel,
  editorOverlayPanelCountBadgeClassName,
  editorOverlayPanelGroupClassName,
} from './EditorOverlayPanel';

interface EditorInsertMenuProps {
  document: EditorDocument | null;
  activeTreeId: string | null;
  mode: 'floating' | 'sheet';
  position?: EditorBranchInsertPosition;
  positionMode?: 'fixed' | 'choose';
  onInsert: (request: EditorAlongBranchInsertRequest) => void;
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
  position = 'after',
  positionMode = 'fixed',
  onInsert,
  onClose,
}: EditorInsertMenuProps) => {
  const [query, setQuery] = useState('');
  const [wrapperTagName, setWrapperTagName] = useState('');
  const [selectedPosition, setSelectedPosition] = useState<EditorBranchInsertPosition | null>(
    positionMode === 'choose' ? null : position
  );
  const deferredQuery = useDeferredValue(query);

  const wrapperOptions = useMemo(() => getAlongBranchWrapperEntries(), []);
  const sections = useMemo(
    () => buildAlongBranchInsertCatalog(document, activeTreeId, deferredQuery),
    [activeTreeId, deferredQuery, document]
  );
  const activePosition = positionMode === 'choose' ? selectedPosition : position;
  const isReadyToInsert = Boolean(wrapperTagName && activePosition);
  const positionLabel = activePosition === 'before' ? '前插' : '后插';

  const heading =
    positionMode === 'choose' ? '向当前节点插入新节点' : `沿当前支线${positionLabel}节点`;
  const description =
    positionMode === 'choose'
      ? '先定前插还是后插，再继续选包装和模板。'
      : '先选控制包装，再挑要插入的节点模板。';
  const surfaceClassName =
    mode === 'sheet'
      ? 'h-[72vh] max-h-[72vh] w-full'
      : 'h-[min(560px,72vh)] min-w-[360px] max-w-[440px]';

  const handleInsert = (template: EditorInsertTemplate) => {
    if (!wrapperTagName || !activePosition) {
      return;
    }

    onInsert({
      position: activePosition,
      wrapperTagName,
      template,
    });
  };

  return (
    <EditorOverlayPanel
      title={heading}
      hint={description}
      closeLabel="关闭插入菜单"
      onClose={onClose}
      dataTestId="editor-insert-menu"
      surfaceClassName={surfaceClassName}
      bodyClassName="overscroll-contain pb-5"
      bodyTestId="editor-insert-menu-body"
      search={{
        value: query,
        placeholder: '搜索节点、模块、子树或英文标识',
        onChange: setQuery,
      }}
      headerContent={
        <>
          {positionMode === 'choose' && (
            <div className={editorOverlayPanelGroupClassName}>
              <div className="flex items-center gap-2 text-xs font-bold tracking-wide text-slate-500">
                <Waypoints className="h-3.5 w-3.5 text-violet-600" />
                <span>插入位置</span>
              </div>
              <div className="mt-2 grid grid-cols-2 gap-2">
                {[
                  { value: 'before', label: '前插', desc: '新节点放在当前节点前面' },
                  { value: 'after', label: '后插', desc: '新节点放在当前节点后面' },
                ].map((option) => (
                  <button
                    key={option.value}
                    type="button"
                    onClick={() => setSelectedPosition(option.value as EditorBranchInsertPosition)}
                    data-testid={`editor-insert-position-${option.value}`}
                    className={`rounded-2xl border px-3 py-2 text-left text-xs font-semibold transition ${
                      selectedPosition === option.value
                        ? 'border-violet-300 bg-violet-50/85 text-violet-700 shadow-[0_14px_32px_-26px_rgba(139,92,246,0.45)]'
                        : 'border-slate-200/90 bg-white text-slate-600 shadow-[0_12px_28px_-26px_rgba(15,23,42,0.3)] hover:border-violet-200 hover:bg-violet-50/40'
                    }`}
                    aria-pressed={selectedPosition === option.value}
                  >
                    <div>{option.label}</div>
                    <div className="mt-1 text-[11px] font-medium text-slate-500">{option.desc}</div>
                  </button>
                ))}
              </div>
              {!activePosition && (
                <div className="mt-2 rounded-2xl border border-amber-100 bg-amber-50/80 px-3 py-2 text-[11px] leading-5 text-amber-700">
                  先明确这次是前插还是后插，避免工具栏入口默认把新节点塞到后面。
                </div>
              )}
            </div>
          )}

          <div className={editorOverlayPanelGroupClassName}>
            <div className="flex items-center gap-2 text-xs font-bold tracking-wide text-slate-500">
              <GitBranch className="h-3.5 w-3.5 text-sky-600" />
              <span>控制包装</span>
            </div>
            <div className="mt-2 grid grid-cols-2 gap-2">
              {wrapperOptions.map((option) => (
                <button
                  key={option.tagName}
                  type="button"
                  onClick={() => setWrapperTagName(option.tagName)}
                  className={`rounded-2xl border px-3 py-2 text-left text-xs font-semibold transition ${
                    wrapperTagName === option.tagName
                      ? 'border-sky-300 bg-sky-50/85 text-sky-700 shadow-[0_14px_32px_-26px_rgba(14,165,233,0.45)]'
                      : 'border-slate-200/90 bg-white text-slate-600 shadow-[0_12px_28px_-26px_rgba(15,23,42,0.3)] hover:border-sky-200 hover:bg-sky-50/40'
                  }`}
                  aria-pressed={wrapperTagName === option.tagName}
                >
                  <div>{option.labelZh}</div>
                  <div className="mt-1 text-[11px] font-medium text-slate-500">{option.tagName}</div>
                </button>
              ))}
            </div>
            {!isReadyToInsert && (
              <div className="mt-2 rounded-2xl border border-amber-100 bg-amber-50/80 px-3 py-2 text-[11px] leading-5 text-amber-700">
                先明确插入方向和控制关系，再选下面要插入的节点。
              </div>
            )}
          </div>
        </>
      }
    >
      <div className="space-y-5">
        {sections.map((section) => (
          <section key={section.id} data-testid={`editor-insert-section-${section.id}`}>
            <div className="mb-2 flex items-center gap-2 text-xs font-bold tracking-wide text-slate-500">
              <span>{section.title}</span>
              <span className={editorOverlayPanelCountBadgeClassName}>{section.items.length}</span>
            </div>

            <div
              className="grid grid-cols-2 gap-3"
              data-testid={`editor-insert-section-items-${section.id}`}
            >
              {section.items.map((item) => {
                const Icon = iconByCategory[item.category];
                return (
                  <button
                    key={item.id}
                    type="button"
                    onClick={() => handleInsert(item.template)}
                    disabled={!isReadyToInsert}
                    className="flex h-full w-full flex-col rounded-[22px] border border-slate-200/90 bg-white px-3 py-3 text-left shadow-[0_16px_30px_-28px_rgba(15,23,42,0.34)] transition hover:border-sky-200 hover:bg-slate-50 disabled:cursor-not-allowed disabled:opacity-45"
                  >
                    <div className="flex items-start gap-3">
                      <div className="mt-0.5 rounded-xl border border-sky-100 bg-sky-50/70 p-2 text-sky-700">
                        <Icon className="h-4 w-4" />
                      </div>
                      <div className="min-w-0 flex-1">
                        <div className="text-sm font-bold text-slate-800">{item.label}</div>
                        <div className="mt-2 flex flex-wrap items-center gap-2 text-[10px] font-semibold">
                          <span className="rounded-full bg-slate-100 px-2 py-0.5 text-slate-500">
                            {getInsertCategoryLabel(item.category)}
                          </span>
                          <span className="rounded-full border border-sky-100 bg-white px-2 py-0.5 text-slate-500">
                            {item.sourceLabel}
                          </span>
                        </div>
                      </div>
                    </div>
                    <p className="mt-3 line-clamp-3 text-xs leading-5 text-slate-600">
                      {item.description}
                    </p>
                    <div className="mt-3 flex flex-wrap items-center gap-2 text-[11px] text-slate-400">
                      <span>{item.groupLabel}</span>
                      <span className="rounded-full bg-slate-100 px-2 py-0.5 font-semibold text-slate-500">
                        {item.tagLabel}
                      </span>
                    </div>
                  </button>
                );
              })}
            </div>
          </section>
        ))}

        {sections.length === 0 && (
          <div className="rounded-[22px] border border-dashed border-sky-100 bg-white px-4 py-10 text-center text-sm text-slate-400">
            当前筛选条件下没有可插入的节点。
          </div>
        )}
      </div>
    </EditorOverlayPanel>
  );
};
