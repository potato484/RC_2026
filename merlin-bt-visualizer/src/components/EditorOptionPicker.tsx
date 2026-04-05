import { useMemo, useState } from 'react';
import { Check } from 'lucide-react';
import {
  EditorOverlayPanel,
  editorOverlayPanelCountBadgeClassName,
} from './EditorOverlayPanel';

export interface EditorOptionPickerItem {
  id: string;
  label: string;
  description?: string;
  badge?: string;
  meta?: string;
  detail?: string;
  searchTokens?: string[];
}

export interface EditorOptionPickerSection {
  id: string;
  title: string;
  layout?: 'grid' | 'list';
  items: EditorOptionPickerItem[];
}

interface EditorOptionPickerProps {
  title: string;
  description: string;
  sections: EditorOptionPickerSection[];
  selectedId?: string | null;
  searchPlaceholder?: string;
  emptyText?: string;
  dataTestId?: string;
  onSelect: (itemId: string) => void;
  onClose: () => void;
}

const normalizeToken = (value: string) => value.trim().toLowerCase();

export const EditorOptionPicker = ({
  title,
  description,
  sections,
  selectedId = null,
  searchPlaceholder,
  emptyText = '当前筛选条件下没有可选项。',
  dataTestId,
  onSelect,
  onClose,
}: EditorOptionPickerProps) => {
  const [query, setQuery] = useState('');
  const showSearch = Boolean(searchPlaceholder);

  const filteredSections = useMemo(() => {
    const normalizedQuery = normalizeToken(query);
    if (!normalizedQuery) {
      return sections;
    }

    return sections
      .map((section) => ({
        ...section,
        items: section.items.filter((item) => {
          const haystack = [
            item.label,
            item.description ?? '',
            item.badge ?? '',
            item.meta ?? '',
            item.detail ?? '',
            ...(item.searchTokens ?? []),
          ];

          return haystack.some((token) => normalizeToken(token).includes(normalizedQuery));
        }),
      }))
      .filter((section) => section.items.length > 0);
  }, [query, sections]);

  return (
    <div
      className="fixed inset-0 z-50 bg-slate-950/24 p-3 backdrop-blur-[1.5px]"
      data-testid={dataTestId}
      onClick={onClose}
    >
      <div
        className="absolute inset-x-3 bottom-3 top-20 lg:inset-auto lg:left-1/2 lg:top-1/2 lg:-translate-x-1/2 lg:-translate-y-1/2"
        onClick={(event) => event.stopPropagation()}
      >
        <EditorOverlayPanel
          title={title}
          hint={description}
          closeLabel={`关闭${title}`}
          onClose={onClose}
          surfaceClassName="h-full w-full lg:h-auto lg:min-w-[360px] lg:max-w-[460px] lg:max-h-[560px]"
          search={
            showSearch
              ? {
                  value: query,
                  placeholder: searchPlaceholder ?? '',
                  onChange: setQuery,
                }
              : undefined
          }
        >
          <div className="space-y-5">
            {filteredSections.map((section) => (
              <section key={section.id}>
                <div className="mb-2 flex items-center gap-2 text-xs font-bold tracking-wide text-slate-500">
                  <span>{section.title}</span>
                  <span className={editorOverlayPanelCountBadgeClassName}>{section.items.length}</span>
                </div>

                <div className={section.layout === 'grid' ? 'grid grid-cols-2 gap-3' : 'space-y-3'}>
                  {section.items.map((item) => {
                    const isSelected = item.id === selectedId;
                    return (
                      <button
                        key={item.id}
                        type="button"
                        onClick={() => onSelect(item.id)}
                        className={`relative w-full rounded-[22px] border px-3 py-3 text-left transition ${
                          isSelected
                            ? 'border-sky-300 bg-sky-50/80 text-sky-700 shadow-[0_18px_36px_-28px_rgba(14,165,233,0.45)]'
                            : 'border-slate-200/90 bg-white text-slate-700 shadow-[0_16px_30px_-28px_rgba(15,23,42,0.34)] hover:border-sky-200 hover:bg-slate-50'
                        }`}
                        aria-pressed={isSelected}
                      >
                        <div className="flex items-start justify-between gap-3">
                          <div className="min-w-0 flex-1">
                            <div className="flex flex-wrap items-center gap-2">
                              <span className="text-sm font-bold text-current">{item.label}</span>
                              {item.badge && (
                                <span
                                  className={`rounded-full px-2 py-0.5 text-[10px] font-semibold ${
                                    isSelected
                                      ? 'border border-sky-100 bg-white text-sky-700'
                                      : 'bg-slate-100 text-slate-500'
                                  }`}
                                >
                                  {item.badge}
                                </span>
                              )}
                            </div>

                            {item.description && (
                              <p className="mt-1 text-xs leading-5 text-slate-600">{item.description}</p>
                            )}

                            {(item.meta || item.detail) && (
                              <div className="mt-2 flex flex-wrap items-center gap-2 text-[11px] text-slate-400">
                                {item.meta && <span>{item.meta}</span>}
                                {item.detail && (
                                  <span className="rounded-full bg-slate-100 px-2 py-0.5 font-semibold text-slate-500">
                                    {item.detail}
                                  </span>
                                )}
                              </div>
                            )}
                          </div>

                          {isSelected && (
                            <span className="rounded-full border border-sky-100 bg-white p-1 text-sky-600 shadow-sm">
                              <Check className="h-4 w-4" />
                            </span>
                          )}
                        </div>
                      </button>
                    );
                  })}
                </div>
              </section>
            ))}

            {filteredSections.length === 0 && (
              <div className="rounded-[22px] border border-dashed border-sky-100 bg-white px-4 py-10 text-center text-sm text-slate-400">
                {emptyText}
              </div>
            )}
          </div>
        </EditorOverlayPanel>
      </div>
    </div>
  );
};
