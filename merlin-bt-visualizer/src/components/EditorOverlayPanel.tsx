import { ReactNode } from 'react';
import { Search, X } from 'lucide-react';

interface EditorOverlayPanelSearchProps {
  value: string;
  placeholder: string;
  onChange: (value: string) => void;
  testId?: string;
}

interface EditorOverlayPanelProps {
  title: string;
  hint?: string;
  closeLabel: string;
  onClose: () => void;
  children: ReactNode;
  dataTestId?: string;
  headerContent?: ReactNode;
  search?: EditorOverlayPanelSearchProps;
  surfaceClassName?: string;
  bodyClassName?: string;
  bodyTestId?: string;
}

export const editorOverlayPanelCountBadgeClassName =
  'rounded-full border border-sky-100 bg-white px-2 py-0.5 text-[10px] text-slate-500 shadow-sm';

export const editorOverlayPanelGroupClassName =
  'rounded-[22px] border border-sky-100 bg-white/92 p-3 shadow-[0_18px_36px_-28px_rgba(14,165,233,0.35)]';

export const editorOverlayPanelSearchClassName =
  'mt-3 flex items-center gap-2 rounded-2xl border border-sky-100 bg-slate-50/90 px-3 py-2.5 shadow-sm';

export const EditorOverlayPanel = ({
  title,
  hint,
  closeLabel,
  onClose,
  children,
  dataTestId,
  headerContent,
  search,
  surfaceClassName = 'w-full',
  bodyClassName = '',
  bodyTestId,
}: EditorOverlayPanelProps) => {
  return (
    <div
      className={`flex min-h-0 flex-col overflow-hidden rounded-[26px] border border-sky-100 bg-white shadow-[0_30px_80px_-36px_rgba(15,23,42,0.42),0_14px_36px_-28px_rgba(14,165,233,0.3)] ${surfaceClassName}`}
      data-testid={dataTestId}
    >
      <div className="border-b border-sky-100/90 bg-gradient-to-b from-sky-50/95 via-white to-white px-4 py-4">
        <div className="flex items-start justify-between gap-3">
          <div className="min-w-0">
            <div className="text-sm font-bold text-slate-900">{title}</div>
            {hint && (
              <div className="mt-2 rounded-2xl border border-sky-100 bg-white/90 px-3 py-2 text-xs leading-5 text-slate-600 shadow-sm">
                {hint}
              </div>
            )}
          </div>
          <button
            type="button"
            onClick={onClose}
            className="rounded-2xl border border-sky-100 bg-white p-2 text-slate-400 shadow-sm transition hover:border-sky-200 hover:bg-sky-50 hover:text-sky-700"
            aria-label={closeLabel}
          >
            <X className="h-4 w-4" />
          </button>
        </div>

        {headerContent && <div className="mt-3 space-y-3">{headerContent}</div>}

        {search && (
          <div className={editorOverlayPanelSearchClassName}>
            <Search className="h-4 w-4 text-sky-500" />
            <input
              value={search.value}
              onChange={(event) => search.onChange(event.target.value)}
              placeholder={search.placeholder}
              data-testid={search.testId}
              className="w-full bg-transparent text-sm text-slate-700 outline-none placeholder:text-slate-400"
            />
          </div>
        )}
      </div>

      <div
        className={`min-h-0 flex-1 overflow-y-auto bg-gradient-to-b from-slate-50/75 via-white to-white px-4 py-4 ${bodyClassName}`}
        data-testid={bodyTestId}
      >
        {children}
      </div>
    </div>
  );
};
