import { useDeferredValue, useMemo, useState } from 'react';
import { Boxes, Search } from 'lucide-react';
import { getBtNodeRegistry } from '../utils/btRegistry';
import { useEditorStore } from '../store/useEditorStore';

export const EditorPalette = () => {
  const [tab, setTab] = useState<'official' | 'robot'>('robot');
  const [query, setQuery] = useState('');
  const deferredQuery = useDeferredValue(query);
  const selectedNodeId = useEditorStore((state) => state.selectedNodeId);
  const insertNode = useEditorStore((state) => state.insertNode);

  const registry = useMemo(() => {
    const keyword = deferredQuery.trim().toLowerCase();
    return getBtNodeRegistry()
      .filter((entry) => entry.source === tab)
      .filter((entry) => {
        if (!keyword) return true;
        return (
          entry.labelZh.toLowerCase().includes(keyword) ||
          entry.group.toLowerCase().includes(keyword) ||
          entry.tagName.toLowerCase().includes(keyword) ||
          entry.keywordsZh.some((item) => item.toLowerCase().includes(keyword)) ||
          entry.keywordsEn.some((item) => item.toLowerCase().includes(keyword))
        );
      });
  }, [deferredQuery, tab]);

  const grouped = useMemo(() => {
    return registry.reduce<Record<string, typeof registry>>((accumulator, entry) => {
      if (!accumulator[entry.group]) {
        accumulator[entry.group] = [];
      }
      accumulator[entry.group].push(entry);
      return accumulator;
    }, {});
  }, [registry]);

  return (
    <div className="glass-panel absolute left-4 top-20 z-20 flex h-[calc(100%-6rem)] w-[320px] flex-col overflow-hidden border border-white/60 bg-white/85">
      <div className="border-b border-slate-200 px-4 py-3">
        <div className="flex items-center gap-2 text-slate-800">
          <Boxes className="h-5 w-5 text-sky-600" />
          <h2 className="text-base font-bold">节点库</h2>
        </div>
        <p className="mt-1 text-xs text-slate-500">支持拖拽到画布节点的插入槽，也支持一键插入到当前选中位置。</p>
      </div>

      <div className="border-b border-slate-200 px-4 py-3">
        <div className="flex items-center gap-2 rounded-2xl border border-slate-200 bg-white px-3 py-2">
          <Search className="h-4 w-4 text-slate-400" />
          <input
            value={query}
            onChange={(event) => setQuery(event.target.value)}
            placeholder="搜索节点、模块或术语"
            className="w-full bg-transparent text-sm text-slate-700 outline-none placeholder:text-slate-400"
          />
        </div>

        <div className="mt-3 flex gap-2">
          <button
            type="button"
            onClick={() => setTab('robot')}
            className={`flex-1 rounded-xl px-3 py-2 text-sm font-semibold transition-colors ${
              tab === 'robot' ? 'bg-emerald-100 text-emerald-700' : 'bg-slate-100 text-slate-500'
            }`}
          >
            机器人模块
          </button>
          <button
            type="button"
            onClick={() => setTab('official')}
            className={`flex-1 rounded-xl px-3 py-2 text-sm font-semibold transition-colors ${
              tab === 'official' ? 'bg-sky-100 text-sky-700' : 'bg-slate-100 text-slate-500'
            }`}
          >
            官方节点
          </button>
        </div>
      </div>

      <div className="min-h-0 flex-1 overflow-y-auto px-4 py-3">
        {Object.keys(grouped).map((group) => (
          <section key={group} className="mb-5">
            <div className="mb-2 text-xs font-bold tracking-wide text-slate-400">{group}</div>
            <div className="space-y-2">
              {grouped[group].map((entry) => (
                <div
                  key={entry.tagName}
                  draggable
                  onDragStart={(event) => {
                    event.dataTransfer.setData('application/x-bt-node-template', entry.tagName);
                    event.dataTransfer.effectAllowed = 'copy';
                  }}
                  className="rounded-2xl border border-slate-200 bg-white/90 p-3 shadow-sm transition hover:border-slate-300"
                >
                  <div className="flex items-start justify-between gap-3">
                    <div className="min-w-0 flex-1">
                      <div className="text-sm font-bold text-slate-800">{entry.labelZh}</div>
                      <div className="mt-1 text-xs leading-5 text-slate-500">{entry.descriptionZh}</div>
                    </div>
                    <span className="rounded-full bg-slate-100 px-2 py-0.5 text-[10px] font-semibold text-slate-500">
                      {entry.category === 'condition' ? '条件' : entry.category === 'action' ? '动作' : entry.category === 'decorator' ? '装饰' : entry.category === 'subtree' ? '子树' : '控制'}
                    </span>
                  </div>

                  {entry.portSchemas.length > 0 && (
                    <div className="mt-2 text-[11px] text-slate-500">
                      端口：{entry.portSchemas.map((port) => port.labelZh).join('、')}
                    </div>
                  )}

                  <button
                    type="button"
                    disabled={!selectedNodeId}
                    onClick={() => {
                      if (selectedNodeId) {
                        insertNode(selectedNodeId, 'append_child', entry.tagName);
                      }
                    }}
                    className="mt-3 w-full rounded-xl bg-slate-100 px-3 py-2 text-sm font-semibold text-slate-700 transition hover:bg-slate-200 disabled:cursor-not-allowed disabled:opacity-40"
                  >
                    插入到当前选中位置
                  </button>
                </div>
              ))}
            </div>
          </section>
        ))}

        {registry.length === 0 && (
          <div className="rounded-2xl border border-dashed border-slate-200 px-4 py-8 text-center text-sm text-slate-400">
            当前筛选条件下没有可用节点。
          </div>
        )}
      </div>
    </div>
  );
};
