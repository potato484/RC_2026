import { useStore } from '../store/useStore';
import { Database, Target, MapPin, Activity, ListOrdered, Clock } from 'lucide-react';
import { TimelineEvent } from '../types';

const typeTranslations: Record<string, string> = {
  sequence: '顺序',
  selector: '选择',
  condition: '条件',
  action: '动作',
  decorator: '装饰',
  subtree: '子树',
};

export const RightPanel = () => {
  const { blackboard, activeNodeId, nodes, timeline } = useStore();
  const activeNode = nodes.find(n => n.id === activeNodeId);

  return (
    <div className="w-[320px] ml-4 flex flex-col gap-4 h-full overflow-hidden">
      {/* Node Info Panel */}
      <div className="glass-panel p-4 flex-none max-h-[300px] flex flex-col">
        <h3 className="text-lg font-bold flex items-center gap-2 mb-4 text-slate-800 flex-none">
          <Activity className="w-5 h-5 text-slate-600" />
          节点详情
        </h3>
        
        {activeNode ? (
          <div className="space-y-4 overflow-y-auto pr-2 custom-scrollbar flex-1 min-h-0">
            <div className="p-3 bg-white/50 rounded-xl border border-slate-200 flex-none">
              <div className="text-sm text-slate-500 mb-1">节点名称</div>
              <div className="font-semibold text-slate-800 break-all">{activeNode.label}</div>
            </div>
            <div className="p-3 bg-white/50 rounded-xl border border-slate-200 flex-none">
              <div className="text-sm text-slate-500 mb-1">节点类型</div>
              <div className="font-semibold text-slate-800">{typeTranslations[activeNode.type] || activeNode.type}</div>
            </div>
            <div className="p-3 bg-white/50 rounded-xl border border-slate-200 flex-none">
              <div className="text-sm text-slate-500 mb-1">节点描述</div>
              <div className="text-sm text-slate-700">{activeNode.desc}</div>
            </div>

            {/* Attached Decorators */}
            {activeNode.decorators && activeNode.decorators.length > 0 && (
              <div className="space-y-2 flex-none">
                <div className="text-sm text-slate-500 px-1">附加装饰器</div>
                {activeNode.decorators.map((mod) => (
                  <div key={mod.id} className="p-3 bg-rose-50/50 rounded-xl border border-rose-100">
                    <div className="flex items-center gap-2 mb-1">
                      <div className="w-2 h-2 rounded-full bg-rose-400" />
                      <div className="font-semibold text-rose-800 text-sm">{mod.label}</div>
                    </div>
                    <div className="text-xs text-rose-700/80">{mod.desc}</div>
                  </div>
                ))}
              </div>
            )}

            <div className="p-3 bg-white/50 rounded-xl border border-slate-200 flex-none">
              <div className="text-sm text-slate-500 mb-1">当前状态</div>
              <div className={`font-semibold inline-flex items-center gap-2 px-2 py-1 rounded-md
                ${activeNode.state === 'running' ? 'bg-amber-100 text-amber-700' :
                  activeNode.state === 'success' ? 'bg-emerald-100 text-emerald-700' :
                  activeNode.state === 'failure' ? 'bg-red-100 text-red-700' :
                  'bg-slate-100 text-slate-700'}`}
              >
                {activeNode.state === 'running' && '运行中'}
                {activeNode.state === 'success' && '成功'}
                {activeNode.state === 'failure' && '失败'}
                {activeNode.state === 'idle' && '空闲'}
              </div>
            </div>
          </div>
        ) : (
          <div className="h-full flex flex-col items-center justify-center text-slate-400 opacity-60 flex-1 min-h-0">
            <ListOrdered className="w-12 h-12 mb-2" />
            <p>点击左侧节点查看详情</p>
          </div>
        )}
      </div>

      {/* Timeline Panel */}
      <div className="glass-panel p-4 flex-1 flex flex-col min-h-0">
        <h3 className="text-lg font-bold flex items-center gap-2 mb-4 text-slate-800 flex-none">
          <Clock className="w-5 h-5 text-indigo-500" />
          执行日志
        </h3>
        <div className="flex-1 overflow-y-auto pr-2 space-y-3 custom-scrollbar min-h-0">
          {timeline.length > 0 ? timeline.map((event: TimelineEvent) => (
            <div key={event.id} className="relative pl-4 border-l-2 border-slate-200 pb-2 last:pb-0">
              <div className={`absolute -left-1.5 top-1.5 w-2.5 h-2.5 rounded-full ${
                event.status === 'success' ? 'bg-emerald-500 ring-2 ring-emerald-200' :
                event.status === 'warning' ? 'bg-amber-500 ring-2 ring-amber-200' :
                'bg-slate-500 ring-2 ring-slate-200'
              }`} />
              <div className="text-xs text-slate-400 font-mono mb-0.5">{event.time}</div>
              <div className="text-sm text-slate-700 break-words">{event.desc}</div>
            </div>
          )) : (
             <div className="h-full flex flex-col items-center justify-center text-slate-400 opacity-60">
                <Clock className="w-8 h-8 mb-2" />
                <p className="text-sm">暂无日志</p>
             </div>
          )}
        </div>
      </div>

      {/* Blackboard Panel */}
      <div className="glass-panel p-4 h-[250px] flex-none flex flex-col">
        <h3 className="text-lg font-bold flex items-center gap-2 mb-4 text-slate-800 flex-none">
          <Database className="w-5 h-5 text-emerald-500" />
          全局黑板
        </h3>
        
        <div className="flex-1 overflow-y-auto pr-2 space-y-3 custom-scrollbar min-h-0">
          {blackboard.length > 0 ? blackboard.map((item) => (
            <div key={item.key} className="p-3 bg-white/50 rounded-xl border border-slate-200 hover:border-emerald-300 transition-colors">
              <div className="flex items-center justify-between mb-1">
                <span className="text-sm font-semibold text-slate-700 font-mono">{item.key}</span>
                <span className="text-xs text-slate-400">
                  {new Date(item.updatedAt).toLocaleTimeString()}
                </span>
              </div>
              <div className="flex items-center gap-2">
                {item.key.includes('target') ? <Target className="w-4 h-4 text-rose-500" /> : <MapPin className="w-4 h-4 text-slate-500" />}
                <span className="font-bold text-emerald-600">{item.value}</span>
              </div>
              <div className="text-xs text-slate-500 mt-1">{item.desc}</div>
            </div>
          )) : (
            <div className="h-full flex flex-col items-center justify-center text-slate-400 opacity-60">
              <Database className="w-8 h-8 mb-2" />
              <p className="text-sm">黑板暂无数据</p>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
