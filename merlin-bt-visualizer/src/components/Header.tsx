import { Play, Pause, RotateCcw, Activity, Cpu, Edit3, Eye } from 'lucide-react';
import { useStore } from '../store/useStore';

export const Header = () => {
  const { isSimulating, isPlaying, activePhase, toggleSimulate, togglePlay, resetTreeState, activeNodeId, nodes, appMode, setAppMode } = useStore();
  const activeNode = nodes.find(n => n.id === activeNodeId);

  const handleModeSwitch = () => {
    if (appMode === 'viewer') {
      setAppMode('editor');
    } else {
      setAppMode('viewer');
    }
  };

  return (
    <header className="glass-panel mb-4 flex flex-col gap-3 p-4 lg:flex-row lg:items-center lg:justify-between">
      <div className="flex flex-wrap items-center gap-3 lg:gap-6">
        <div className="flex items-center gap-3 rounded-2xl bg-white/60 px-4 py-3 lg:px-5">
          <span className="text-2xl lg:text-3xl">🎯</span>
          <h1 className="text-lg font-bold text-gray-800 lg:text-2xl" data-testid="active-phase-label">当前：{activePhase}</h1>
        </div>
        
        {appMode === 'viewer' ? (
          <div className="flex min-w-0 items-center gap-2 text-base font-medium text-slate-700 lg:text-2xl">
            <Activity className="animate-pulse" />
            {isPlaying && activeNode ? (
              <span className="truncate">正在执行: {activeNode.label}</span>
            ) : (
              <span>已暂停 / 待命</span>
            )}
          </div>
        ) : (
          <div className="flex items-center gap-2 text-base font-medium text-slate-700 lg:text-2xl">
            <Edit3 className="text-blue-500" />
            <span>编辑模式</span>
          </div>
        )}
      </div>

      <div className="flex flex-wrap items-center justify-end gap-2 lg:gap-4">
        <button 
          onClick={handleModeSwitch}
          data-testid="app-mode-toggle"
          className="flex items-center gap-2 rounded-2xl bg-indigo-100 px-4 py-2.5 font-bold text-indigo-700 transition-all hover:bg-indigo-200 lg:px-6 lg:py-3"
        >
          {appMode === 'viewer' ? <><Edit3 className="h-5 w-5 lg:h-6 lg:w-6" /> 进入编辑</> : <><Eye className="h-5 w-5 lg:h-6 lg:w-6" /> 退出编辑</>}
        </button>

        {appMode === 'viewer' && (
          <>
            <button 
              onClick={toggleSimulate}
              data-testid="simulate-mode-toggle"
              className={`flex items-center gap-2 rounded-2xl px-4 py-2.5 font-bold transition-all lg:px-6 lg:py-3 ${isSimulating ? 'bg-amber-100 text-amber-700' : 'bg-emerald-100 text-emerald-700'}`}
            >
              <Cpu className="h-5 w-5 lg:h-6 lg:w-6" />
              {isSimulating ? '模拟模式' : '实机模式'}
            </button>

            <div className="flex items-center gap-1 rounded-2xl bg-white/60 p-1.5 lg:gap-2 lg:p-2">
              <button onClick={togglePlay} data-testid="play-toggle" className="glass-button p-2.5 hover:text-slate-700 lg:p-3">
                {isPlaying ? <Pause className="h-5 w-5 lg:h-6 lg:w-6" /> : <Play className="h-5 w-5 lg:h-6 lg:w-6" />}
              </button>
              <button onClick={resetTreeState} data-testid="reset-tree-state" className="glass-button p-2.5 hover:text-amber-600 lg:p-3">
                <RotateCcw className="h-5 w-5 lg:h-6 lg:w-6" />
              </button>
            </div>
          </>
        )}
      </div>
    </header>
  );
};
