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
    <header className="glass-panel p-4 flex items-center justify-between mb-4">
      <div className="flex items-center gap-6">
        <div className="flex items-center gap-3 bg-white/60 px-5 py-3 rounded-2xl">
          <span className="text-3xl">🎯</span>
          <h1 className="text-2xl font-bold text-gray-800" data-testid="active-phase-label">当前：{activePhase}</h1>
        </div>
        
        {appMode === 'viewer' ? (
          <div className="text-2xl text-slate-700 font-medium flex items-center gap-2">
            <Activity className="animate-pulse" />
            {isPlaying ? (
              <span>正在执行: {activeNode ? activeNode.label : '...'}</span>
            ) : (
              <span>已暂停 / 待命</span>
            )}
          </div>
        ) : (
          <div className="text-2xl text-slate-700 font-medium flex items-center gap-2">
            <Edit3 className="text-blue-500" />
            <span>编辑模式</span>
          </div>
        )}
      </div>

      <div className="flex items-center gap-4">
        <button 
          onClick={handleModeSwitch}
          data-testid="app-mode-toggle"
          className="flex items-center gap-2 px-6 py-3 rounded-2xl font-bold transition-all bg-indigo-100 text-indigo-700 hover:bg-indigo-200"
        >
          {appMode === 'viewer' ? <><Edit3 className="w-6 h-6" /> 进入编辑</> : <><Eye className="w-6 h-6" /> 退出编辑</>}
        </button>

        {appMode === 'viewer' && (
          <>
            <button 
              onClick={toggleSimulate}
              data-testid="simulate-mode-toggle"
              className={`flex items-center gap-2 px-6 py-3 rounded-2xl font-bold transition-all ${isSimulating ? 'bg-amber-100 text-amber-700' : 'bg-emerald-100 text-emerald-700'}`}
            >
              <Cpu className="w-6 h-6" />
              {isSimulating ? '模拟模式' : '实机模式'}
            </button>

            <div className="flex items-center gap-2 bg-white/60 p-2 rounded-2xl">
              <button onClick={togglePlay} data-testid="play-toggle" className="glass-button p-3 hover:text-slate-700">
                {isPlaying ? <Pause className="w-6 h-6" /> : <Play className="w-6 h-6" />}
              </button>
              <button onClick={resetTreeState} data-testid="reset-tree-state" className="glass-button p-3 hover:text-amber-600">
                <RotateCcw className="w-6 h-6" />
              </button>
            </div>
          </>
        )}
      </div>
    </header>
  );
};
