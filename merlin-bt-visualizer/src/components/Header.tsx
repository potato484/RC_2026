import { Play, Pause, RotateCcw, Activity, Cpu, Edit3, Eye } from 'lucide-react';
import { useStore } from '../store/useStore';
import { useEditorStore } from '../store/useEditorStore';
import mfTreeXml from '../../../src/rc26_decision/behavior_trees/mf_tree.xml?raw';
import combatTreeXml from '../../../src/rc26_decision/behavior_trees/combat_tree.xml?raw';
import mcTreeXml from '../../../src/rc26_decision/behavior_trees/mc_tree.xml?raw';

export const Header = () => {
  const { isSimulating, isPlaying, activePhase, toggleSimulate, togglePlay, resetTreeState, activeNodeId, nodes, appMode, setAppMode } = useStore();
  const activeNode = nodes.find(n => n.id === activeNodeId);
  const loadXml = useEditorStore(state => state.loadXml);

  const handleModeSwitch = () => {
    if (appMode === 'viewer') {
      setAppMode('editor');
      // Load current area XML into editor store
      let xmlToLoad = mfTreeXml;
      if (activePhase === '武馆区') xmlToLoad = mcTreeXml;
      if (activePhase === '对抗区') xmlToLoad = combatTreeXml;
      loadXml(xmlToLoad);
    } else {
      setAppMode('viewer');
    }
  };

  return (
    <header className="glass-panel p-4 flex items-center justify-between mb-4">
      <div className="flex items-center gap-6">
        <div className="flex items-center gap-3 bg-white/60 px-5 py-3 rounded-2xl">
          <span className="text-3xl">🎯</span>
          <h1 className="text-2xl font-bold text-gray-800">当前：{activePhase}</h1>
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
          className="flex items-center gap-2 px-6 py-3 rounded-2xl font-bold transition-all bg-indigo-100 text-indigo-700 hover:bg-indigo-200"
        >
          {appMode === 'viewer' ? <><Edit3 className="w-6 h-6" /> 进入编辑</> : <><Eye className="w-6 h-6" /> 退出编辑</>}
        </button>

        {appMode === 'viewer' && (
          <>
            <button 
              onClick={toggleSimulate}
              className={`flex items-center gap-2 px-6 py-3 rounded-2xl font-bold transition-all ${isSimulating ? 'bg-amber-100 text-amber-700' : 'bg-emerald-100 text-emerald-700'}`}
            >
              <Cpu className="w-6 h-6" />
              {isSimulating ? '模拟模式' : '实机模式'}
            </button>

            <div className="flex items-center gap-2 bg-white/60 p-2 rounded-2xl">
              <button onClick={togglePlay} className="glass-button p-3 hover:text-slate-700">
                {isPlaying ? <Pause className="w-6 h-6" /> : <Play className="w-6 h-6" />}
              </button>
              <button onClick={resetTreeState} className="glass-button p-3 hover:text-amber-600">
                <RotateCcw className="w-6 h-6" />
              </button>
            </div>
          </>
        )}
      </div>
    </header>
  );
};
