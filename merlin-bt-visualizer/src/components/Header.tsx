
import { Play, Pause, RotateCcw, Activity, Cpu } from 'lucide-react';
import { useStore } from '../store/useStore';

export const Header = () => {
  const { isSimulating, isPlaying, activePhase, toggleSimulate, togglePlay } = useStore();

  return (
    <header className="glass-panel p-4 flex items-center justify-between mb-4">
      <div className="flex items-center gap-6">
        <div className="flex items-center gap-3 bg-white/60 px-5 py-3 rounded-2xl">
          <span className="text-3xl">🎯</span>
          <h1 className="text-2xl font-bold text-gray-800">当前：{activePhase}</h1>
        </div>
        <div className="text-2xl text-blue-600 font-medium flex items-center gap-2">
          <Activity className="animate-pulse" />
          正在思考下一步：寻找 5 号格
        </div>
      </div>

      <div className="flex items-center gap-4">
        <button 
          onClick={toggleSimulate}
          className={`flex items-center gap-2 px-6 py-3 rounded-2xl font-bold transition-all ${isSimulating ? 'bg-amber-100 text-amber-700' : 'bg-emerald-100 text-emerald-700'}`}
        >
          <Cpu className="w-6 h-6" />
          {isSimulating ? '模拟模式' : '实机模式'}
        </button>

        <div className="flex items-center gap-2 bg-white/60 p-2 rounded-2xl">
          <button onClick={togglePlay} className="glass-button p-3 hover:text-blue-600">
            {isPlaying ? <Pause className="w-6 h-6" /> : <Play className="w-6 h-6" />}
          </button>
          <button className="glass-button p-3 hover:text-amber-600">
            <RotateCcw className="w-6 h-6" />
          </button>
        </div>
      </div>
    </header>
  );
};
