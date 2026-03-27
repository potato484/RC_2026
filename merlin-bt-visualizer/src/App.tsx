import { useEffect } from 'react';
import { Header } from './components/Header';
import { Sidebar } from './components/Sidebar';
import { TreeVisualizer } from './components/TreeVisualizer';
import { RightPanel } from './components/RightPanel';
import { Timeline } from './components/Timeline';
import { useStore } from './store/useStore';

function App() {
  const { isPlaying, updateNodeState, addTimelineEvent, updateBlackboard } = useStore();

  // 模拟决策循环
  useEffect(() => {
    if (!isPlaying) return;

    let step = 0;
    const interval = setInterval(() => {
      const states = ['scan', 'move', 'grab', 'check_target'];
      const current = states[step % states.length];
      
      // 重置所有状态
      states.forEach(s => updateNodeState(s, 'idle'));
      updateNodeState('root', 'running');
      updateNodeState('action_sel', 'running');
      
      // 设置当前运行节点
      updateNodeState(current, 'running');
      
      // 模拟黑板更新
      if (current === 'scan') {
        updateBlackboard({ key: 'target', value: '寻找中', desc: '目标状态', updatedAt: Date.now() });
        addTimelineEvent({ id: Date.now().toString(), time: new Date().toLocaleTimeString(), desc: '正在环顾四周...', icon: 'scan', status: 'info' });
      } else if (current === 'move') {
        updateBlackboard({ key: 'target', value: '已锁定', desc: '目标状态', updatedAt: Date.now() });
        updateBlackboard({ key: 'distance', value: '1.2m', desc: '距离目标', updatedAt: Date.now() });
        addTimelineEvent({ id: Date.now().toString(), time: new Date().toLocaleTimeString(), desc: '锁定目标，开始移动', icon: 'move', status: 'success' });
      }

      step++;
    }, 2000);

    return () => clearInterval(interval);
  }, [isPlaying, updateNodeState, addTimelineEvent, updateBlackboard]);

  return (
    <div className="w-screen h-screen p-4 flex flex-col relative overflow-hidden bg-cover bg-center" style={{ backgroundImage: 'radial-gradient(circle at top left, #e0eafc, #cfdef3)' }}>
      <Header />
      <div className="flex-1 flex overflow-hidden">
        <Sidebar />
        <div className="flex-1 relative">
          <TreeVisualizer />
        </div>
        <RightPanel />
      </div>
      <Timeline />
    </div>
  );
}

export default App;
