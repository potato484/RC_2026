import { useEffect, useRef } from 'react';
import { Header } from './components/Header';
import { Sidebar } from './components/Sidebar';
import { TreeVisualizer } from './components/TreeVisualizer';
import { RightPanel } from './components/RightPanel';
import { useStore } from './store/useStore';

function App() {
  const { isPlaying, isSimulating, updateNodeState, addTimelineEvent, updateBlackboard } = useStore();
  const isPlayingRef = useRef(isPlaying);
  
  useEffect(() => {
    isPlayingRef.current = isPlaying;
  }, [isPlaying]);

  useEffect(() => {
    if (!isPlaying || !isSimulating) return;

    const state = useStore.getState();
    const { nodes, edges } = state;
    if (nodes.length === 0) return;

    let timeoutId: ReturnType<typeof setTimeout>;
    let currentStack: string[] = [];
    
    // Find root (node with no incoming edges)
    const targets = new Set(edges.map(e => e.target));
    const root = nodes.find(n => !targets.has(n.id));
    
    if (!root) return;

    const executeNode = async (nodeId: string): Promise<'success' | 'failure' | 'running'> => {
      if (!isPlayingRef.current) return 'failure';

      const node = useStore.getState().nodes.find(n => n.id === nodeId);
      if (!node) return 'failure';

      currentStack.push(nodeId);
      updateNodeState(nodeId, 'running');
      
      // Simulate delay for visualization
      await new Promise(resolve => { timeoutId = setTimeout(resolve, 800); });
      if (!isPlayingRef.current) return 'failure';

      const childrenEdges = edges.filter(e => e.source === nodeId);
      const childrenIds = childrenEdges.map(e => e.target);

      let result: 'success' | 'failure' | 'running' = 'success';

      addTimelineEvent({
        id: Date.now().toString() + Math.random(),
        time: new Date().toLocaleTimeString(),
        desc: `执行节点: ${node.label}`,
        icon: 'scan',
        status: 'info'
      });

      if (node.type === 'action') {
        result = Math.random() > 0.1 ? 'success' : 'failure';
        if (node.label.includes('Nav')) {
          updateBlackboard({ key: 'location', value: node.label.replace('NavTo', ''), desc: '当前位置', updatedAt: Date.now() });
        }
      } else if (node.type === 'condition') {
        result = Math.random() > 0.2 ? 'success' : 'failure';
      } else if (node.type === 'sequence') {
        for (const childId of childrenIds) {
          const childResult = await executeNode(childId);
          if (!isPlayingRef.current) return 'failure';
          if (childResult === 'failure') {
            result = 'failure';
            break;
          }
          if (childResult === 'running') {
            result = 'running';
            break;
          }
        }
      } else if (node.type === 'selector') {
        result = 'failure';
        for (const childId of childrenIds) {
          const childResult = await executeNode(childId);
          if (!isPlayingRef.current) return 'failure';
          if (childResult === 'success') {
            result = 'success';
            break;
          }
          if (childResult === 'running') {
            result = 'running';
            break;
          }
        }
      } else if (node.type === 'decorator') {
        if (node.label === 'Inverter') {
          const childResult = await executeNode(childrenIds[0]);
          result = childResult === 'success' ? 'failure' : childResult === 'failure' ? 'success' : 'running';
        } else {
           result = await executeNode(childrenIds[0]);
        }
      } else if (node.type === 'subtree') {
         if (childrenIds.length > 0) {
            result = await executeNode(childrenIds[0]);
         }
      }

      if (isPlayingRef.current) {
        updateNodeState(nodeId, result);
      }
      currentStack.pop();
      return result;
    };

    const runTree = async () => {
      // Reset all
      useStore.getState().nodes.forEach(n => updateNodeState(n.id, 'idle'));
      
      while (isPlayingRef.current) {
        await executeNode(root.id);
        if (!isPlayingRef.current) break;
        await new Promise(resolve => { timeoutId = setTimeout(resolve, 1500); });
        if (isPlayingRef.current) {
          useStore.getState().nodes.forEach(n => updateNodeState(n.id, 'idle'));
        }
      }
    };

    runTree();

    return () => {
      clearTimeout(timeoutId);
    };
  }, [isPlaying, isSimulating]);

  useEffect(() => {
    if (!isPlaying && !isSimulating) {
      console.log('已进入实机模式，等待接收真实行为树状态...');
    }
  }, [isPlaying, isSimulating]);

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
    </div>
  );
}

export default App;
