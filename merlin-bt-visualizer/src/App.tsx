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
    const { nodes, edges, setActiveTree } = state;
    if (nodes.length === 0) return;

    let timeoutId: ReturnType<typeof setTimeout>;
    let currentStack: string[] = [];
    
    // Find root (node with no incoming edges) in current active tree
    const targets = new Set(edges.map(e => e.target));
    const root = nodes.find(n => !targets.has(n.id));
    
    if (!root) return;

    const executeNode = async (nodeId: string, currentTreeId: string): Promise<'success' | 'failure' | 'running'> => {
      if (!isPlayingRef.current) return 'failure';

      // We need to look up the node from the trees map because it might not be in the current active `nodes`
      const tree = useStore.getState().trees[currentTreeId];
      if (!tree) return 'failure';
      
      const node = tree.nodes.find(n => n.id === nodeId);
      if (!node) return 'failure';

      // Optionally auto-switch view to the tree currently executing (if you want the visualizer to follow execution)
      // if (useStore.getState().activeTreeId !== currentTreeId) {
      //   setActiveTree(currentTreeId);
      // }

      currentStack.push(nodeId);
      updateNodeState(nodeId, 'running');
      
      // Simulate delay for visualization
      await new Promise(resolve => { timeoutId = setTimeout(resolve, 800); });
      if (!isPlayingRef.current) return 'failure';

      const childrenEdges = tree.edges.filter(e => e.source === nodeId);
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
        
        // Simulate Script node updates to blackboard
        if (node.label.includes('执行脚本') && node.desc.includes('=')) {
           const assignments = node.desc.match(/([a-zA-Z_0-9]+):=([^;\]]+)/g);
           if (assignments) {
              assignments.forEach(assignment => {
                 const [key, val] = assignment.split(':=');
                 updateBlackboard({ key: key.trim(), value: val.trim(), desc: `脚本更新: ${node.label}`, updatedAt: Date.now() });
              });
           }
        }
      } else if (node.type === 'condition') {
        result = Math.random() > 0.2 ? 'success' : 'failure';
      } else if (node.type === 'sequence') {
        for (const childId of childrenIds) {
          const childResult = await executeNode(childId, currentTreeId);
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
          const childResult = await executeNode(childId, currentTreeId);
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
          const childResult = await executeNode(childrenIds[0], currentTreeId);
          result = childResult === 'success' ? 'failure' : childResult === 'failure' ? 'success' : 'running';
        } else {
           result = await executeNode(childrenIds[0], currentTreeId);
        }
      } else if (node.type === 'subtree') {
         // Jump to subtree
         const targetTreeId = node.subTreeId;
         if (targetTreeId && useStore.getState().trees[targetTreeId]) {
            const targetTree = useStore.getState().trees[targetTreeId];
            const targetTreeEdges = targetTree.edges;
            const targetTargets = new Set(targetTreeEdges.map(e => e.target));
            const targetRoot = targetTree.nodes.find(n => !targetTargets.has(n.id));
            
            if (targetRoot) {
              // Optionally switch view to target tree while it executes
              const prevActive = useStore.getState().activeTreeId;
              setActiveTree(targetTreeId);
              
              result = await executeNode(targetRoot.id, targetTreeId);
              
              // Switch back
              if (isPlayingRef.current) {
                setActiveTree(prevActive);
              }
            }
         } else {
            // Fallback if subtree not found
            result = 'failure';
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
      useStore.getState().resetTreeState();
      
      const currentState = useStore.getState();
      const currentActiveTreeId = currentState.activeTreeId;
      const currentTreeEdges = currentState.trees[currentActiveTreeId]?.edges || [];
      const t = new Set(currentTreeEdges.map(e => e.target));
      const currentRoot = currentState.trees[currentActiveTreeId]?.nodes.find(n => !t.has(n.id));
      
      if (!currentRoot) return;

      while (isPlayingRef.current) {
        await executeNode(currentRoot.id, currentActiveTreeId);
        if (!isPlayingRef.current) break;
        await new Promise(resolve => { timeoutId = setTimeout(resolve, 1500); });
        if (isPlayingRef.current) {
          useStore.getState().resetTreeState();
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
      console.log('已进入实机模式，等待接收真实行为状态...');
    }
  }, [isPlaying, isSimulating]);

  return (
    <div className="w-screen h-screen p-4 flex flex-col relative overflow-hidden bg-slate-50">
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
