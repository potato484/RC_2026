import { useEffect, useRef, useState } from 'react';
import { FolderTree, PanelsTopLeft } from 'lucide-react';
import { Header } from './components/Header';
import { Sidebar } from './components/Sidebar';
import { TreeVisualizer } from './components/TreeVisualizer';
import { RightPanel } from './components/RightPanel';
import { EditorVisualizer } from './components/EditorVisualizer';
import { EditorRightPanel } from './components/EditorRightPanel';
import { useStore } from './store/useStore';
import { useEditorStore } from './store/useEditorStore';
import { getBehaviorTreeXmlForPhase } from './utils/behaviorTreeSources';

function App() {
  const { isPlaying, isSimulating, activePhase, updateNodeState, addTimelineEvent, updateBlackboard, appMode } = useStore();
  const ensureEditorPhaseLoaded = useEditorStore((state) => state.ensurePhaseLoaded);
  const isPlayingRef = useRef(isPlaying);
  const [mobileSidebarOpen, setMobileSidebarOpen] = useState(false);
  const [mobilePanelOpen, setMobilePanelOpen] = useState(false);
  
  useEffect(() => {
    isPlayingRef.current = isPlaying;
  }, [isPlaying]);

  useEffect(() => {
    if (!isPlaying || !isSimulating) return;

    const state = useStore.getState();
    const { nodes, edges } = state;
    if (nodes.length === 0) return;

    let isActive = true;
    let timeoutId: ReturnType<typeof setTimeout>;
    let currentStack: string[] = [];
    
    // Find root (node with no incoming edges) in current active tree
    const targets = new Set(edges.map(e => e.target));
    const root = nodes.find(n => !targets.has(n.id));
    
    if (!root) return;

    const executeNode = async (nodeId: string, currentTreeId: string): Promise<'success' | 'failure' | 'running'> => {
      if (!isPlayingRef.current || !isActive) return 'failure';

      // We need to look up the node from the trees map because it might not be in the current active `nodes`
      const tree = useStore.getState().trees[currentTreeId];
      const node = nodes.find(n => n.id === nodeId);
      if (!node) return 'failure';

      // Optionally auto-switch view to the tree currently executing (if you want the visualizer to follow execution)
      // if (useStore.getState().activeTreeId !== currentTreeId) {
      //   setActiveTree(currentTreeId);
      // }

      currentStack.push(nodeId);
      updateNodeState(nodeId, 'running');
      
      // Simulate delay for visualization
      await new Promise(resolve => { timeoutId = setTimeout(resolve, 800); });
      if (!isPlayingRef.current || !isActive) return 'failure';

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
        result = 'success'; // 模拟模式下默认全部动作成功
        if (node.label.includes('Nav')) {
          updateBlackboard({ key: 'location', value: node.label.replace('NavTo', ''), desc: '当前位置', updatedAt: Date.now() });
        }
        
        // Simulate Script node updates to blackboard
        if (node.label.includes('执行脚本') && node.desc.includes('=')) {
           const assignments = node.desc.match(/([a-zA-Z_0-9]+):=([^;\\]]+)/g);
           if (assignments) {
              assignments.forEach(assignment => {
                 const [key, val] = assignment.split(':=');
                 updateBlackboard({ key: key.trim(), value: val.trim(), desc: `脚本更新: ${node.label}`, updatedAt: Date.now() });
              });
           }
        }
      } else if (node.type === ('condition' as string)) {
        // 条件节点在模拟模式下默认成功，除非它是被 Inverter 包裹的阻塞检查
        result = 'success'; 
        if (node.label.includes('CheckR1Blocking') || node.label.includes('CheckExitCondition')) {
           result = 'failure'; // 模拟不阻塞/不退出，让树继续跑
        }
      } else if (node.type === 'sequence') {
        for (const childId of childrenIds) {
          const childResult = await executeNode(childId, currentTreeId);
          if (!isPlayingRef.current || !isActive) return 'failure';
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
          if (!isPlayingRef.current || !isActive) return 'failure';
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
        // Now decorators are mostly attached to nodes natively by parser, but just in case some are left standalone
        if (node.label === 'Inverter' || node.label.includes('条件取反')) {
          const childResult = await executeNode(childrenIds[0], currentTreeId);
          result = childResult === 'success' ? 'failure' : childResult === 'failure' ? 'success' : 'running';
        } else if (node.label.includes('RetryUntilSuccessful') || node.label.includes('重试') || node.label.includes('KeepRunningUntilFailure') || node.label.includes('循环执行')) {
           const childResult = await executeNode(childrenIds[0], currentTreeId);
           result = childResult === 'failure' ? 'success' : childResult;
        } else {
           result = await executeNode(childrenIds[0], currentTreeId);
        }
      } else if (node.type === 'subtree') {
         // Subtrees are already expanded by btParser into the current graph. 
         // Since it unfolds the root Sequence, we treat the SubTree node itself as a Sequence.
         for (const childId of childrenIds) {
           const childResult = await executeNode(childId, currentTreeId);
           if (!isPlayingRef.current || !isActive) return 'failure';
           if (childResult === 'failure') {
             result = 'failure';
             break;
           }
           if (childResult === 'running') {
             result = 'running';
             break;
           }
         }
      }

      // Check decorators attached to the node
      if (node.decorators && node.decorators.length > 0) {
        for (const dec of node.decorators) {
          if (dec.label === 'Inverter' || dec.label.includes('条件取反')) {
             result = result === 'success' ? 'failure' : result === 'failure' ? 'success' : 'running';
          } else if (dec.label.includes('RetryUntilSuccessful') || dec.label.includes('重试') || dec.label.includes('KeepRunningUntilFailure') || dec.label.includes('循环') || dec.label.includes('死循环')) {
             // 模拟模式下为了防卡死，重试直接视为成功或者至少不卡住抛出失败
             if (result === 'failure') result = 'success';
          } else if (dec.label === 'ForceSuccess' || dec.label.includes('始终成功') || dec.label.includes('必定成功')) {
             result = 'success';
          } else if (dec.label === 'ForceFailure' || dec.label.includes('必定失败')) {
             result = 'failure';
          }
        }
      }

      if (isPlayingRef.current && isActive) {
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

      if (isPlayingRef.current && isActive) {
        await executeNode(currentRoot.id, currentActiveTreeId);
        
        // Tree finished execution
        if (isActive) {
          addTimelineEvent({
            id: Date.now().toString() + Math.random(),
            time: new Date().toLocaleTimeString(),
            desc: `行为树执行完毕`,
            icon: 'scan',
            status: 'success'
          });
          useStore.getState().stopPlay();
        }
      }
    };

    runTree();

    return () => {
      isActive = false;
      clearTimeout(timeoutId);
    };
  }, [isPlaying, isSimulating, activePhase]);

  useEffect(() => {
    if (!isPlaying && !isSimulating) {
      console.log('已进入实机模式，等待接收真实行为状态...');
    }
  }, [isPlaying, isSimulating]);

  useEffect(() => {
    if (appMode !== 'editor') return;
    ensureEditorPhaseLoaded(activePhase, getBehaviorTreeXmlForPhase(activePhase));
  }, [activePhase, appMode, ensureEditorPhaseLoaded]);

  useEffect(() => {
    setMobileSidebarOpen(false);
    setMobilePanelOpen(false);
  }, [activePhase, appMode]);

  return (
    <div className="relative flex h-screen w-screen flex-col overflow-hidden bg-slate-50 p-3 lg:p-4">
      <Header />
      <div className="relative flex min-h-0 flex-1 overflow-hidden">
        <div className="hidden shrink-0 lg:block">
          <Sidebar />
        </div>

        <div className="relative min-h-0 flex-1">
          <div className="absolute left-3 top-3 z-30 flex items-center gap-2 lg:hidden">
            <button
              type="button"
              onClick={() => setMobileSidebarOpen(true)}
              className="flex items-center gap-2 rounded-2xl bg-white/92 px-3 py-2 text-sm font-semibold text-slate-700 shadow-lg backdrop-blur"
            >
              <FolderTree className="h-4 w-4" />
              决策树
            </button>
            <button
              type="button"
              onClick={() => setMobilePanelOpen(true)}
              className="flex items-center gap-2 rounded-2xl bg-white/92 px-3 py-2 text-sm font-semibold text-slate-700 shadow-lg backdrop-blur"
            >
              <PanelsTopLeft className="h-4 w-4" />
              {appMode === 'viewer' ? '详情' : '检查器'}
            </button>
          </div>

          {appMode === 'viewer' ? <TreeVisualizer /> : <EditorVisualizer />}
        </div>

        <div className="hidden shrink-0 lg:flex">
          {appMode === 'viewer' ? <RightPanel /> : <EditorRightPanel />}
        </div>
      </div>

      {mobileSidebarOpen && (
        <div className="fixed inset-0 z-40 bg-slate-950/20 p-3 lg:hidden" onClick={() => setMobileSidebarOpen(false)}>
          <div
            className="h-full w-[86vw] max-w-[18rem]"
            onClick={(event) => event.stopPropagation()}
          >
            <Sidebar />
          </div>
        </div>
      )}

      {mobilePanelOpen && (
        <div className="fixed inset-0 z-40 bg-slate-950/20 p-3 lg:hidden" onClick={() => setMobilePanelOpen(false)}>
          <div
            className="absolute inset-x-3 bottom-3 top-24"
            onClick={(event) => event.stopPropagation()}
          >
            {appMode === 'viewer' ? <RightPanel /> : <EditorRightPanel />}
          </div>
        </div>
      )}
    </div>
  );
}

export default App;
