import { useCallback, useEffect, useState } from 'react';
import { ReactFlow, Background, Node, useNodesState, useEdgesState, Controls } from '@xyflow/react';
import '@xyflow/react/dist/style.css';
import { useEditorStore } from '../store/useEditorStore';
import { EditorNodeComponent } from './EditorNode';
import { AlertCircle, CheckCircle2, Download, Save } from 'lucide-react';
import { useStore } from '../store/useStore';

const nodeTypes = {
  editorNode: EditorNodeComponent,
};

export const EditorVisualizer = () => {
  const activePhase = useStore((state) => state.activePhase);
  const replacePhaseXml = useStore((state) => state.replacePhaseXml);
  const { 
    flowNodes, 
    flowEdges, 
    selectedNodeId,
    setSelectedNode, 
    exportXml,
    activeTreeId
  } = useEditorStore();
  const [saveState, setSaveState] = useState<{ type: 'idle' | 'saving' | 'success' | 'error'; message: string }>({
    type: 'idle',
    message: '',
  });

  // Add selected state to nodes
  const nodesWithSelection = flowNodes.map(node => ({
    ...node,
    data: {
      ...node.data,
      selected: node.id === selectedNodeId
    }
  }));

  const [nodes, setNodes, onNodesChange] = useNodesState(nodesWithSelection);
  const [edges, setEdges, onEdgesChange] = useEdgesState(flowEdges);

  // Sync with store when they change
  useEffect(() => {
    setNodes(nodesWithSelection);
    setEdges(flowEdges);
  }, [flowNodes, flowEdges, selectedNodeId, setNodes, setEdges]);

  const onNodeClick = useCallback((_: React.MouseEvent, node: Node) => {
    setSelectedNode(node.id);
  }, [setSelectedNode]);

  const onPaneClick = useCallback(() => {
    setSelectedNode(null);
  }, [setSelectedNode]);

  const handleDownload = useCallback(() => {
    const xml = exportXml();
    if (!xml) return;
    
    const blob = new Blob([xml], { type: 'text/xml' });
    const url = URL.createObjectURL(blob);
    const a = window.document.createElement('a');
    a.href = url;
    a.download = activeTreeId ? `${activeTreeId}.xml` : 'behavior_tree.xml';
    window.document.body.appendChild(a);
    a.click();
    window.document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }, [exportXml, activeTreeId]);

  const handleSave = useCallback(async () => {
    const xml = exportXml();
    if (!xml) {
      setSaveState({ type: 'error', message: '当前没有可保存的源文件内容' });
      return;
    }

    if (!import.meta.env.DEV) {
      setSaveState({ type: 'error', message: '当前只在开发模式下支持写回源文件' });
      return;
    }

    setSaveState({ type: 'saving', message: '正在写回源文件...' });

    try {
      const response = await fetch('/api/editor/save-xml', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          phase: activePhase,
          xmlContent: xml,
        }),
      });

      const payload = await response.json().catch(() => null);
      if (!response.ok) {
        throw new Error(payload?.message || '保存失败');
      }

      replacePhaseXml(activePhase, xml);
      setSaveState({
        type: 'success',
        message: payload?.message || `已写回 ${activePhase} 对应的源文件`,
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : '保存失败';
      setSaveState({ type: 'error', message });
    }
  }, [activePhase, exportXml, replacePhaseXml]);

  return (
    <div className="w-full h-full relative glass-panel overflow-hidden bg-white/40" data-testid="editor-canvas">
      <div className="absolute top-4 left-4 z-10 flex gap-2">
        <button 
          onClick={handleSave}
          disabled={saveState.type === 'saving'}
          data-testid="save-source-button"
          className="px-3 py-1.5 bg-emerald-600 hover:bg-emerald-700 disabled:bg-emerald-400 text-white rounded-md shadow flex items-center gap-1.5 text-sm font-medium transition-colors"
          title={import.meta.env.DEV ? '把当前区域行为树写回工作区源文件' : '当前只在开发模式下支持写回源文件'}
        >
          <Save className="w-4 h-4" />
          {saveState.type === 'saving' ? '保存中' : '保存到源文件'}
        </button>
        <button 
          onClick={handleDownload}
          data-testid="download-source-button"
          className="px-3 py-1.5 bg-blue-600 hover:bg-blue-700 text-white rounded-md shadow flex items-center gap-1.5 text-sm font-medium transition-colors"
        >
          <Download className="w-4 h-4" />
          导出源文件
        </button>
      </div>

      {saveState.type !== 'idle' && (
        <div data-testid="save-state-banner" className={`absolute top-4 right-4 z-10 px-3 py-2 rounded-lg shadow text-sm flex items-center gap-2 ${
          saveState.type === 'error'
            ? 'bg-rose-100 text-rose-700'
            : saveState.type === 'success'
              ? 'bg-emerald-100 text-emerald-700'
              : 'bg-amber-100 text-amber-700'
        }`}>
          {saveState.type === 'error' ? <AlertCircle className="w-4 h-4" /> : <CheckCircle2 className="w-4 h-4" />}
          <span>{saveState.message}</span>
        </div>
      )}

      <ReactFlow
        nodes={nodes}
        edges={edges}
        nodeTypes={nodeTypes}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        onNodeClick={onNodeClick}
        onPaneClick={onPaneClick}
        fitView
        minZoom={0.1}
        className="bg-transparent"
      >
        <Background color="#cbd5e1" gap={24} size={2} className="opacity-50" />
        <Controls className="!bg-white/80 backdrop-blur-sm !border-slate-200" />
      </ReactFlow>
    </div>
  );
};
