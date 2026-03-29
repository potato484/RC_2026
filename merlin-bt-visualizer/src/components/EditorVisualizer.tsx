import { useCallback, useEffect } from 'react';
import { ReactFlow, Background, Node, useNodesState, useEdgesState, Controls } from '@xyflow/react';
import '@xyflow/react/dist/style.css';
import { useEditorStore } from '../store/useEditorStore';
import { EditorNodeComponent } from './EditorNode';
import { Download } from 'lucide-react';

const nodeTypes = {
  editorNode: EditorNodeComponent,
};

export const EditorVisualizer = () => {
  const { 
    flowNodes, 
    flowEdges, 
    selectedNodeId,
    setSelectedNode, 
    exportXml,
    activeTreeId
  } = useEditorStore();

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

  return (
    <div className="w-full h-full relative glass-panel overflow-hidden bg-white/40">
      <div className="absolute top-4 left-4 z-10 flex gap-2">
        <button 
          onClick={handleDownload}
          className="px-3 py-1.5 bg-blue-600 hover:bg-blue-700 text-white rounded-md shadow flex items-center gap-1.5 text-sm font-medium transition-colors"
        >
          <Download className="w-4 h-4" />
          导出 XML
        </button>
      </div>

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
