import { useCallback, useEffect, useMemo, useState } from 'react';
import { Background, Controls, Node, ReactFlow } from '@xyflow/react';
import '@xyflow/react/dist/style.css';
import { AlertCircle, CheckCircle2, Download, Save } from 'lucide-react';
import { useEditorStore } from '../store/useEditorStore';
import { useStore } from '../store/useStore';
import { EditorNodeComponent } from './EditorNode';
import { EditorPalette } from './EditorPalette';
import { EditorContextMenu } from './EditorContextMenu';

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
    toggleNodeCollapse,
    deleteNode,
    replaceNodeType,
    exportXml,
    activeTreeId,
    insertNode: insertNodeAt,
  } = useEditorStore();
  const [saveState, setSaveState] = useState<{ type: 'idle' | 'saving' | 'success' | 'error'; message: string }>({
    type: 'idle',
    message: '',
  });
  const [contextMenu, setContextMenu] = useState<{ nodeId: string; tagName: string; x: number; y: number } | null>(null);

  const nodesWithSelection = useMemo(
    () =>
      flowNodes.map((node) => ({
        ...node,
        data: {
          ...node.data,
          selected: node.id === selectedNodeId,
        },
      })),
    [flowNodes, selectedNodeId]
  );

  const saveXmlToSource = useCallback(async () => {
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
      setSaveState({
        type: 'error',
        message: error instanceof Error ? error.message : '保存失败',
      });
    }
  }, [activePhase, exportXml, replacePhaseXml]);

  const handleDownload = useCallback(() => {
    const xml = exportXml();
    if (!xml) {
      return;
    }

    const blob = new Blob([xml], { type: 'text/xml' });
    const url = URL.createObjectURL(blob);
    const link = window.document.createElement('a');
    link.href = url;
    link.download = activeTreeId ? `${activeTreeId}.xml` : 'behavior_tree.xml';
    window.document.body.appendChild(link);
    link.click();
    window.document.body.removeChild(link);
    URL.revokeObjectURL(url);
  }, [activeTreeId, exportXml]);

  const handleDrop = useCallback(
    (event: React.DragEvent<HTMLDivElement>) => {
      event.preventDefault();
      const tagName = event.dataTransfer.getData('application/x-bt-node-template');
      if (!tagName) {
        return;
      }

      const element = document.elementFromPoint(event.clientX, event.clientY) as HTMLElement | null;
      const slot = element?.closest('[data-drop-zone]') as HTMLElement | null;
      if (slot) {
        const nodeId = slot.dataset.nodeId;
        const mode = slot.dataset.dropZone as 'before' | 'after' | 'append_child' | undefined;
        if (nodeId && mode) {
          insertNodeAt(nodeId, mode, tagName);
          setContextMenu(null);
          return;
        }
      }

      const nodeCard = element?.closest('[data-editor-node-id]') as HTMLElement | null;
      if (nodeCard?.dataset.editorNodeId) {
        insertNodeAt(nodeCard.dataset.editorNodeId, 'append_child', tagName);
        setContextMenu(null);
        return;
      }

      if (selectedNodeId) {
        insertNodeAt(selectedNodeId, 'append_child', tagName);
      }
    },
    [insertNodeAt, selectedNodeId]
  );

  const handleKeydown = useCallback(
    (event: KeyboardEvent) => {
      if ((event.metaKey || event.ctrlKey) && event.key.toLowerCase() === 's') {
        event.preventDefault();
        void saveXmlToSource();
        return;
      }

      if (!selectedNodeId) {
        return;
      }

      if (event.key === 'Delete' || event.key === 'Backspace') {
        event.preventDefault();
        deleteNode(selectedNodeId);
        return;
      }

      if (event.key === ' ') {
        event.preventDefault();
        toggleNodeCollapse(selectedNodeId);
        return;
      }

      if (event.key.toLowerCase() === 't') {
        event.preventDefault();
        useEditorStore.getState().cycleCompositeType(selectedNodeId);
        return;
      }

      if (event.key.toLowerCase() === 'a' && event.shiftKey) {
        event.preventDefault();
        insertNodeAt(selectedNodeId, 'after', 'Sequence');
        return;
      }

      if (event.key.toLowerCase() === 'a') {
        event.preventDefault();
        insertNodeAt(selectedNodeId, 'append_child', 'Sequence');
        return;
      }
    },
    [deleteNode, insertNodeAt, saveXmlToSource, selectedNodeId, toggleNodeCollapse]
  );

  useEffect(() => {
    window.addEventListener('keydown', handleKeydown);
    return () => window.removeEventListener('keydown', handleKeydown);
  }, [handleKeydown]);

  return (
    <div
      onDragOver={(event) => event.preventDefault()}
      onDrop={handleDrop}
      className="relative h-full w-full overflow-hidden rounded-[28px] border border-white/50 bg-white/45"
      data-testid="editor-canvas"
    >
      <EditorPalette />

      <div className="absolute left-[352px] top-4 z-20 flex gap-2">
        <button
          type="button"
          onClick={() => void saveXmlToSource()}
          disabled={saveState.type === 'saving'}
          data-testid="save-source-button"
          className="flex items-center gap-1.5 rounded-xl bg-emerald-600 px-4 py-2 text-sm font-semibold text-white shadow transition-colors hover:bg-emerald-700 disabled:bg-emerald-400"
        >
          <Save className="h-4 w-4" />
          {saveState.type === 'saving' ? '保存中' : '保存到源文件'}
        </button>
        <button
          type="button"
          onClick={handleDownload}
          data-testid="download-source-button"
          className="flex items-center gap-1.5 rounded-xl bg-sky-600 px-4 py-2 text-sm font-semibold text-white shadow transition-colors hover:bg-sky-700"
        >
          <Download className="h-4 w-4" />
          导出源文件
        </button>
        <div className="rounded-xl bg-white/90 px-3 py-2 text-xs font-semibold text-slate-500 shadow-sm">
          快捷键：删除键删除，空格键折叠，T 键切换复合节点，A 键添加子节点，Shift+A 后插，Ctrl/Cmd+S 保存
        </div>
      </div>

      {saveState.type !== 'idle' && (
        <div
          data-testid="save-state-banner"
          className={`absolute right-4 top-4 z-20 flex items-center gap-2 rounded-xl px-4 py-3 text-sm shadow ${
            saveState.type === 'error'
              ? 'bg-rose-100 text-rose-700'
              : saveState.type === 'success'
                ? 'bg-emerald-100 text-emerald-700'
                : 'bg-amber-100 text-amber-700'
          }`}
        >
          {saveState.type === 'error' ? <AlertCircle className="h-4 w-4" /> : <CheckCircle2 className="h-4 w-4" />}
          <span>{saveState.message}</span>
        </div>
      )}

      <ReactFlow
        nodes={nodesWithSelection}
        edges={flowEdges}
        nodeTypes={nodeTypes}
        onNodeClick={(_, node: Node) => {
          setSelectedNode(node.id);
          setContextMenu(null);
        }}
        onNodeDoubleClick={(_, node: Node) => {
          toggleNodeCollapse(node.id);
        }}
        onNodeContextMenu={(event, node) => {
          event.preventDefault();
          setSelectedNode(node.id);
          setContextMenu({
            nodeId: node.id,
            tagName: String((node.data as { tagName?: string }).tagName ?? ''),
            x: event.clientX - 24,
            y: event.clientY - 24,
          });
        }}
        onPaneClick={() => {
          setSelectedNode(null);
          setContextMenu(null);
        }}
        fitView
        nodesDraggable={false}
        nodesConnectable={false}
        elementsSelectable
        minZoom={0.1}
        className="bg-transparent"
      >
        <Background color="#cbd5e1" gap={24} size={2} className="opacity-50" />
        <Controls className="!border-slate-200 !bg-white/80 backdrop-blur-sm" />
      </ReactFlow>

      {contextMenu && (
        <EditorContextMenu
          nodeId={contextMenu.nodeId}
          tagName={contextMenu.tagName}
          x={contextMenu.x}
          y={contextMenu.y}
          onClose={() => setContextMenu(null)}
          onToggleCollapse={toggleNodeCollapse}
          onDelete={deleteNode}
          onReplace={replaceNodeType}
          onWrapInverter={(nodeId) => insertNodeAt(nodeId, 'wrap', 'Inverter')}
          onWrapRetry={(nodeId) => insertNodeAt(nodeId, 'wrap', 'RetryUntilSuccessful')}
        />
      )}
    </div>
  );
};
