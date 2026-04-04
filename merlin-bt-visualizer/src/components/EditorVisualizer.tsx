import { useCallback, useEffect, useMemo, useState } from 'react';
import { Background, Controls, Edge, Node, ReactFlow } from '@xyflow/react';
import '@xyflow/react/dist/style.css';
import {
  AlertCircle,
  CheckCircle2,
  Download,
  LibraryBig,
  Plus,
  Redo2,
  Save,
  Undo2,
} from 'lucide-react';
import { useEditorStore } from '../store/useEditorStore';
import { useStore } from '../store/useStore';
import { CANVAS_BACKGROUND } from '../utils/btCanvasTheme';
import { EditorAlongBranchInsertRequest, EditorInsertTemplate } from '../types/editor';
import { EditorNodeComponent } from './EditorNode';
import { EditorContextMenu } from './EditorContextMenu';
import { EditorInsertEdge } from './EditorInsertEdge';
import { EditorInsertMenu } from './EditorInsertMenu';
import { EditorKnowledgeBase } from './EditorKnowledgeBase';

const nodeTypes = {
  editorNode: EditorNodeComponent,
};

const edgeTypes = {
  editorInsertEdge: EditorInsertEdge,
};

interface DraggedAlongBranchInsertPayload {
  template: EditorInsertTemplate;
  wrapperTagName: string;
}

function parseDraggedInsertRequest(event: React.DragEvent<HTMLDivElement>): DraggedAlongBranchInsertPayload | null {
  const requestPayload = event.dataTransfer.getData('application/x-bt-along-branch-insert');
  if (requestPayload) {
    try {
      const parsed = JSON.parse(requestPayload) as DraggedAlongBranchInsertPayload;
      if (parsed?.template?.tagName && parsed.wrapperTagName) {
        return parsed;
      }
    } catch {
      return null;
    }
  }

  const payload = event.dataTransfer.getData('application/x-bt-node-template');
  if (!payload) {
    return null;
  }

  try {
    const parsed = JSON.parse(payload) as EditorInsertTemplate;
    if (parsed?.tagName) {
      return {
        template: parsed,
        wrapperTagName: '',
      };
    }
  } catch {
    return {
      template: { tagName: payload },
      wrapperTagName: '',
    };
  }

  return null;
}

export const EditorVisualizer = () => {
  const activePhase = useStore((state) => state.activePhase);
  const replacePhaseXml = useStore((state) => state.replacePhaseXml);
  const {
    document: editorDocument,
    flowNodes,
    flowEdges,
    selectedNodeId,
    setSelectedNode,
    toggleNodeCollapse,
    deleteNode,
    replaceNodeType,
    exportXml,
    activeTreeId,
    insertAlongBranch,
    insertAlongBranchOnEdge,
    wrapNode,
    undo,
    redo,
    canUndo,
    canRedo,
  } = useEditorStore();
  const [saveState, setSaveState] = useState<{ type: 'idle' | 'saving' | 'success' | 'error'; message: string }>({
    type: 'idle',
    message: '',
  });
  const [contextMenu, setContextMenu] = useState<{ nodeId: string; tagName: string; x: number; y: number } | null>(null);
  const [activeEdgeMenuId, setActiveEdgeMenuId] = useState<string | null>(null);
  const [knowledgeBaseOpen, setKnowledgeBaseOpen] = useState(false);
  const [toolbarInsertOpen, setToolbarInsertOpen] = useState(false);
  const [branchInsertDialog, setBranchInsertDialog] = useState<{
    nodeId: string;
    position: 'before' | 'after';
  } | null>(null);

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

  const edgesWithActions = useMemo(
    () =>
      flowEdges.map((edge) => ({
        ...edge,
        data: {
          ...(edge.data as Record<string, unknown> | undefined),
          isMenuOpen: activeEdgeMenuId === edge.id,
          onToggleMenu: (edgeId: string | null) => {
            setContextMenu(null);
            setBranchInsertDialog(null);
            setKnowledgeBaseOpen(false);
            setToolbarInsertOpen(false);
            setActiveEdgeMenuId(edgeId);
          },
          onInsertTemplate: (parentNodeId: string, childNodeId: string, request: EditorAlongBranchInsertRequest) => {
            insertAlongBranchOnEdge(parentNodeId, childNodeId, request);
            setActiveEdgeMenuId(null);
          },
        },
      })) as Edge[],
    [activeEdgeMenuId, flowEdges, insertAlongBranchOnEdge]
  );

  const openKnowledgeBase = useCallback(() => {
    setContextMenu(null);
    setActiveEdgeMenuId(null);
    setBranchInsertDialog(null);
    setToolbarInsertOpen(false);
    setKnowledgeBaseOpen(true);
  }, []);

  const openToolbarInsert = useCallback(() => {
    if (!selectedNodeId) {
      return;
    }

    setContextMenu(null);
    setActiveEdgeMenuId(null);
    setBranchInsertDialog(null);
    setKnowledgeBaseOpen(false);
    setToolbarInsertOpen(true);
  }, [selectedNodeId]);

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
      const requestPayload = parseDraggedInsertRequest(event);
      if (!requestPayload?.template?.tagName || !requestPayload.wrapperTagName) {
        return;
      }

      const element = window.document.elementFromPoint(event.clientX, event.clientY) as HTMLElement | null;
      const slot = element?.closest('[data-drop-zone]') as HTMLElement | null;
      if (slot) {
        const nodeId = slot.dataset.nodeId;
        const mode = slot.dataset.dropZone as 'before' | 'after' | undefined;
        if (nodeId && mode) {
          insertAlongBranch(nodeId, {
            position: mode,
            wrapperTagName: requestPayload.wrapperTagName,
            template: requestPayload.template,
          });
          setContextMenu(null);
          setActiveEdgeMenuId(null);
          setBranchInsertDialog(null);
          setKnowledgeBaseOpen(false);
          setToolbarInsertOpen(false);
          return;
        }
      }

      const edgeTrigger = element?.closest('[data-edge-source][data-edge-target]') as HTMLElement | null;
      if (edgeTrigger?.dataset.edgeSource && edgeTrigger.dataset.edgeTarget) {
        insertAlongBranchOnEdge(edgeTrigger.dataset.edgeSource, edgeTrigger.dataset.edgeTarget, {
          position: 'before',
          wrapperTagName: requestPayload.wrapperTagName,
          template: requestPayload.template,
        });
        setContextMenu(null);
        setActiveEdgeMenuId(null);
        setBranchInsertDialog(null);
        setKnowledgeBaseOpen(false);
        setToolbarInsertOpen(false);
        return;
      }

      const nodeCard = element?.closest('[data-editor-node-id]') as HTMLElement | null;
      if (nodeCard?.dataset.editorNodeId) {
        insertAlongBranch(nodeCard.dataset.editorNodeId, {
          position: 'after',
          wrapperTagName: requestPayload.wrapperTagName,
          template: requestPayload.template,
        });
        setContextMenu(null);
        setActiveEdgeMenuId(null);
        setBranchInsertDialog(null);
        setKnowledgeBaseOpen(false);
        setToolbarInsertOpen(false);
        return;
      }

      if (selectedNodeId) {
        insertAlongBranch(selectedNodeId, {
          position: 'after',
          wrapperTagName: requestPayload.wrapperTagName,
          template: requestPayload.template,
        });
        setKnowledgeBaseOpen(false);
        setToolbarInsertOpen(false);
      }
    },
    [insertAlongBranch, insertAlongBranchOnEdge, selectedNodeId]
  );

  const handleKeydown = useCallback(
    (event: KeyboardEvent) => {
      const isMetaKey = event.metaKey || event.ctrlKey;
      if (isMetaKey && event.key.toLowerCase() === 's') {
        event.preventDefault();
        void saveXmlToSource();
        return;
      }

      if (isMetaKey && event.key.toLowerCase() === 'z' && event.shiftKey) {
        event.preventDefault();
        redo();
        return;
      }

      if (isMetaKey && event.key.toLowerCase() === 'y') {
        event.preventDefault();
        redo();
        return;
      }

      if (isMetaKey && event.key.toLowerCase() === 'z') {
        event.preventDefault();
        undo();
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
        setContextMenu(null);
        setActiveEdgeMenuId(null);
        setKnowledgeBaseOpen(false);
        setToolbarInsertOpen(false);
        setBranchInsertDialog({ nodeId: selectedNodeId, position: 'before' });
        return;
      }

      if (event.key.toLowerCase() === 'a') {
        event.preventDefault();
        setContextMenu(null);
        setActiveEdgeMenuId(null);
        setKnowledgeBaseOpen(false);
        setToolbarInsertOpen(false);
        setBranchInsertDialog({ nodeId: selectedNodeId, position: 'after' });
      }
    },
    [deleteNode, redo, saveXmlToSource, selectedNodeId, toggleNodeCollapse, undo]
  );

  useEffect(() => {
    window.addEventListener('keydown', handleKeydown);
    return () => window.removeEventListener('keydown', handleKeydown);
  }, [handleKeydown]);

  const desktopToolbar = (
    <div className="absolute left-4 right-4 top-4 z-20 hidden items-start justify-between gap-4 lg:flex">
      <div className="flex flex-wrap items-center gap-2">
        <button
          type="button"
          onClick={openKnowledgeBase}
          data-testid="editor-knowledge-base-button"
          className="flex items-center gap-1.5 rounded-xl bg-white/92 px-3 py-2 text-sm font-semibold text-slate-700 shadow transition hover:bg-white"
        >
          <LibraryBig className="h-4 w-4" />
          节点知识库
        </button>
        <button
          type="button"
          onClick={openToolbarInsert}
          disabled={!selectedNodeId}
          data-testid="editor-toolbar-insert-button"
          className="flex items-center gap-1.5 rounded-xl bg-violet-600 px-3 py-2 text-sm font-semibold text-white shadow transition hover:bg-violet-700 disabled:cursor-not-allowed disabled:bg-violet-300"
          title={selectedNodeId ? '向当前选中节点前后插入新节点' : '先在画布中选中一个节点'}
        >
          <Plus className="h-4 w-4" />
          插入节点
        </button>
        <button
          type="button"
          onClick={undo}
          disabled={!canUndo}
          className="flex items-center gap-1.5 rounded-xl bg-white/92 px-3 py-2 text-sm font-semibold text-slate-700 shadow transition hover:bg-white disabled:cursor-not-allowed disabled:opacity-40"
        >
          <Undo2 className="h-4 w-4" />
          撤销
        </button>
        <button
          type="button"
          onClick={redo}
          disabled={!canRedo}
          className="flex items-center gap-1.5 rounded-xl bg-white/92 px-3 py-2 text-sm font-semibold text-slate-700 shadow transition hover:bg-white disabled:cursor-not-allowed disabled:opacity-40"
        >
          <Redo2 className="h-4 w-4" />
          重做
        </button>
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
      </div>

      <div className="max-w-[360px] rounded-2xl bg-white/92 px-4 py-3 text-xs font-semibold leading-5 text-slate-500 shadow">
        “节点知识库”只负责查说明，“插入节点”负责改结构。工具栏插入必须先选中目标节点，再显式选择前插或后插；边中点和 A/Shift+A 仍保留快捷入口。
      </div>
    </div>
  );

  const mobileToolbar = (
    <div className="absolute inset-x-3 bottom-3 z-20 flex items-center justify-between gap-2 rounded-[22px] border border-white/70 bg-white/94 px-3 py-2 shadow-2xl backdrop-blur lg:hidden">
      <div className="flex items-center gap-2">
        <button
          type="button"
          onClick={openKnowledgeBase}
          data-testid="editor-mobile-knowledge-base-button"
          className="flex items-center gap-1.5 rounded-xl bg-slate-100 px-3 py-2 text-sm font-semibold text-slate-700"
        >
          <LibraryBig className="h-4 w-4" />
          知识库
        </button>
        <button
          type="button"
          onClick={openToolbarInsert}
          disabled={!selectedNodeId}
          data-testid="editor-mobile-insert-button"
          className="flex items-center gap-1.5 rounded-xl bg-violet-600 px-3 py-2 text-sm font-semibold text-white disabled:bg-violet-300"
        >
          <Plus className="h-4 w-4" />
          插入
        </button>
      </div>

      <div className="flex items-center gap-1">
        <button
          type="button"
          onClick={undo}
          disabled={!canUndo}
          className="rounded-xl p-2 text-slate-600 transition hover:bg-slate-100 disabled:opacity-40"
          aria-label="撤销"
        >
          <Undo2 className="h-4 w-4" />
        </button>
        <button
          type="button"
          onClick={redo}
          disabled={!canRedo}
          className="rounded-xl p-2 text-slate-600 transition hover:bg-slate-100 disabled:opacity-40"
          aria-label="重做"
        >
          <Redo2 className="h-4 w-4" />
        </button>
        <button
          type="button"
          onClick={() => void saveXmlToSource()}
          className="rounded-xl bg-emerald-600 p-2 text-white shadow"
          aria-label="保存到源文件"
        >
          <Save className="h-4 w-4" />
        </button>
        <button
          type="button"
          onClick={handleDownload}
          className="rounded-xl bg-sky-600 p-2 text-white shadow"
          aria-label="导出源文件"
        >
          <Download className="h-4 w-4" />
        </button>
      </div>
    </div>
  );

  return (
    <div
      onDragOver={(event) => event.preventDefault()}
      onDrop={handleDrop}
      className="relative h-full w-full overflow-hidden rounded-[28px] border border-white/50 bg-white/45"
      data-testid="editor-canvas"
    >
      {desktopToolbar}
      {mobileToolbar}

      {saveState.type !== 'idle' && (
        <div
          data-testid="save-state-banner"
          className={`absolute right-4 top-4 z-30 flex items-center gap-2 rounded-xl px-4 py-3 text-sm shadow ${
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

      <div className="absolute inset-0 top-0 lg:bottom-0 lg:right-0 lg:top-20">
        <ReactFlow
          nodes={nodesWithSelection}
          edges={edgesWithActions}
          nodeTypes={nodeTypes}
          edgeTypes={edgeTypes}
          onNodeClick={(_, node: Node) => {
            setSelectedNode(node.id);
            setContextMenu(null);
            setActiveEdgeMenuId(null);
            setBranchInsertDialog(null);
          }}
          onNodeDoubleClick={(_, node: Node) => {
            toggleNodeCollapse(node.id);
          }}
          onNodeContextMenu={(event, node) => {
            event.preventDefault();
            setSelectedNode(node.id);
            setActiveEdgeMenuId(null);
            setBranchInsertDialog(null);
            setKnowledgeBaseOpen(false);
            setToolbarInsertOpen(false);
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
            setActiveEdgeMenuId(null);
            setBranchInsertDialog(null);
            setToolbarInsertOpen(false);
          }}
          fitView
          nodesDraggable={false}
          nodesConnectable={false}
          elementsSelectable
          minZoom={0.1}
          className="bg-transparent"
        >
          <Background {...CANVAS_BACKGROUND} className="opacity-50" />
          <Controls className="!border-slate-200 !bg-white/86 backdrop-blur-sm" />
        </ReactFlow>
      </div>

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
          onWrapInverter={(nodeId) => wrapNode(nodeId, 'Inverter')}
          onWrapRetry={(nodeId) => wrapNode(nodeId, 'RetryUntilSuccessful')}
        />
      )}

      {branchInsertDialog && (
        <div
          className="fixed inset-0 z-40 bg-slate-950/18 p-3"
          onClick={() => setBranchInsertDialog(null)}
        >
          <div
            className="absolute left-1/2 top-1/2 hidden -translate-x-1/2 -translate-y-1/2 lg:block"
            onClick={(event) => event.stopPropagation()}
          >
            <EditorInsertMenu
              document={editorDocument}
              activeTreeId={activeTreeId}
              mode="floating"
              position={branchInsertDialog.position}
              onInsert={(request) => {
                insertAlongBranch(branchInsertDialog.nodeId, request);
                setBranchInsertDialog(null);
              }}
              onClose={() => setBranchInsertDialog(null)}
            />
          </div>

          <div
            className="absolute inset-x-3 bottom-3 lg:hidden"
            onClick={(event) => event.stopPropagation()}
          >
            <EditorInsertMenu
              document={editorDocument}
              activeTreeId={activeTreeId}
              mode="sheet"
              position={branchInsertDialog.position}
              onInsert={(request) => {
                insertAlongBranch(branchInsertDialog.nodeId, request);
                setBranchInsertDialog(null);
              }}
              onClose={() => setBranchInsertDialog(null)}
            />
          </div>
        </div>
      )}

      {toolbarInsertOpen && (
        <div className="fixed inset-0 z-40 bg-slate-950/18 p-3" onClick={() => setToolbarInsertOpen(false)}>
          <div
            className="absolute left-1/2 top-1/2 hidden -translate-x-1/2 -translate-y-1/2 lg:block"
            onClick={(event) => event.stopPropagation()}
          >
            <EditorInsertMenu
              document={editorDocument}
              activeTreeId={activeTreeId}
              mode="floating"
              positionMode="choose"
              onInsert={(request) => {
                if (!selectedNodeId) {
                  return;
                }
                insertAlongBranch(selectedNodeId, request);
                setToolbarInsertOpen(false);
              }}
              onClose={() => setToolbarInsertOpen(false)}
            />
          </div>

          <div className="absolute inset-x-3 bottom-3 top-20 lg:hidden" onClick={(event) => event.stopPropagation()}>
            <EditorInsertMenu
              document={editorDocument}
              activeTreeId={activeTreeId}
              mode="sheet"
              positionMode="choose"
              onInsert={(request) => {
                if (!selectedNodeId) {
                  return;
                }
                insertAlongBranch(selectedNodeId, request);
                setToolbarInsertOpen(false);
              }}
              onClose={() => setToolbarInsertOpen(false)}
            />
          </div>
        </div>
      )}

      {knowledgeBaseOpen && (
        <div
          className="absolute inset-0 z-40 overflow-hidden bg-slate-50/98 backdrop-blur-[2px]"
          data-testid="editor-knowledge-overlay"
        >
          <EditorKnowledgeBase
            className="h-full w-full"
            mode="workspace"
            onRequestClose={() => setKnowledgeBaseOpen(false)}
          />
        </div>
      )}
    </div>
  );
};
