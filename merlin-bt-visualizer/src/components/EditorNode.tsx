import { Handle, Position } from '@xyflow/react';
import { motion } from 'framer-motion';
import {
  ArrowRight,
  Boxes,
  ChevronDown,
  ChevronRight,
  Eye,
  FolderTree,
  GitBranch,
  RefreshCcw,
  RotateCw,
} from 'lucide-react';
import { EditorFlowNodeData } from '../utils/editorProjection';
import { useEditorStore } from '../store/useEditorStore';

interface EditorNodeProps {
  data: EditorFlowNodeData;
  selected?: boolean;
}

const bgColors: Record<string, string> = {
  control: 'bg-teal-50 text-teal-700 border-teal-200',
  decorator: 'bg-rose-50 text-rose-700 border-rose-200 border-dashed',
  condition: 'bg-slate-50 text-slate-700 border-slate-300 border-dashed',
  action: 'bg-white text-slate-700 border-slate-200',
  subtree: 'bg-indigo-50 text-indigo-700 border-indigo-200 border-dashed',
};

const iconMap = {
  control: GitBranch,
  decorator: RotateCw,
  condition: Eye,
  action: Boxes,
  subtree: FolderTree,
};

const QuickSlot = ({
  nodeId,
  mode,
  label,
  className,
  visible,
}: {
  nodeId: string;
  mode: 'before' | 'after' | 'append_child';
  label: string;
  className: string;
  visible: boolean;
}) => (
  <div
    data-drop-zone={mode}
    data-node-id={nodeId}
    className={`absolute z-20 flex h-6 min-w-6 items-center justify-center rounded-full border border-dashed border-slate-300 bg-white/95 px-2 text-[10px] font-semibold text-slate-500 shadow-sm transition-opacity ${
      visible ? 'opacity-100' : 'opacity-0'
    } ${className}`}
    title={`拖拽节点到这里${label}`}
  >
    {label}
  </div>
);

export const EditorNodeComponent = ({ data }: EditorNodeProps) => {
  const toggleNodeCollapse = useEditorStore((state) => state.toggleNodeCollapse);
  const cycleCompositeType = useEditorStore((state) => state.cycleCompositeType);

  const visualType = data.nodeKind === 'control' ? 'control' : data.nodeKind;
  const Icon = iconMap[visualType] ?? Boxes;
  const baseBgColor = bgColors[visualType] ?? bgColors.action;
  const showQuickSlots = Boolean(data.selected);

  const isControlFlow = visualType === 'control';
  const isModifier = visualType === 'decorator';
  const isActionLike = visualType === 'action' || visualType === 'condition' || visualType === 'subtree';

  let sizeClass = '';
  let shapeClass = '';

  if (isControlFlow) {
    sizeClass = 'w-[140px] h-[48px]';
    shapeClass = 'rounded-full';
  } else if (isModifier) {
    sizeClass = 'w-[180px] h-[48px]';
    shapeClass = 'rounded-xl';
  } else {
    sizeClass = 'w-[240px] min-h-[80px]';
    shapeClass = 'rounded-xl';
  }

  return (
    <div data-editor-node-id={data.editorNodeId} data-testid="editor-node-card" className="group relative">
      {!data.isRoot && (
        <QuickSlot
          nodeId={data.editorNodeId}
          mode="before"
          label="前"
          visible={showQuickSlots}
          className="-top-3 left-1/2 -translate-x-1/2 group-hover:opacity-100"
        />
      )}

      {!data.isRoot && (
        <QuickSlot
          nodeId={data.editorNodeId}
          mode="after"
          label="后"
          visible={showQuickSlots}
          className="-bottom-3 left-1/2 -translate-x-1/2 group-hover:opacity-100"
        />
      )}

      {data.canAcceptChildren && (
        <QuickSlot
          nodeId={data.editorNodeId}
          mode="append_child"
          label="子"
          visible={showQuickSlots}
          className="right-[-18px] top-1/2 -translate-y-1/2 group-hover:opacity-100"
        />
      )}

      <motion.div
        whileHover={{ scale: 1.01 }}
        className={`relative border-2 bg-white transition-all duration-200 ${
          data.selected
            ? 'ring-4 ring-sky-200 shadow-xl shadow-sky-100 border-sky-300'
            : 'hover:border-slate-300 shadow-sm'
        } ${sizeClass} ${shapeClass}`}
      >
        {!data.isRoot && <Handle type="target" position={Position.Left} className="opacity-0" />}

        <div className={`h-full w-full ${isActionLike ? 'p-3' : 'px-3 py-2'}`}>
          <div
            className={`flex h-full ${
              isActionLike ? 'items-start gap-2.5' : 'items-center justify-center gap-1.5'
            }`}
          >
            <div
              className={`flex-none rounded-lg flex items-center justify-center ${baseBgColor} ${
                isActionLike ? 'h-9 w-9 p-2' : 'h-6 w-6 rounded-full p-1'
              }`}
            >
              <Icon className="h-full w-full" />
            </div>

            <div className="min-w-0 flex-1">
              <div className={`flex items-center gap-1 ${isActionLike ? 'justify-between' : 'justify-center'}`}>
                <div
                  className={`truncate font-bold text-slate-800 ${
                    isActionLike ? 'text-sm text-left' : 'text-xs text-center'
                  }`}
                  title={String(data.displayLabel)}
                >
                  {String(data.displayLabel)}
                </div>

                <div className="flex items-center gap-1">
                  {data.switchCandidates.length > 0 && (
                    <button
                      type="button"
                      onClick={(event) => {
                        event.stopPropagation();
                        cycleCompositeType(data.editorNodeId);
                      }}
                      className={`rounded-lg p-1 text-slate-400 transition hover:bg-slate-100 hover:text-sky-700 ${
                        data.selected ? 'opacity-100' : 'opacity-0 group-hover:opacity-100'
                      }`}
                      title="一键切换复合节点类型"
                    >
                      <RefreshCcw className="h-3.5 w-3.5" />
                    </button>
                  )}

                  {data.hasChildren && (
                    <button
                      type="button"
                      onClick={(event) => {
                        event.stopPropagation();
                        toggleNodeCollapse(data.editorNodeId);
                      }}
                      className="rounded-lg bg-slate-100 p-0.5 text-slate-400 transition hover:bg-slate-200 hover:text-slate-700"
                      title="折叠或展开子节点"
                    >
                      {data.collapsed ? (
                        <ChevronRight className="h-3 w-3" />
                      ) : (
                        <ChevronDown className="h-3 w-3" />
                      )}
                    </button>
                  )}
                </div>
              </div>

              {isActionLike ? (
                <>
                  <div className="mt-1 line-clamp-2 text-[11px] leading-5 text-slate-500" title={String(data.baseDescription)}>
                    {data.collapsed ? '（已折叠）' : String(data.baseDescription)}
                  </div>
                  <div className="mt-2 flex flex-wrap items-center gap-1.5 text-[10px] font-semibold text-slate-400">
                    <span className="rounded-full bg-slate-100 px-2 py-0.5 text-slate-500">
                      {String(data.categoryLabel)}
                    </span>
                    <span className="rounded-full bg-white px-2 py-0.5 ring-1 ring-slate-200">
                      {String(data.tagName)}
                    </span>
                    {data.attributeSummary && (
                      <span className="truncate text-slate-400">{String(data.attributeSummary)}</span>
                    )}
                  </div>
                </>
              ) : (
                <div className="mt-1 flex items-center justify-center gap-1 text-[10px] font-semibold text-slate-400">
                  <ArrowRight className="h-3 w-3" />
                  <span className="truncate">{String(data.tagName)}</span>
                </div>
              )}
            </div>
          </div>
        </div>

        <Handle type="source" position={Position.Right} className="opacity-0" />
      </motion.div>
    </div>
  );
};
