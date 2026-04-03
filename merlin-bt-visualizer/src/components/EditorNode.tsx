import { Handle, Position } from '@xyflow/react';
import { motion } from 'framer-motion';
import { Boxes, CornerDownRight, GitBranch, Layers3, RefreshCcw, ShieldCheck, Sparkles } from 'lucide-react';
import { EditorFlowNodeData } from '../utils/editorProjection';
import { useEditorStore } from '../store/useEditorStore';

interface EditorNodeProps {
  data: EditorFlowNodeData;
  selected?: boolean;
}

const paletteStyles: Record<string, string> = {
  control: 'bg-sky-50 border-sky-300 text-sky-900 shadow-sky-200/70',
  decorator: 'bg-amber-50 border-amber-300 text-amber-900 shadow-amber-200/70',
  subtree: 'bg-violet-50 border-violet-300 text-violet-900 shadow-violet-200/70',
  leaf: 'bg-emerald-50 border-emerald-300 text-emerald-900 shadow-emerald-200/70',
};

const iconByType = {
  control: GitBranch,
  decorator: ShieldCheck,
  subtree: Layers3,
  leaf: Boxes,
};

const QuickSlot = ({
  nodeId,
  mode,
  label,
  className,
}: {
  nodeId: string;
  mode: 'before' | 'after' | 'append_child';
  label: string;
  className: string;
}) => (
  <div
    data-drop-zone={mode}
    data-node-id={nodeId}
    className={`absolute z-20 flex h-6 min-w-6 items-center justify-center rounded-full border border-dashed border-slate-300 bg-white/95 px-2 text-[10px] font-semibold text-slate-500 shadow-sm ${className}`}
    title={`拖拽节点到这里${label}`}
  >
    {label}
  </div>
);

export const EditorNodeComponent = ({ data }: EditorNodeProps) => {
  const toggleNodeCollapse = useEditorStore((state) => state.toggleNodeCollapse);
  const cycleCompositeType = useEditorStore((state) => state.cycleCompositeType);

  const Icon = iconByType[data.uiType] ?? Boxes;
  const shellClass = paletteStyles[data.uiType] ?? paletteStyles.leaf;

  return (
    <div
      data-editor-node-id={data.editorNodeId}
      data-testid="editor-node-card"
      className="group relative"
    >
      {!data.isRoot && (
        <QuickSlot
          nodeId={data.editorNodeId}
          mode="before"
          label="前"
          className="-top-3 left-1/2 -translate-x-1/2 opacity-0 transition-opacity group-hover:opacity-100"
        />
      )}

      {!data.isRoot && (
        <QuickSlot
          nodeId={data.editorNodeId}
          mode="after"
          label="后"
          className="-bottom-3 left-1/2 -translate-x-1/2 opacity-0 transition-opacity group-hover:opacity-100"
        />
      )}

      {data.canAcceptChildren && (
        <QuickSlot
          nodeId={data.editorNodeId}
          mode="append_child"
          label="子"
          className="right-[-18px] top-1/2 -translate-y-1/2 opacity-0 transition-opacity group-hover:opacity-100"
        />
      )}

      <motion.div
        whileHover={{ scale: 1.02 }}
        className={`relative min-w-[220px] rounded-2xl border-2 px-4 py-3 shadow-lg transition-all ${shellClass} ${
          data.selected ? 'ring-4 ring-slate-300/60' : ''
        }`}
      >
        {!data.isRoot && <Handle type="target" position={Position.Left} className="opacity-0" />}

        <div className="mb-2 flex items-start justify-between gap-3">
          <div className="flex min-w-0 flex-1 items-center gap-2">
            <div className="rounded-xl bg-white/70 p-2 shadow-sm">
              <Icon className="h-4 w-4" />
            </div>
            <div className="min-w-0 flex-1">
              <div className="truncate text-sm font-bold" title={String(data.displayLabel)}>
                {String(data.displayLabel)}
              </div>
              <div className="mt-1 flex flex-wrap items-center gap-1">
                <span className="rounded-full bg-white/80 px-2 py-0.5 text-[10px] font-semibold text-slate-600">
                  {String(data.categoryLabel)}
                </span>
                <span className="rounded-full bg-slate-900/5 px-2 py-0.5 text-[10px] font-semibold text-slate-500">
                  {String(data.sourceLabel)}
                </span>
              </div>
            </div>
          </div>

          <div className="flex items-center gap-1">
            {data.switchCandidates.length > 0 && (
              <button
                type="button"
                onClick={(event) => {
                  event.stopPropagation();
                  cycleCompositeType(data.editorNodeId);
                }}
                className="rounded-lg border border-white/70 bg-white/80 p-1.5 text-slate-500 transition-colors hover:text-sky-700"
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
                className="rounded-lg border border-white/70 bg-white/80 p-1.5 text-slate-500 transition-colors hover:text-slate-700"
                title="折叠或展开子节点"
              >
                <CornerDownRight className="h-3.5 w-3.5" />
              </button>
            )}
          </div>
        </div>

        <div className="line-clamp-2 text-xs leading-5 text-slate-600" title={String(data.displayDesc)}>
          {String(data.attributeSummary || data.displayDesc)}
        </div>

        {data.switchCandidates.length > 0 && (
          <div className="mt-2 flex items-center gap-1 text-[10px] font-semibold text-slate-500">
            <Sparkles className="h-3 w-3" />
            <span>可一键切换为：{data.switchCandidates.join('、')}</span>
          </div>
        )}

        <Handle type="source" position={Position.Right} className="opacity-0" />
      </motion.div>
    </div>
  );
};
