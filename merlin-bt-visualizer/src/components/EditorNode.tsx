import { Handle, Position } from '@xyflow/react';
import { motion } from 'framer-motion';
import { EditorFlowNodeData } from '../utils/editorProjection';
import { GitBranch, Box, Puzzle, AlertCircle, Settings } from 'lucide-react';

interface EditorNodeProps {
  data: EditorFlowNodeData;
  selected?: boolean;
}

export const EditorNodeComponent = ({ data }: EditorNodeProps) => {
  const isControl = data.uiType === 'control';
  const isDecorator = data.uiType === 'decorator';
  const isLeaf = data.uiType === 'leaf';
  const isSubtree = data.uiType === 'subtree';
  const selected = data.selected as boolean;

  let Icon = Box;
  if (isControl) Icon = GitBranch;
  if (isSubtree) Icon = Puzzle;
  if (isDecorator) Icon = AlertCircle;
  if (isLeaf) Icon = Settings;

  let bgClass = 'bg-white';
  let borderClass = selected ? 'border-blue-500 shadow-blue-500/30' : 'border-slate-200';
  let textClass = 'text-slate-700';
  let iconClass = 'text-slate-400';

  if (isControl) {
    bgClass = 'bg-blue-50';
    borderClass = selected ? 'border-blue-600 shadow-blue-600/30 shadow-md ring-2 ring-blue-600/20' : 'border-blue-200';
    textClass = 'text-blue-800';
    iconClass = 'text-blue-500';
  } else if (isDecorator) {
    bgClass = 'bg-amber-50';
    borderClass = selected ? 'border-amber-600 shadow-amber-600/30 shadow-md ring-2 ring-amber-600/20' : 'border-amber-200';
    textClass = 'text-amber-800';
    iconClass = 'text-amber-500';
  } else if (isSubtree) {
    bgClass = 'bg-purple-50';
    borderClass = selected ? 'border-purple-600 shadow-purple-600/30 shadow-md ring-2 ring-purple-600/20' : 'border-purple-200';
    textClass = 'text-purple-800';
    iconClass = 'text-purple-500';
  } else {
    // Leaf action
    bgClass = 'bg-emerald-50';
    borderClass = selected ? 'border-emerald-600 shadow-emerald-600/30 shadow-md ring-2 ring-emerald-600/20' : 'border-emerald-200';
    textClass = 'text-emerald-800';
    iconClass = 'text-emerald-500';
  }

  // Format attributes for display
  const attrEntries = Object.entries(data.attributes);
  const displayAttrs = attrEntries.slice(0, 2).map(([k, v]) => `${k}="${v}"`).join(' ');
  const hasMoreAttrs = attrEntries.length > 2;

  return (
    <div className={`relative rounded-xl border-2 transition-all duration-200 ${bgClass} ${borderClass}`}>
      <motion.div
        whileHover={{ scale: 1.02 }}
        className="px-4 py-3 min-w-[140px] flex flex-col gap-2"
      >
        {/* Only show target handle if it's not the root node */}
        {!data.isRoot && <Handle type="target" position={Position.Left} className="opacity-0" />}

        <div className="flex items-center gap-2">
          <div className={`p-1.5 rounded-md bg-white/60 ${iconClass}`}>
            <Icon className="w-4 h-4" />
          </div>
          <div className={`flex-1 font-bold text-sm truncate ${textClass}`} title={data.tagName}>
            {data.tagName}
          </div>
        </div>

        {attrEntries.length > 0 && (
          <div className="text-xs font-mono text-slate-500 truncate" title={Object.entries(data.attributes).map(([k,v]) => `${k}="${v}"`).join(' ')}>
            {displayAttrs} {hasMoreAttrs && '...'}
          </div>
        )}

        <Handle type="source" position={Position.Right} className="opacity-0" />
      </motion.div>
    </div>
  );
};
