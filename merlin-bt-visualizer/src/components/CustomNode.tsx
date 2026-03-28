import { Handle, Position } from '@xyflow/react';
import { motion } from 'framer-motion';
import { Brain, Eye, Footprints, Hand, CheckCircle2, XCircle, Loader2, LucideIcon, ArrowRight, CornerDownRight, RotateCw, GitBranch, TerminalSquare, AlertCircle, PlayCircle, Clock, ChevronDown, ChevronRight, ExternalLink } from 'lucide-react';
import { BTNode } from '../types';
import { useStore } from '../store/useStore';

const iconMap: Record<string, LucideIcon | Record<string, LucideIcon>> = {
  sequence: ArrowRight,
  selector: GitBranch,
  condition: Eye,
  action: TerminalSquare,
  decorator: RotateCw,
  subtree: CornerDownRight,
};

const bgColors: Record<string, string> = {
  sequence: 'bg-emerald-100 text-emerald-700 border-emerald-300',
  selector: 'bg-amber-100 text-amber-700 border-amber-300',
  condition: 'bg-blue-100 text-blue-700 border-blue-300',
  action: 'bg-purple-100 text-purple-700 border-purple-300',
  decorator: 'bg-rose-100 text-rose-700 border-rose-300',
  subtree: 'bg-indigo-100 text-indigo-700 border-indigo-300',
};

const typeTranslations: Record<string, string> = {
  sequence: '顺序',
  selector: '选择',
  condition: '条件',
  action: '动作',
  decorator: '装饰',
  subtree: '子树',
};

const stateColors: Record<string, string> = {
  idle: 'opacity-80',
  running: 'ring-4 ring-blue-400 shadow-xl shadow-blue-200',
  success: 'ring-2 ring-emerald-400 opacity-90',
  failure: 'ring-2 ring-red-400 opacity-90',
};

export const CustomNode = ({ data }: { data: BTNode }) => {
  const isRunning = data.state === 'running';
  const { setActiveTree, trees } = useStore();
  let Icon = (iconMap[data.type] as LucideIcon) || Brain;
  
  if (data.type === 'action') {
    if (data.label.includes('导航')) Icon = Footprints;
    else if (data.label.includes('抓取')) Icon = Hand;
    else if (data.label.includes('设置')) Icon = PlayCircle;
    else if (data.label.includes('延迟')) Icon = Clock;
  } else if (data.type === 'condition') {
    if (data.label.includes('检测') || data.label.includes('检查')) Icon = AlertCircle;
  }

  const bgColor = bgColors[data.type] || bgColors.action;
  const stateColor = stateColors[data.state] || stateColors.idle;
  
  // Show expand/collapse indicator for nodes that likely have children (but not subtrees since they are standalone now)
  const hasChildren = data.type === 'sequence' || data.type === 'selector' || data.type === 'decorator';
  const isSubTree = data.type === 'subtree' && data.subTreeId;
  const hasTargetTree = isSubTree && data.subTreeId && trees[data.subTreeId];

  const handleJumpToSubTree = (e: React.MouseEvent) => {
    e.stopPropagation();
    if (hasTargetTree && data.subTreeId) {
      setActiveTree(data.subTreeId);
    }
  };

  return (
    <motion.div
      animate={isRunning ? { scale: [1, 1.02, 1], transition: { repeat: Infinity, duration: 1.5 } } : { scale: 1 }}
      className={`relative px-3 py-3 rounded-xl border-2 flex items-center gap-2 transition-all duration-300 bg-white ${stateColor} w-[240px] h-[80px] group cursor-pointer hover:border-blue-400`}
    >
      <Handle type="target" position={Position.Top} className="opacity-0" />
      
      {/* Sibling Index Badge for execution order */}
      {data.siblingIndex !== undefined && data.siblingIndex > 0 && (
        <div className="absolute -left-2 -top-2 w-5 h-5 rounded-full bg-indigo-500 text-white flex items-center justify-center text-[10px] font-bold shadow-sm z-10">
          {data.siblingIndex}
        </div>
      )}

      <div className={`p-2 rounded-lg ${bgColor} relative flex-none`}>
        {isRunning ? <Loader2 className="w-5 h-5 animate-spin" /> : <Icon className="w-5 h-5" />}
      </div>
      
      <div className="flex-1 overflow-hidden flex flex-col justify-center">
        <div className="flex items-center justify-between gap-1">
          <div className="flex items-center gap-1 overflow-hidden">
            <div className="text-sm font-bold truncate" title={data.label}>{data.label}</div>
            {isSubTree && (
              <button 
                onClick={handleJumpToSubTree}
                className={`flex-none rounded p-0.5 transition-colors ${hasTargetTree ? 'text-indigo-500 hover:text-indigo-700 bg-indigo-50 hover:bg-indigo-100' : 'text-slate-400 bg-slate-50 cursor-not-allowed'}`}
                title={hasTargetTree ? `进入子树: ${data.subTreeId}` : `子树未找到: ${data.subTreeId}`}
                disabled={!hasTargetTree}
              >
                <ExternalLink className="w-3.5 h-3.5" />
              </button>
            )}
          </div>
          {hasChildren && (
            <div className="text-slate-400 bg-slate-100 rounded p-0.5 flex-none" title="双击折叠/展开">
              {data.collapsed ? <ChevronRight className="w-3 h-3" /> : <ChevronDown className="w-3 h-3" />}
            </div>
          )}
        </div>
        <div className="text-[11px] text-gray-500 font-mono mt-0.5 truncate" title={data.desc}>
          {data.collapsed ? '（已折叠）' : data.desc}
        </div>
        
        {/* Type Badge */}
        <div className="absolute -top-2.5 right-3 text-[10px] font-bold px-2 py-0.5 rounded-full bg-slate-800 text-white shadow-sm flex items-center gap-1">
          {typeTranslations[data.type] || data.type}
        </div>
      </div>

      {/* Tooltip for scripts / details on hover */}
      <div className="absolute left-1/2 -translate-x-1/2 bottom-full mb-2 hidden group-hover:block w-max max-w-[250px] p-2 bg-slate-800 text-white text-xs rounded-lg shadow-xl z-50 pointer-events-none">
        <div className="font-bold mb-1 border-b border-slate-600 pb-1">{data.label}</div>
        <div className="whitespace-pre-wrap break-words">{data.desc || '暂无详细说明'}</div>
        <div className="absolute left-1/2 -translate-x-1/2 top-full w-0 h-0 border-l-[6px] border-l-transparent border-r-[6px] border-r-transparent border-t-[6px] border-t-slate-800"></div>
      </div>

      <div className="absolute -top-2 -right-2">
        {data.state === 'success' && <CheckCircle2 className="w-5 h-5 text-emerald-500 bg-white rounded-full" />}
        {data.state === 'failure' && <XCircle className="w-5 h-5 text-red-500 bg-white rounded-full" />}
      </div>

      <Handle type="source" position={Position.Bottom} className="opacity-0" />
    </motion.div>
  );
};
