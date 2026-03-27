import { Handle, Position } from '@xyflow/react';
import { motion } from 'framer-motion';
import { Brain, Eye, Footprints, Hand, CheckCircle2, XCircle, Loader2, LucideIcon, ArrowRight, CornerDownRight, RotateCw, GitBranch, TerminalSquare, AlertCircle, PlayCircle, Clock } from 'lucide-react';
import { BTNode } from '../types';

const iconMap: Record<string, LucideIcon | Record<string, LucideIcon>> = {
  sequence: ArrowRight,
  selector: GitBranch,
  condition: Eye,
  action: TerminalSquare,
  decorator: RotateCw,
  subtree: CornerDownRight,
};

const bgColors = {
  sequence: 'bg-emerald-100 text-emerald-700 border-emerald-300',
  selector: 'bg-amber-100 text-amber-700 border-amber-300',
  condition: 'bg-blue-100 text-blue-700 border-blue-300',
  action: 'bg-purple-100 text-purple-700 border-purple-300',
  decorator: 'bg-rose-100 text-rose-700 border-rose-300',
  subtree: 'bg-slate-100 text-slate-700 border-slate-300',
};

const typeTranslations: Record<string, string> = {
  sequence: '顺序',
  selector: '选择',
  condition: '条件',
  action: '动作',
  decorator: '装饰',
  subtree: '子树',
};

const stateColors = {
  idle: 'opacity-80',
  running: 'ring-4 ring-blue-400 shadow-xl shadow-blue-200',
  success: 'ring-2 ring-emerald-400 opacity-90',
  failure: 'ring-2 ring-red-400 opacity-90',
};

export const CustomNode = ({ data }: { data: BTNode }) => {
  const isRunning = data.state === 'running';
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

  return (
    <motion.div
      animate={isRunning ? { scale: [1, 1.02, 1], transition: { repeat: Infinity, duration: 1.5 } } : { scale: 1 }}
      className={`relative px-4 py-3 rounded-xl border-2 flex items-center gap-3 transition-all duration-300 bg-white ${stateColor} min-w-[200px]`}
    >
      <Handle type="target" position={Position.Top} className="opacity-0" />
      
      <div className={`p-2 rounded-lg ${bgColor}`}>
        {isRunning ? <Loader2 className="w-6 h-6 animate-spin" /> : <Icon className="w-6 h-6" />}
      </div>
      
      <div className="flex-1 overflow-hidden">
        <div className="text-sm font-bold truncate" title={data.label}>{data.label}</div>
        <div className="text-xs text-gray-500 font-mono mt-0.5 truncate" title={data.desc}>{data.desc}</div>
        
        {/* Type Badge */}
        <div className="absolute -top-2.5 left-4 text-[10px] font-bold px-2 py-0.5 rounded-full bg-slate-800 text-white shadow-sm">
          {typeTranslations[data.type] || data.type}
        </div>
      </div>

      <div className="absolute -top-3 -right-3">
        {data.state === 'success' && <CheckCircle2 className="w-6 h-6 text-emerald-500 bg-white rounded-full" />}
        {data.state === 'failure' && <XCircle className="w-6 h-6 text-red-500 bg-white rounded-full" />}
      </div>

      <Handle type="source" position={Position.Bottom} className="opacity-0" />
    </motion.div>
  );
};
