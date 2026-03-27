import { Handle, Position } from '@xyflow/react';
import { motion } from 'framer-motion';
import { Brain, Eye, Footprints, Hand, CheckCircle2, XCircle, Loader2, LucideIcon } from 'lucide-react';
import { BTNode } from '../types';

const iconMap: Record<string, LucideIcon | Record<string, LucideIcon>> = {
  sequence: Brain,
  selector: Brain,
  condition: Eye,
  action: {
    '扫描': Eye,
    '左右张望': Eye,
    '走向目标': Footprints,
    '伸手抓取': Hand,
  }
};

const stateColors = {
  idle: 'bg-white/80 text-gray-500 border-gray-200',
  running: 'bg-blue-50 text-blue-600 border-blue-400 shadow-blue-200 shadow-lg',
  success: 'bg-emerald-50 text-emerald-600 border-emerald-400',
  failure: 'bg-red-50 text-red-600 border-red-400',
};

export const CustomNode = ({ data }: { data: BTNode }) => {
  const isRunning = data.state === 'running';
  let Icon = Brain as LucideIcon;
  if (data.type === 'action') {
    const actionMap = iconMap.action as Record<string, LucideIcon>;
    Icon = actionMap[data.label] || Hand;
  } else {
    Icon = (iconMap[data.type] as LucideIcon) || Brain;
  }

  return (
    <motion.div
      animate={isRunning ? { scale: [1, 1.05, 1], transition: { repeat: Infinity, duration: 2 } } : { scale: 1 }}
      className={`relative px-6 py-4 rounded-3xl border-2 flex items-center gap-4 transition-all duration-500 ${stateColors[data.state]}`}
    >
      <Handle type="target" position={Position.Top} className="opacity-0" />
      
      <div className={`p-3 rounded-2xl ${isRunning ? 'bg-blue-100 text-blue-600' : 'bg-gray-100 text-gray-500'}`}>
        {isRunning ? <Loader2 className="w-8 h-8 animate-spin" /> : <Icon className="w-8 h-8" />}
      </div>
      
      <div>
        <div className="text-xl font-bold tracking-wider">{data.label}</div>
        <div className="text-sm opacity-70 mt-1">{data.desc}</div>
      </div>

      <div className="absolute -top-3 -right-3">
        {data.state === 'success' && <CheckCircle2 className="w-8 h-8 text-emerald-500 bg-white rounded-full" />}
        {data.state === 'failure' && <XCircle className="w-8 h-8 text-red-500 bg-white rounded-full" />}
      </div>

      <Handle type="source" position={Position.Bottom} className="opacity-0" />
    </motion.div>
  );
};
