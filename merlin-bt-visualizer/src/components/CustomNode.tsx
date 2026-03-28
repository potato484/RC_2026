import { Handle, Position } from '@xyflow/react';
import { motion } from 'framer-motion';
import { Brain, Eye, Footprints, Hand, CheckCircle2, XCircle, Loader2, LucideIcon, ArrowRight, RotateCw, GitBranch, TerminalSquare, AlertCircle, PlayCircle, Clock, ChevronDown, ChevronRight, FolderTree } from 'lucide-react';
import { BTNode } from '../types';

const iconMap: Record<string, LucideIcon | Record<string, LucideIcon>> = {
  sequence: ArrowRight,
  selector: GitBranch,
  condition: Eye,
  action: TerminalSquare,
  decorator: RotateCw,
  subtree: FolderTree, // 改用文件夹树图标，代表里面有子流程
};

const bgColors: Record<string, string> = {
  sequence: 'bg-teal-50 text-teal-700 border-teal-200',
  selector: 'bg-amber-50 text-amber-700 border-amber-200',
  condition: 'bg-slate-50 text-slate-600 border-slate-300 border-dashed',
  action: 'bg-white text-slate-700 border-slate-200',
  decorator: 'bg-rose-50 text-rose-700 border-rose-200 border-dashed',
  subtree: 'bg-indigo-50 text-indigo-700 border-indigo-200 border-dashed',
};

const stateColors: Record<string, string> = {
  idle: 'opacity-90',
  running: 'ring-4 ring-amber-400 shadow-xl shadow-amber-200 border-amber-400',
  success: 'ring-2 ring-emerald-400 border-emerald-400 opacity-100',
  failure: 'ring-2 ring-red-400 border-red-400 opacity-100',
};

export const CustomNode = ({ data }: { data: BTNode }) => {
  const isRunning = data.state === 'running';
  let Icon = (iconMap[data.type] as LucideIcon) || Brain;
  
  if (data.type === 'action') {
    if (data.label.includes('导航')) Icon = Footprints;
    else if (data.label.includes('抓取')) Icon = Hand;
    else if (data.label.includes('设置')) Icon = PlayCircle;
    else if (data.label.includes('延迟')) Icon = Clock;
    else if (data.label.includes('检测') || data.label.includes('检查')) Icon = AlertCircle;
  }

  const baseBgColor = bgColors[data.type] || bgColors.action;
  const stateColor = stateColors[data.state] || stateColors.idle;
  
  const hasChildren = data.type === 'sequence' || data.type === 'selector' || data.type === 'decorator' || data.type === 'subtree';
  const isSubTreeContainer = data.type === 'subtree';

  const isControlFlow = data.type === 'sequence' || data.type === 'selector';
  const isModifier = data.type === 'decorator';
  const isAction = data.type === 'action' || data.type === 'subtree';

  let sizeClass = '';
  let shapeClass = '';

  if (isControlFlow) {
    sizeClass = 'w-[140px] h-[48px]';
    shapeClass = 'rounded-full'; // 胶囊形状
  } else if (isModifier) {
    sizeClass = 'w-[180px] h-[48px]';
    shapeClass = 'rounded-xl'; // 扁平矩形
  } else {
    sizeClass = 'w-[240px] h-[80px]';
    shapeClass = 'rounded-xl'; // 大卡片
  }

  // 整理所有附加的 modifiers (decorators)
  const attachedModifiers = [];
  if (data.decorators && data.decorators.length > 0) {
    attachedModifiers.push(...data.decorators.map(d => ({ ...d, isDecorator: true })));
  }

  return (
    <div className="relative">
      {/* 附加修饰器：横向紧凑排列，紧贴节点左上方 */}
      {attachedModifiers.length > 0 && (
        <div className="absolute bottom-full left-0 mb-1 flex flex-wrap gap-1 w-max max-w-[320px] z-20">
          {attachedModifiers.map((mod) => (
            <div 
              key={mod.id} 
              className={`text-[10px] px-2 py-0.5 rounded shadow-sm border whitespace-nowrap flex items-center gap-1 cursor-pointer transition-colors ${
                mod.isDecorator 
                  ? 'bg-rose-50 text-rose-600 border-rose-200 hover:bg-rose-100' 
                  : 'bg-slate-50 text-slate-600 border-slate-200 hover:bg-slate-100'
              }`}
              title={mod.desc}
            >
              {mod.isDecorator ? <RotateCw className="w-3 h-3" /> : <Eye className="w-3 h-3" />}
              <span className="font-bold">{mod.label}</span>
            </div>
          ))}
        </div>
      )}

      <motion.div
        animate={isRunning ? { scale: [1, 1.02, 1], transition: { repeat: Infinity, duration: 1.5 } } : { scale: 1 }}
        className={`relative border-2 flex items-center transition-all duration-300 bg-white cursor-pointer hover:border-slate-400 ${sizeClass} ${shapeClass} ${stateColor}`}
        title={data.subTreeId && !isSubTreeContainer ? `来自子树: ${data.subTreeId}` : undefined}
      >
        <Handle type="target" position={Position.Left} className="opacity-0" />
        
        {/* Sibling Index Badge */}
        {data.siblingIndex !== undefined && data.siblingIndex > 0 && (
          <div className="absolute top-1/2 -translate-y-1/2 -left-3 w-5 h-5 rounded-full bg-slate-700 text-white flex items-center justify-center text-[10px] font-bold shadow-sm z-10 border border-white">
            {data.siblingIndex}
          </div>
        )}

        {/* 节点内部内容布局 */}
        <div className={`w-full h-full flex items-center ${isAction ? 'p-3 gap-2' : 'px-3 gap-1.5 justify-center'}`}>
          
          {/* Icon */}
          <div className={`flex-none rounded-lg flex items-center justify-center ${baseBgColor} ${isAction ? 'p-2 w-9 h-9' : 'p-1 w-6 h-6 rounded-full'}`}>
            {isRunning ? <Loader2 className="w-full h-full animate-spin" /> : <Icon className="w-full h-full" />}
          </div>
          
          {/* Text Content */}
          <div className="flex-1 overflow-hidden flex flex-col justify-center min-w-0">
            <div className="flex items-center gap-1 justify-between">
              <div className={`font-bold truncate ${isAction ? 'text-sm text-left' : 'text-xs text-center flex-1'}`} title={data.label}>
                {data.label}
              </div>
              
              {/* Control elements right side */}
              <div className="flex items-center gap-0.5 flex-none">
                {hasChildren && (
                  <div className="text-slate-400 bg-slate-100 rounded p-0.5" title="双击折叠/展开">
                    {data.collapsed ? <ChevronRight className="w-3 h-3" /> : <ChevronDown className="w-3 h-3" />}
                  </div>
                )}
              </div>
            </div>
            
            {/* Description only for Action/Subtree */}
            {isAction && (
              <div className="text-[11px] text-gray-500 font-mono mt-0.5 truncate text-left" title={data.desc}>
                {data.collapsed ? '（已折叠）' : data.desc}
              </div>
            )}
          </div>
        </div>


        {/* Status Icons */}
        <div className="absolute -top-2 -right-2 z-10">
          {data.state === 'success' && <CheckCircle2 className="w-5 h-5 text-emerald-500 bg-white rounded-full" />}
          {data.state === 'failure' && <XCircle className="w-5 h-5 text-red-500 bg-white rounded-full" />}
        </div>

        <Handle type="source" position={Position.Right} className="opacity-0" />
      </motion.div>
    </div>
  );
};
