
import { useStore } from '../store/useStore';
import { motion, AnimatePresence } from 'framer-motion';
import { BrainCircuit, Database } from 'lucide-react';

export const RightPanel = () => {
  const { nodes, activeNodeId, blackboard } = useStore();
  const activeNode = nodes.find(n => n.id === activeNodeId) || nodes.find(n => n.state === 'running');

  return (
    <div className="w-80 flex flex-col gap-4 ml-4">
      {/* Node Info Card */}
      <div className="glass-panel p-6 flex-1 flex flex-col">
        <div className="flex items-center gap-3 text-gray-500 mb-6 font-bold">
          <BrainCircuit className="w-6 h-6" />
          <span>当前焦点</span>
        </div>
        
        <AnimatePresence mode="wait">
          <motion.div
            key={activeNode?.id || 'empty'}
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: -20 }}
            className="flex-1 flex flex-col items-center justify-center text-center gap-6"
          >
            {activeNode ? (
              <>
                <div className="text-6xl text-blue-500">
                  {activeNode.state === 'success' ? '✅' : activeNode.state === 'failure' ? '❌' : activeNode.state === 'running' ? '⏳' : '💤'}
                </div>
                <h2 className="text-3xl font-bold text-gray-800">{activeNode.label}</h2>
                <p className="text-xl text-gray-600 bg-white/60 p-4 rounded-2xl w-full">
                  {activeNode.desc}
                </p>
                <div className="px-6 py-2 rounded-full font-bold text-xl bg-blue-100 text-blue-600 uppercase">
                  {activeNode.state}
                </div>
              </>
            ) : (
              <div className="text-gray-400 text-xl font-medium">点击左侧节点查看详情</div>
            )}
          </motion.div>
        </AnimatePresence>
      </div>

      {/* Blackboard Card */}
      <div className="glass-panel p-6 flex-[1.5] flex flex-col">
        <div className="flex items-center gap-3 text-gray-500 mb-4 font-bold">
          <Database className="w-6 h-6" />
          <span>黑板记忆 (最新)</span>
        </div>
        
        <div className="flex flex-col gap-3 overflow-y-auto pr-2">
          <AnimatePresence>
            {blackboard.map((item) => (
              <motion.div
                key={item.key}
                initial={{ opacity: 0, x: 20 }}
                animate={{ opacity: 1, x: 0 }}
                className="bg-white/60 p-4 rounded-2xl flex flex-col gap-2"
              >
                <div className="flex justify-between items-center">
                  <span className="text-lg font-bold text-gray-700">{item.desc}</span>
                  <span className={`px-4 py-1 rounded-xl font-bold text-lg ${
                    item.value === '是' ? 'bg-emerald-100 text-emerald-600' :
                    item.value === '否' ? 'bg-red-100 text-red-600' :
                    'bg-blue-100 text-blue-600'
                  }`}>
                    {item.value}
                  </span>
                </div>
              </motion.div>
            ))}
          </AnimatePresence>
        </div>
      </div>
    </div>
  );
};
