
import { Swords, Map, Crosshair, Target } from 'lucide-react';
import { useStore } from '../store/useStore';
import { motion, AnimatePresence } from 'framer-motion';

const phases = [
  { id: '武馆区', icon: Swords, color: 'text-rose-500', bg: 'bg-rose-100' },
  { id: '梅林区', icon: Map, color: 'text-blue-500', bg: 'bg-blue-100' },
  { id: '对抗区', icon: Crosshair, color: 'text-purple-500', bg: 'bg-purple-100' },
];

export const Sidebar = () => {
  const { activePhase, setActivePhase, activeTreeId, setActiveTree, trees } = useStore();
  const treeList = Object.keys(trees);

  return (
    <div className="glass-panel w-48 flex flex-col gap-4 p-4 py-8 mr-4 overflow-y-auto">
      <div className="flex flex-col gap-4 items-center mb-6">
        {phases.map((p) => {
          const isActive = activePhase === p.id;
          const Icon = p.icon;
          return (
            <button
              key={p.id}
              onClick={() => setActivePhase(p.id as any)}
              className="relative flex flex-col items-center gap-2 w-full group"
            >
              {isActive && (
                <motion.div
                  layoutId="active-pill"
                  className={`absolute inset-0 ${p.bg} rounded-3xl -z-10`}
                  transition={{ type: "spring", stiffness: 300, damping: 30 }}
                />
              )}
              <div className={`p-4 rounded-2xl transition-all duration-300 ${isActive ? p.color : 'text-gray-400 group-hover:bg-white/50'}`}>
                <Icon className="w-10 h-10" />
              </div>
              <span className={`text-sm font-bold ${isActive ? p.color : 'text-gray-500'}`}>{p.id}</span>
            </button>
          );
        })}
      </div>

      <div className="w-full h-px bg-slate-200/50 my-2" />

      <div className="flex flex-col gap-2 w-full">
        <h3 className="text-xs font-bold text-slate-400 uppercase tracking-wider mb-2 px-2">
          子树列表
        </h3>
        <AnimatePresence mode="popLayout">
          {treeList.map((treeId) => {
            const isTreeActive = activeTreeId === treeId;
            const treeName = trees[treeId]?.name || treeId;
            return (
              <motion.button
                initial={{ opacity: 0, x: -10 }}
                animate={{ opacity: 1, x: 0 }}
                exit={{ opacity: 0, x: -10 }}
                key={treeId}
                onClick={() => setActiveTree(treeId)}
                className={`relative flex items-center gap-2 w-full p-3 rounded-xl transition-all duration-300 text-left ${
                  isTreeActive 
                    ? 'bg-blue-500 text-white shadow-md shadow-blue-500/20' 
                    : 'text-slate-600 hover:bg-white/60 hover:text-slate-900'
                }`}
              >
                <Target className={`w-4 h-4 shrink-0 ${isTreeActive ? 'text-blue-100' : 'text-slate-400'}`} />
                <span className="text-sm font-medium truncate" title={treeName}>
                  {treeName}
                </span>
              </motion.button>
            );
          })}
        </AnimatePresence>
      </div>
    </div>
  );
};
