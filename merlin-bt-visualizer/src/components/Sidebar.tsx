
import { Swords, Map, Crosshair } from 'lucide-react';
import { useStore } from '../store/useStore';
import { motion } from 'framer-motion';

const phases = [
  { id: '武馆区', icon: Swords, color: 'text-rose-500', bg: 'bg-rose-100' },
  { id: '梅林区', icon: Map, color: 'text-blue-500', bg: 'bg-blue-100' },
  { id: '对抗区', icon: Crosshair, color: 'text-purple-500', bg: 'bg-purple-100' },
];

export const Sidebar = () => {
  const { activePhase, setActivePhase } = useStore();

  return (
    <div className="glass-panel w-32 flex flex-col gap-4 p-4 items-center py-8 mr-4">
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
  );
};
