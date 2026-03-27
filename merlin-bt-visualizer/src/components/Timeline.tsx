
import { useStore } from '../store/useStore';
import { motion } from 'framer-motion';
import { Clock, CheckCircle2, Info, AlertTriangle } from 'lucide-react';

export const Timeline = () => {
  const { timeline } = useStore();

  return (
    <div className="glass-panel mt-4 p-4 h-32 flex items-center overflow-x-auto gap-4 scroll-smooth">
      <div className="sticky left-0 bg-glassDark p-4 rounded-2xl backdrop-blur-md flex items-center gap-3 font-bold text-gray-600 z-10 shrink-0">
        <Clock className="w-8 h-8" />
        <span className="text-xl">时间线</span>
      </div>

      <div className="flex gap-4 items-center px-4">
        {timeline.map((event, i) => (
          <motion.div
            key={event.id}
            initial={{ opacity: 0, scale: 0.8 }}
            animate={{ opacity: 1, scale: 1 }}
            className="flex items-center gap-4 shrink-0"
          >
            <div className={`flex items-center gap-3 px-6 py-4 rounded-3xl bg-white/70 shadow-sm ${
              event.status === 'success' ? 'border-b-4 border-emerald-400' :
              event.status === 'warning' ? 'border-b-4 border-amber-400' :
              'border-b-4 border-blue-400'
            }`}>
              {event.status === 'success' ? <CheckCircle2 className="w-8 h-8 text-emerald-500" /> :
               event.status === 'warning' ? <AlertTriangle className="w-8 h-8 text-amber-500" /> :
               <Info className="w-8 h-8 text-blue-500" />}
              
              <div className="flex flex-col">
                <span className="text-lg font-bold text-gray-800">{event.desc}</span>
                <span className="text-sm text-gray-500">{event.time}</span>
              </div>
            </div>
            {i < timeline.length - 1 && <div className="w-8 h-1 bg-gray-300 rounded-full" />}
          </motion.div>
        ))}
      </div>
    </div>
  );
};
