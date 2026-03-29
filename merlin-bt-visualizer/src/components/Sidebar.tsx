import { Swords, Map, Crosshair, Target, FolderTree, FileJson } from 'lucide-react';
import { useStore } from '../store/useStore';
import { useEditorStore } from '../store/useEditorStore';
import { motion } from 'framer-motion';

const phases = [
  { id: '武馆区', icon: Swords, color: 'text-rose-500', bg: 'bg-rose-100' },
  { id: '梅林区', icon: Map, color: 'text-sky-500', bg: 'bg-sky-100' },
  { id: '对抗区', icon: Crosshair, color: 'text-purple-500', bg: 'bg-purple-100' },
];

export const Sidebar = () => {
  const { activePhase, setActivePhase, activeTreeId, setActiveTree, trees, appMode } = useStore();
  const editorDocument = useEditorStore(state => state.document);
  const editorActiveTreeId = useEditorStore(state => state.activeTreeId);
  const setEditorActiveTree = useEditorStore(state => state.setActiveTree);

  const treeList = Object.keys(trees);
  
  const handlePhaseChange = (phaseId: string) => {
    setActivePhase(phaseId as any);
    // If in editor mode, the Header component handles the XML loading
  };

  return (
    <div className="glass-panel w-48 flex flex-col gap-4 p-4 py-8 mr-4 overflow-y-auto">
      <div className="flex flex-col gap-4 items-center mb-6">
        {phases.map((p) => {
          const isActive = activePhase === p.id;
          const Icon = p.icon;
          return (
            <button
              key={p.id}
              onClick={() => handlePhaseChange(p.id)}
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
        <div className="flex flex-col gap-1 w-full">
          {appMode === 'viewer' ? (
            // Viewer Mode Trees
            treeList.map((treeId) => {
              const tree = trees[treeId];
              if (!tree || (tree.parentTreeId && trees[tree.parentTreeId])) return null;
              
              const renderTreeItem = (id: string, depth: number) => {
                const currentTree = trees[id];
                if (!currentTree) return null;
                const isTreeActive = activeTreeId === id;
                const treeName = currentTree.name || id;
                
                const childTrees = treeList.filter(childId => trees[childId]?.parentTreeId === id);
                
                return (
                  <div key={id} className="flex flex-col w-full">
                    <button
                      onClick={() => setActiveTree(id)}
                      style={{ paddingLeft: `${0.75 + depth * 1.0}rem` }}
                      className={`relative flex items-center gap-2 w-full py-2.5 pr-3 rounded-xl transition-all duration-200 text-left ${
                        isTreeActive
                          ? 'bg-slate-700 text-white shadow-md shadow-slate-500/20'
                          : 'text-slate-600 hover:bg-white/60 hover:text-slate-900'
                      }`}
                    >
                      {depth === 0 ? (
                        <Target className={`w-4 h-4 shrink-0 ${isTreeActive ? 'text-slate-100' : 'text-slate-400'}`} />
                      ) : childTrees.length > 0 ? (
                        <FolderTree className={`w-3.5 h-3.5 shrink-0 ${isTreeActive ? 'text-indigo-200' : 'text-indigo-400'}`} />
                      ) : (
                        <FileJson className={`w-3.5 h-3.5 shrink-0 ${isTreeActive ? 'text-slate-300' : 'text-slate-400'}`} />
                      )}
                      <span className="text-sm font-medium truncate" title={treeName}>
                        {treeName}
                      </span>
                    </button>
                    
                    {childTrees.length > 0 && (
                      <div className="flex flex-col mt-0.5 relative gap-0.5">
                        {childTrees.map(childId => renderTreeItem(childId, depth + 1))}
                      </div>
                    )}
                  </div>
                );
              };

              return renderTreeItem(treeId, 0);
            })
          ) : (
            // Editor Mode Trees
            editorDocument?.trees.map((tree) => {
              const isTreeActive = editorActiveTreeId === tree.id;
              const treeName = tree.name || tree.id;
              return (
                <button
                  key={tree.id}
                  onClick={() => setEditorActiveTree(tree.id)}
                  className={`relative flex items-center gap-2 w-full py-2.5 px-3 rounded-xl transition-all duration-200 text-left ${
                    isTreeActive
                      ? 'bg-blue-600 text-white shadow-md shadow-blue-500/20'
                      : 'text-slate-600 hover:bg-white/60 hover:text-slate-900'
                  }`}
                >
                  <Target className={`w-4 h-4 shrink-0 ${isTreeActive ? 'text-slate-100' : 'text-slate-400'}`} />
                  <span className="text-sm font-medium truncate" title={treeName}>
                    {treeName}
                  </span>
                </button>
              );
            })
          )}
        </div>
      </div>
    </div>
  );
};
