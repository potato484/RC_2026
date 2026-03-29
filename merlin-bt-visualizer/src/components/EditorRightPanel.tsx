import { useEditorStore } from '../store/useEditorStore';
import { Settings, Plus, X, Trash2, CornerDownRight } from 'lucide-react';
import { useState, useEffect } from 'react';

export const EditorRightPanel = () => {
  const { 
    selectedNodeId, 
    document, 
    activeTreeId,
    updateNodeAttributes,
    addChildNode,
    deleteNode,
    exportXml
  } = useEditorStore();

  const [attributes, setAttributes] = useState<Record<string, string>>({});
  const [newKey, setNewKey] = useState('');
  const [newValue, setNewValue] = useState('');
  const [newChildTag, setNewChildTag] = useState('');
  const [activeTab, setActiveTab] = useState<'props' | 'xml'>('props');

  // Find selected node to display properties
  let selectedNode = null;
  if (selectedNodeId && document && activeTreeId) {
    const findNodeById = (node: any, id: string): any => {
      if (node.id === id) return node;
      for (const child of node.children) {
        const found = findNodeById(child, id);
        if (found) return found;
      }
      return null;
    };
    
    const activeTree = document.trees.find(t => t.id === activeTreeId);
    if (activeTree) {
      selectedNode = findNodeById(activeTree.rootNode, selectedNodeId);
    }
  }

  // Load attributes when selected node changes
  useEffect(() => {
    if (selectedNode) {
      setAttributes({ ...selectedNode.attributes });
    } else {
      setAttributes({});
    }
    setNewKey('');
    setNewValue('');
    setNewChildTag('');
  }, [selectedNode?.id]);

  const handleAttrChange = (key: string, value: string) => {
    setAttributes(prev => ({ ...prev, [key]: value }));
  };

  const handleRemoveAttr = (key: string) => {
    const newAttrs = { ...attributes };
    delete newAttrs[key];
    setAttributes(newAttrs);
    if (selectedNodeId) {
      updateNodeAttributes(selectedNodeId, newAttrs);
    }
  };

  const handleAddAttr = () => {
    if (!newKey.trim()) return;
    const newAttrs = { ...attributes, [newKey]: newValue };
    setAttributes(newAttrs);
    setNewKey('');
    setNewValue('');
    if (selectedNodeId) {
      updateNodeAttributes(selectedNodeId, newAttrs);
    }
  };

  const handleSaveAttrs = () => {
    if (selectedNodeId) {
      updateNodeAttributes(selectedNodeId, attributes);
    }
  };

  const handleAddChild = () => {
    if (!newChildTag.trim() || !selectedNodeId) return;
    addChildNode(selectedNodeId, newChildTag.trim());
    setNewChildTag('');
  };

  const handleDeleteNode = () => {
    if (selectedNodeId) {
      deleteNode(selectedNodeId);
    }
  };

  const currentXml = exportXml() || '';

  return (
    <div className="w-[380px] h-full flex flex-col gap-4 ml-4">
      <div className="glass-panel p-4 flex-1 flex flex-col">
        {/* Tabs */}
        <div className="flex items-center gap-2 mb-4 pb-2 border-b border-slate-200">
          <button 
            onClick={() => setActiveTab('props')}
            className={`flex-1 py-2 text-sm font-bold transition-colors border-b-2 ${activeTab === 'props' ? 'border-blue-500 text-blue-600' : 'border-transparent text-slate-500 hover:text-slate-700'}`}
          >
            节点属性
          </button>
          <button 
            onClick={() => setActiveTab('xml')}
            className={`flex-1 py-2 text-sm font-bold transition-colors border-b-2 ${activeTab === 'xml' ? 'border-blue-500 text-blue-600' : 'border-transparent text-slate-500 hover:text-slate-700'}`}
          >
            实时 XML 预览
          </button>
        </div>

        {activeTab === 'xml' ? (
          <div className="flex-1 overflow-y-auto custom-scrollbar bg-slate-800 rounded-xl p-4 text-slate-300 font-mono text-xs whitespace-pre">
            {currentXml}
          </div>
        ) : (
          <>
            {!selectedNode ? (
              <div className="flex-1 flex flex-col items-center justify-center text-slate-400">
                <Settings className="w-12 h-12 mb-4 opacity-50" />
                <p>请在画布中选中一个节点以编辑属性</p>
              </div>
            ) : (
              <>
                <div className="flex items-center justify-between mb-4 pb-4 border-b border-slate-200">
                  <div className="flex flex-col gap-1">
                    <h3 className="text-lg font-bold flex items-center gap-2 text-slate-800">
                      <Settings className="w-5 h-5 text-blue-500" />
                      编辑节点
                    </h3>
                    <span className="text-xs bg-slate-100 text-slate-500 px-2 py-1 rounded font-mono truncate max-w-[200px]" title={selectedNode.tagName}>
                      {selectedNode.tagName}
                    </span>
                  </div>
                  <button 
                    onClick={handleDeleteNode}
                    className="p-2 text-rose-500 hover:bg-rose-50 rounded-lg transition-colors"
                    title="删除节点"
                  >
                    <Trash2 className="w-5 h-5" />
                  </button>
                </div>

                <div className="flex-1 overflow-y-auto pr-2 custom-scrollbar">
                  <div className="space-y-4">
                    {Object.entries(attributes).map(([key, value]) => (
                      <div key={key} className="flex flex-col gap-1.5 p-3 bg-white/50 rounded-lg border border-slate-200">
                        <div className="flex items-center justify-between">
                          <label className="text-sm font-semibold text-slate-700 font-mono">{key}</label>
                          <button 
                            onClick={() => handleRemoveAttr(key)}
                            className="text-slate-400 hover:text-red-500 transition-colors"
                          >
                            <X className="w-4 h-4" />
                          </button>
                        </div>
                        <input
                          type="text"
                          value={value}
                          onChange={(e) => handleAttrChange(key, e.target.value)}
                          onBlur={handleSaveAttrs}
                          className="w-full px-3 py-1.5 bg-white border border-slate-200 rounded-md text-sm focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent font-mono"
                        />
                      </div>
                    ))}

                    {Object.keys(attributes).length === 0 && (
                      <div className="text-sm text-center text-slate-400 py-4 border border-dashed border-slate-200 rounded-lg">
                        该节点暂无属性
                      </div>
                    )}
                  </div>
                </div>

                <div className="mt-4 pt-4 border-t border-slate-200 flex flex-col gap-4">
                  <div>
                    <h4 className="text-sm font-semibold text-slate-700 mb-2">添加新属性</h4>
                    <div className="flex flex-col gap-2">
                      <div className="flex gap-2">
                        <input
                          type="text"
                          placeholder="属性名"
                          value={newKey}
                          onChange={(e) => setNewKey(e.target.value)}
                          className="flex-1 min-w-0 px-3 py-1.5 bg-white border border-slate-200 rounded-md text-sm focus:outline-none focus:ring-2 focus:ring-blue-500 font-mono"
                        />
                        <input
                          type="text"
                          placeholder="属性值"
                          value={newValue}
                          onChange={(e) => setNewValue(e.target.value)}
                          onKeyDown={(e) => e.key === 'Enter' && handleAddAttr()}
                          className="flex-1 min-w-0 px-3 py-1.5 bg-white border border-slate-200 rounded-md text-sm focus:outline-none focus:ring-2 focus:ring-blue-500 font-mono"
                        />
                      </div>
                      <button
                        onClick={handleAddAttr}
                        disabled={!newKey.trim()}
                        className="w-full flex items-center justify-center gap-1.5 px-3 py-1.5 bg-slate-100 hover:bg-slate-200 text-slate-700 rounded-md text-sm font-medium transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                      >
                        <Plus className="w-4 h-4" />
                        添加属性
                      </button>
                    </div>
                  </div>
                  
                  <div className="pt-4 border-t border-slate-100">
                    <h4 className="text-sm font-semibold text-slate-700 mb-2">添加子节点</h4>
                    <div className="flex flex-col gap-2">
                      <input
                        type="text"
                        placeholder="节点类型 (如 Sequence, GrabTip)"
                        value={newChildTag}
                        onChange={(e) => setNewChildTag(e.target.value)}
                        onKeyDown={(e) => e.key === 'Enter' && handleAddChild()}
                        className="w-full px-3 py-1.5 bg-white border border-slate-200 rounded-md text-sm focus:outline-none focus:ring-2 focus:ring-blue-500 font-mono"
                      />
                      <button
                        onClick={handleAddChild}
                        disabled={!newChildTag.trim()}
                        className="w-full flex items-center justify-center gap-1.5 px-3 py-2 bg-blue-600 hover:bg-blue-700 text-white rounded-md text-sm font-medium transition-colors disabled:opacity-50 disabled:cursor-not-allowed shadow-sm"
                      >
                        <CornerDownRight className="w-4 h-4" />
                        添加子节点
                      </button>
                    </div>
                  </div>
                </div>
              </>
            )}
          </>
        )}
      </div>
    </div>
  );
};
