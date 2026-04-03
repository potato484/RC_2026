import { useEffect, useMemo, useState } from 'react';
import { CornerDownRight, Database, GitBranch, Info, Layers3, Plus, Trash2 } from 'lucide-react';
import { useEditorStore } from '../store/useEditorStore';
import { EditorNode } from '../types/editor';
import { BtPortSchema } from '../generated/btNodeRegistry';
import {
  getBehaviorTreeAttributeDisplays,
  getBehaviorTreeNodeCategoryLabel,
  getBehaviorTreeNodeDisplay,
  getBehaviorTreeTreeName,
  translateBlackboardKey,
} from '../utils/btDisplay';
import { buildEditorTreePreview } from '../utils/editorTreeView';
import { formatPortBindingValue, getBtNodeDefinition, getBtNodeRegistry } from '../utils/btRegistry';

const findNodeById = (node: EditorNode, nodeId: string): EditorNode | null => {
  if (node.id === nodeId) {
    return node;
  }
  for (const child of node.children) {
    const found = findNodeById(child, nodeId);
    if (found) {
      return found;
    }
  }
  return null;
};

const bindingModeOptions = [
  { value: 'literal', label: '固定值' },
  { value: 'blackboard', label: '黑板' },
  { value: 'root_blackboard', label: '根黑板' },
] as const;

export const EditorRightPanel = () => {
  const {
    selectedNodeId,
    document,
    activeTreeId,
    updateSingleAttribute,
    deleteNode,
    exportXml,
    insertNode,
    replaceNodeType,
  } = useEditorStore();

  const [activeTab, setActiveTab] = useState<'info' | 'preview' | 'source'>('info');
  const [insertPosition, setInsertPosition] = useState<'before' | 'after' | 'prepend_child' | 'append_child'>('append_child');
  const [insertTagName, setInsertTagName] = useState('Sequence');
  const [wrapTagName, setWrapTagName] = useState('Inverter');
  const [newKey, setNewKey] = useState('');
  const [newValue, setNewValue] = useState('');

  const activeTree = useMemo(
    () => (activeTreeId && document ? document.trees.find((tree) => tree.id === activeTreeId) ?? null : null),
    [activeTreeId, document]
  );

  const selectedNode = useMemo(() => {
    if (!activeTree || !selectedNodeId) {
      return null;
    }
    return findNodeById(activeTree.rootNode, selectedNodeId);
  }, [activeTree, selectedNodeId]);

  useEffect(() => {
    setNewKey('');
    setNewValue('');
  }, [selectedNodeId]);

  const registry = useMemo(() => getBtNodeRegistry(), []);
  const selectedNodeDisplay = selectedNode ? getBehaviorTreeNodeDisplay(selectedNode.tagName, selectedNode.attributes) : null;
  const selectedDefinition = selectedNode ? getBtNodeDefinition(selectedNode.tagName) : undefined;
  const sourcePreview = exportXml() ?? '当前没有可导出的源文件内容';
  const currentStructurePreview = document ? buildEditorTreePreview(document, activeTreeId) : '当前没有可预览的结构';
  const attributeDisplays = selectedNode ? getBehaviorTreeAttributeDisplays(selectedNode.attributes, selectedNode.tagName) : [];

  const extraAttributes = attributeDisplays.filter(
    (attribute) => !selectedDefinition?.portSchemas.some((port) => port.name === attribute.rawKey)
  );

  const renderPortField = (port: BtPortSchema) => {
    if (!selectedNode) {
      return null;
    }
    const binding = selectedNode.portBindings[port.name];
    const currentMode = binding?.mode ?? (port.direction === 'output' ? 'blackboard' : 'literal');
    const currentValue = binding?.bindingValue ?? port.defaultValue ?? '';

    return (
      <div key={port.name} className="rounded-2xl border border-slate-200 bg-white/80 p-3">
        <div className="flex items-start justify-between gap-3">
          <div>
            <div className="text-sm font-semibold text-slate-800">{port.labelZh}</div>
            <div className="mt-1 text-xs leading-5 text-slate-500">{port.descriptionZh}</div>
          </div>
          <span className="rounded-full bg-slate-100 px-2 py-1 text-[10px] font-semibold text-slate-500">
            {port.direction === 'input' ? '输入' : port.direction === 'output' ? '输出' : '双向'}
          </span>
        </div>

        <div className="mt-3 flex gap-2">
          <select
            value={currentMode}
            onChange={(event) =>
              updateSingleAttribute(
                selectedNode.id,
                port.name,
                formatPortBindingValue(event.target.value as 'literal' | 'blackboard' | 'root_blackboard', currentValue)
              )
            }
            className="rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
          >
            {bindingModeOptions.map((option) => (
              <option key={option.value} value={option.value}>
                {option.label}
              </option>
            ))}
          </select>
          <input
            value={currentValue}
            onChange={(event) =>
              updateSingleAttribute(
                selectedNode.id,
                port.name,
                formatPortBindingValue(currentMode, event.target.value)
              )
            }
            placeholder={port.defaultValue ? `默认值：${port.defaultValue}` : '请输入参数或黑板键'}
            className="flex-1 rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none placeholder:text-slate-400"
          />
        </div>

        {currentMode !== 'literal' && currentValue && (
          <div className="mt-2 text-xs text-emerald-700">
            当前绑定：{currentMode === 'root_blackboard' ? '根黑板' : '黑板'} {translateBlackboardKey(currentValue)}
          </div>
        )}
      </div>
    );
  };

  const handleAddAttribute = () => {
    if (!selectedNode || !newKey.trim()) {
      return;
    }
    updateSingleAttribute(selectedNode.id, newKey.trim(), newValue);
    setNewKey('');
    setNewValue('');
  };

  return (
    <div className="ml-4 flex h-full w-[420px] flex-col gap-4">
      <div className="glass-panel flex min-h-0 flex-1 flex-col p-4" data-testid="editor-right-panel">
        <div className="mb-4 flex items-center gap-2 border-b border-slate-200 pb-2">
          {[
            { id: 'info', label: '节点信息' },
            { id: 'preview', label: '结构预览' },
            { id: 'source', label: '源文件预览' },
          ].map((tab) => (
            <button
              key={tab.id}
              type="button"
              onClick={() => setActiveTab(tab.id as 'info' | 'preview' | 'source')}
              data-testid={tab.id === 'preview' ? 'editor-tab-preview' : undefined}
              className={`flex-1 rounded-xl px-3 py-2 text-sm font-semibold transition-colors ${
                activeTab === tab.id ? 'bg-sky-100 text-sky-700' : 'text-slate-500 hover:bg-slate-100'
              }`}
            >
              {tab.label}
            </button>
          ))}
        </div>

        {activeTab === 'preview' && (
          <div data-testid="editor-structure-preview" className="min-h-0 flex-1 overflow-y-auto rounded-2xl bg-slate-900 p-4 font-mono text-xs leading-6 text-slate-200">
            {currentStructurePreview}
          </div>
        )}

        {activeTab === 'source' && (
          <div className="min-h-0 flex-1 overflow-y-auto rounded-2xl bg-slate-950 p-4 font-mono text-xs leading-6 text-slate-200">
            {sourcePreview}
          </div>
        )}

        {activeTab === 'info' && (
          <>
            {!selectedNode ? (
              <div className="flex flex-1 flex-col items-center justify-center text-slate-400">
                <Info className="mb-3 h-12 w-12 opacity-60" />
                <p>请在画布中选中一个节点以编辑</p>
              </div>
            ) : (
              <div className="min-h-0 flex flex-1 flex-col">
                <div className="mb-4 rounded-2xl border border-slate-200 bg-white/80 p-4">
                  <div className="flex items-start justify-between gap-3">
                    <div>
                      <h3 className="text-lg font-bold text-slate-800">{selectedNodeDisplay?.label}</h3>
                      <div className="mt-2 flex flex-wrap items-center gap-2 text-xs">
                        <span className="rounded-full bg-sky-100 px-2 py-1 font-semibold text-sky-700">
                          {getBehaviorTreeNodeCategoryLabel(selectedNode.nodeKind)}
                        </span>
                        <span className="rounded-full bg-slate-100 px-2 py-1 font-semibold text-slate-500">
                          {selectedNode.source === 'official' ? '官方节点' : selectedNode.source === 'robot' ? '机器人模块' : '未注册节点'}
                        </span>
                        {activeTree && (
                          <span className="rounded-full bg-violet-100 px-2 py-1 font-semibold text-violet-700">
                            {getBehaviorTreeTreeName(activeTree.id, activeTree.name)}
                          </span>
                        )}
                      </div>
                    </div>

                    <button
                      type="button"
                      onClick={() => deleteNode(selectedNode.id)}
                      className="rounded-xl p-2 text-rose-500 transition-colors hover:bg-rose-50"
                      title="删除节点"
                    >
                      <Trash2 className="h-5 w-5" />
                    </button>
                  </div>

                  <p className="mt-3 text-sm leading-6 text-slate-600">{selectedNodeDisplay?.desc}</p>
                </div>

                <div className="min-h-0 flex-1 overflow-y-auto pr-1">
                  {selectedDefinition && selectedDefinition.portSchemas.length > 0 && (
                    <section className="mb-4">
                      <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                        <Database className="h-4 w-4 text-emerald-600" />
                        端口与参数
                      </div>
                      <div className="space-y-3">{selectedDefinition.portSchemas.map(renderPortField)}</div>
                    </section>
                  )}

                  {extraAttributes.length > 0 && (
                    <section className="mb-4">
                      <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                        <Plus className="h-4 w-4 text-slate-500" />
                        附加属性
                      </div>
                      <div className="space-y-2">
                        {extraAttributes.map((attribute) => (
                          <div key={attribute.rawKey} className="rounded-2xl border border-slate-200 bg-white/80 p-3">
                            <div className="flex items-center justify-between gap-3">
                              <div className="text-sm font-semibold text-slate-700">{attribute.label}</div>
                              <button
                                type="button"
                                onClick={() => updateSingleAttribute(selectedNode.id, attribute.rawKey, '')}
                                className="rounded-lg px-2 py-1 text-xs font-semibold text-rose-500 transition-colors hover:bg-rose-50"
                              >
                                移除
                              </button>
                            </div>
                            <input
                              value={attribute.rawValue}
                              onChange={(event) => updateSingleAttribute(selectedNode.id, attribute.rawKey, event.target.value)}
                              className="mt-2 w-full rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                            />
                          </div>
                        ))}
                      </div>
                    </section>
                  )}

                  <section className="mb-4">
                    <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                      <Plus className="h-4 w-4 text-slate-500" />
                      添加附加属性
                    </div>
                    <div className="rounded-2xl border border-slate-200 bg-white/80 p-3">
                      <div className="flex gap-2">
                        <input
                          value={newKey}
                          onChange={(event) => setNewKey(event.target.value)}
                          placeholder="新属性键名"
                          data-testid="new-attribute-key"
                          className="flex-1 rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                        />
                        <input
                          value={newValue}
                          onChange={(event) => setNewValue(event.target.value)}
                          placeholder="新属性值"
                          data-testid="new-attribute-value"
                          className="flex-1 rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                        />
                      </div>
                      <button
                        type="button"
                        onClick={handleAddAttribute}
                        data-testid="add-attribute-button"
                        className="mt-3 w-full rounded-xl bg-slate-100 px-3 py-2 text-sm font-semibold text-slate-700 transition hover:bg-slate-200"
                      >
                        添加属性
                      </button>
                    </div>
                  </section>

                  <section className="mb-4">
                    <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                      <GitBranch className="h-4 w-4 text-sky-600" />
                      复合节点切换
                    </div>
                    <div className="flex flex-wrap gap-2">
                      {['Sequence', 'SequenceWithMemory', 'ReactiveSequence', 'Fallback', 'ReactiveFallback', 'Parallel', 'ParallelAll'].map((tag) => {
                        const definition = getBtNodeDefinition(tag);
                        if (!definition || selectedNode.tagName === tag) {
                          return null;
                        }
                        return (
                          <button
                            key={tag}
                            type="button"
                            onClick={() => replaceNodeType(selectedNode.id, tag)}
                            className="rounded-xl bg-sky-50 px-3 py-2 text-sm font-semibold text-sky-700 transition hover:bg-sky-100"
                          >
                            {definition.labelZh}
                          </button>
                        );
                      })}
                    </div>
                  </section>

                  <section className="mb-4">
                    <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                      <Layers3 className="h-4 w-4 text-violet-600" />
                      结构操作
                    </div>
                    <div className="rounded-2xl border border-slate-200 bg-white/80 p-3">
                      <label className="mb-2 block text-xs font-semibold text-slate-500">插入位置</label>
                      <select
                        value={insertPosition}
                        onChange={(event) =>
                          setInsertPosition(event.target.value as 'before' | 'after' | 'prepend_child' | 'append_child')
                        }
                        className="w-full rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                      >
                        <option value="before">在当前节点前插入</option>
                        <option value="after">在当前节点后插入</option>
                        <option value="prepend_child">作为首个子节点插入</option>
                        <option value="append_child">作为末尾子节点插入</option>
                      </select>

                      <label className="mb-2 mt-3 block text-xs font-semibold text-slate-500">节点模板</label>
                      <select
                        value={insertTagName}
                        onChange={(event) => setInsertTagName(event.target.value)}
                        className="w-full rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                      >
                        {registry.map((entry) => (
                          <option key={entry.tagName} value={entry.tagName}>
                            {entry.labelZh}
                          </option>
                        ))}
                      </select>

                      <button
                        type="button"
                        onClick={() => insertNode(selectedNode.id, insertPosition, insertTagName)}
                        className="mt-3 flex w-full items-center justify-center gap-2 rounded-xl bg-sky-600 px-3 py-2 text-sm font-semibold text-white transition hover:bg-sky-700"
                      >
                        <CornerDownRight className="h-4 w-4" />
                        执行插入
                      </button>

                      <label className="mb-2 mt-4 block text-xs font-semibold text-slate-500">包裹模板</label>
                      <select
                        value={wrapTagName}
                        onChange={(event) => setWrapTagName(event.target.value)}
                        className="w-full rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                      >
                        {['Inverter', 'RetryUntilSuccessful', 'Delay', 'ForceSuccess', 'ForceFailure', 'Sequence', 'Fallback', 'Parallel'].map((tag) => {
                          const definition = getBtNodeDefinition(tag);
                          return definition ? (
                            <option key={tag} value={tag}>
                              {definition.labelZh}
                            </option>
                          ) : null;
                        })}
                      </select>
                      <button
                        type="button"
                        onClick={() => insertNode(selectedNode.id, 'wrap', wrapTagName)}
                        className="mt-3 flex w-full items-center justify-center gap-2 rounded-xl bg-violet-600 px-3 py-2 text-sm font-semibold text-white transition hover:bg-violet-700"
                      >
                        <GitBranch className="h-4 w-4" />
                        执行包裹
                      </button>
                    </div>
                  </section>
                </div>
              </div>
            )}
          </>
        )}
      </div>
    </div>
  );
};
