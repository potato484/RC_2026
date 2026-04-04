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
} from '../utils/btDisplay';
import { buildAlongBranchInsertCatalog, getInsertCategoryLabel } from '../utils/editorInsertCatalog';
import { buildEditorTreePreview } from '../utils/editorTreeView';
import {
  canNodeAddBranch,
  getBtNodeDefinition,
  getBtNodeRegistry,
  getAlongBranchWrapperEntries,
  getNodeChildPolicy,
} from '../utils/btRegistry';

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

const numericLiteralValueTypes = new Set(['int', 'double', 'float', 'uint16']);

const isVisibleNumericPort = (port: BtPortSchema): boolean => {
  const normalizedType = port.valueType.trim().toLowerCase();
  const bindingMode = port.bindingMode ?? 'literal';
  return port.direction === 'input' && bindingMode === 'literal' && numericLiteralValueTypes.has(normalizedType);
};

const getNumericInputMode = (valueType: string): 'numeric' | 'decimal' => {
  const normalizedType = valueType.trim().toLowerCase();
  return normalizedType === 'double' || normalizedType === 'float' ? 'decimal' : 'numeric';
};

export const EditorRightPanel = () => {
  const {
    selectedNodeId,
    document,
    activeTreeId,
    updateRegisteredAttribute,
    deleteNode,
    exportXml,
    insertAlongBranch,
    insertBranch,
    wrapNode,
  } = useEditorStore();

  const [activeTab, setActiveTab] = useState<'info' | 'preview' | 'source'>('info');
  const [insertPosition, setInsertPosition] = useState<'before' | 'after'>('after');
  const [alongBranchItemId, setAlongBranchItemId] = useState('');
  const [alongBranchWrapperTagName, setAlongBranchWrapperTagName] = useState('');
  const [branchInsertTagName, setBranchInsertTagName] = useState('Sequence');
  const [wrapTagName, setWrapTagName] = useState('Inverter');
  const [branchInsertIndex, setBranchInsertIndex] = useState(0);

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
    setBranchInsertIndex(selectedNode?.children.length ?? 0);
  }, [selectedNodeId, selectedNode?.children.length]);

  const registry = useMemo(() => getBtNodeRegistry(), []);
  const alongBranchWrapperOptions = useMemo(() => getAlongBranchWrapperEntries(), []);
  const alongBranchItems = useMemo(
    () => buildAlongBranchInsertCatalog(document, activeTreeId).flatMap((section) => section.items),
    [activeTreeId, document]
  );
  const selectedNodeDisplay = selectedNode ? getBehaviorTreeNodeDisplay(selectedNode.tagName, selectedNode.attributes) : null;
  const selectedDefinition = selectedNode ? getBtNodeDefinition(selectedNode.tagName) : undefined;
  const sourcePreview = exportXml() ?? '当前没有可导出的源文件内容';
  const currentStructurePreview = document ? buildEditorTreePreview(document, activeTreeId) : '当前没有可预览的结构';
  const attributeDisplays = selectedNode ? getBehaviorTreeAttributeDisplays(selectedNode.attributes, selectedNode.tagName) : [];
  const visiblePortSchemas = useMemo(
    () => selectedDefinition?.portSchemas.filter(isVisibleNumericPort) ?? [],
    [selectedDefinition]
  );
  const registeredAttributeNames = new Set(selectedDefinition?.portSchemas.map((port) => port.name) ?? []);
  const readonlyAttributes = attributeDisplays.filter(
    (attribute) => !registeredAttributeNames.has(attribute.rawKey)
  );
  const isRootSelected = Boolean(selectedNode && activeTree && activeTree.rootNode.id === selectedNode.id);
  const childPolicy = selectedNode ? getNodeChildPolicy(selectedNode.tagName) : null;
  const isBranchContainer = Boolean(
    selectedNode &&
      selectedNode.nodeKind === 'control' &&
      childPolicy &&
      (childPolicy.max === null || childPolicy.max > 1)
  );
  const branchLimitLabel =
    childPolicy?.max === null ? '无上限' : childPolicy ? `最多 ${childPolicy.max} 条` : '';
  const canAddBranch = Boolean(
    selectedNode &&
      isBranchContainer &&
      canNodeAddBranch(selectedNode, selectedNode.children.length)
  );

  const branchInsertOptions = selectedNode
    ? Array.from({ length: selectedNode.children.length + 1 }, (_, index) => ({
        value: index,
        label:
          index === 0
            ? '插入到最前面'
            : index === selectedNode.children.length
              ? `追加到第 ${index} 条分支后`
              : `插入到第 ${index} 条分支后`,
      }))
    : [];
  const selectedAlongBranchItem = alongBranchItems.find((item) => item.id === alongBranchItemId) ?? null;

  useEffect(() => {
    if (alongBranchItems.length === 0) {
      setAlongBranchItemId('');
      return;
    }

    if (!alongBranchItems.some((item) => item.id === alongBranchItemId)) {
      setAlongBranchItemId(alongBranchItems[0].id);
    }
  }, [alongBranchItemId, alongBranchItems]);

  useEffect(() => {
    if (registry.length === 0) {
      setBranchInsertTagName('');
      return;
    }

    if (!registry.some((entry) => entry.tagName === branchInsertTagName)) {
      setBranchInsertTagName(registry[0].tagName);
    }
  }, [branchInsertTagName, registry]);

  const renderPortField = (port: BtPortSchema) => {
    if (!selectedNode) {
      return null;
    }

    const currentValue = selectedNode.attributes[port.name] ?? port.defaultValue ?? '';
    const placeholder = port.defaultValue ? `默认值：${port.defaultValue}` : '请输入数值';

    return (
      <div
        key={port.name}
        data-testid={`editor-port-field-${port.name}`}
        className="rounded-2xl border border-slate-200 bg-white/80 p-3"
      >
        <div className="flex items-start justify-between gap-3">
          <div>
            <div className="text-sm font-semibold text-slate-800">{port.labelZh}</div>
            <div className="mt-1 text-xs leading-5 text-slate-500">{port.descriptionZh}</div>
          </div>
          <span className="rounded-full bg-slate-100 px-2 py-1 text-[10px] font-semibold text-slate-500">
            {port.direction === 'input' ? '输入' : port.direction === 'output' ? '输出' : '双向'}
          </span>
        </div>

        <div className="mt-3">
          <input
            value={currentValue}
            onChange={(event) =>
              updateRegisteredAttribute(selectedNode.id, port.name, event.target.value)
            }
            data-testid={`editor-port-value-${port.name}`}
            inputMode={getNumericInputMode(port.valueType)}
            placeholder={placeholder}
            className="w-full rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none placeholder:text-slate-400"
          />
        </div>
      </div>
    );
  };

  return (
    <div className="flex h-full w-full flex-col gap-4 lg:ml-4 lg:max-w-[420px]">
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
          <pre
            data-testid="editor-structure-preview"
            className="min-h-0 flex-1 overflow-auto rounded-2xl bg-slate-900 p-4 font-mono text-xs leading-6 text-slate-200 whitespace-pre-wrap break-words"
          >
            {currentStructurePreview}
          </pre>
        )}

        {activeTab === 'source' && (
          <pre
            data-testid="editor-source-preview"
            className="min-h-0 flex-1 overflow-auto rounded-2xl bg-slate-950 p-4 font-mono text-xs leading-6 text-slate-200 whitespace-pre"
          >
            {sourcePreview}
          </pre>
        )}

        {activeTab === 'info' && (
          <>
            {!selectedNode ? (
              <div className="flex flex-1 flex-col items-center justify-center text-slate-400">
                <Info className="mb-3 h-12 w-12 opacity-60" />
                <p>请在画布中选中一个节点以查看说明或配置数值参数</p>
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
                        <span className="rounded-full bg-white px-2 py-1 font-semibold text-slate-500 ring-1 ring-slate-200">
                          {selectedNode.tagName}
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
                      disabled={isRootSelected}
                      className="rounded-xl p-2 text-rose-500 transition-colors hover:bg-rose-50 disabled:cursor-not-allowed disabled:opacity-40"
                      title={isRootSelected ? '根节点不能直接删除' : '删除节点'}
                    >
                      <Trash2 className="h-5 w-5" />
                    </button>
                  </div>

                  <p className="mt-3 text-sm leading-6 text-slate-600">{selectedNodeDisplay?.desc}</p>
                </div>

                <div className="min-h-0 flex-1 overflow-y-auto pr-1">
                  {visiblePortSchemas.length > 0 && (
                    <section className="mb-4">
                      <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                        <Database className="h-4 w-4 text-emerald-600" />
                        已定义参数
                      </div>
                      <div className="mb-2 text-xs leading-5 text-slate-500">
                        这里只显示节点定义里真正需要手改的数值参数，不再展示字符串、布尔或黑板绑定类端口。
                      </div>
                      <div className="space-y-3">{visiblePortSchemas.map(renderPortField)}</div>
                    </section>
                  )}

                  {readonlyAttributes.length > 0 && (
                    <section className="mb-4">
                      <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                        <Info className="h-4 w-4 text-slate-500" />
                        保留的原始属性
                      </div>
                      <div className="mb-2 text-xs leading-5 text-slate-500">
                        这些属性来自原始 XML，但当前节点定义没有把它们声明为可编辑参数，因此这里只读展示并在导出时原样保留。
                      </div>
                      <div className="space-y-2">
                        {readonlyAttributes.map((attribute) => (
                          <div key={attribute.rawKey} className="rounded-2xl border border-slate-200 bg-white/80 p-3">
                            <div className="flex items-center justify-between gap-3">
                              <div className="text-sm font-semibold text-slate-700">{attribute.label}</div>
                              <span className="rounded-full bg-slate-100 px-2 py-1 text-[10px] font-semibold text-slate-500">
                                只读
                              </span>
                            </div>
                            <div className="mt-2 rounded-xl border border-slate-100 bg-slate-50 px-3 py-2 text-sm text-slate-600">
                              {attribute.rawValue || '空'}
                            </div>
                          </div>
                        ))}
                      </div>
                    </section>
                  )}

                  <section className="mb-4">
                    <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                      <Layers3 className="h-4 w-4 text-violet-600" />
                      同支线插入
                    </div>
                    <div className="rounded-2xl border border-slate-200 bg-white/80 p-3">
                      <div className="mb-3 text-xs leading-5 text-slate-500">
                        前插和后插都会先显式新建一层控制包装，再把“原节点 + 新节点”串成同一条执行链。这里只插动作、条件或子树；新增支线请走下面的独立入口。
                      </div>

                      <label className="mb-2 block text-xs font-semibold text-slate-500">插入位置</label>
                      <select
                        value={insertPosition}
                        onChange={(event) => setInsertPosition(event.target.value as 'before' | 'after')}
                        className="w-full rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                      >
                        <option value="before">在当前节点前插入</option>
                        <option value="after">在当前节点后插入</option>
                      </select>

                      <label className="mb-2 mt-3 block text-xs font-semibold text-slate-500">控制包装</label>
                      <select
                        value={alongBranchWrapperTagName}
                        onChange={(event) => setAlongBranchWrapperTagName(event.target.value)}
                        data-testid="editor-along-branch-wrapper-select"
                        className="w-full rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                      >
                        <option value="">先选择顺序 / 回退 / 并行等控制关系</option>
                        {alongBranchWrapperOptions.map((entry) => (
                          <option key={entry.tagName} value={entry.tagName}>
                            {entry.labelZh}
                          </option>
                        ))}
                      </select>

                      <label className="mb-2 mt-3 block text-xs font-semibold text-slate-500">节点模板</label>
                      <select
                        value={alongBranchItemId}
                        onChange={(event) => setAlongBranchItemId(event.target.value)}
                        data-testid="editor-along-branch-template-select"
                        className="w-full rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                      >
                        {alongBranchItems.map((item) => (
                          <option key={item.id} value={item.id}>
                            {item.label} · {getInsertCategoryLabel(item.category)}
                          </option>
                        ))}
                      </select>

                      <button
                        type="button"
                        onClick={() => {
                          if (!selectedAlongBranchItem || !alongBranchWrapperTagName) {
                            return;
                          }

                          insertAlongBranch(selectedNode.id, {
                            position: insertPosition,
                            wrapperTagName: alongBranchWrapperTagName,
                            template: selectedAlongBranchItem.template,
                          });
                        }}
                        disabled={!selectedAlongBranchItem || !alongBranchWrapperTagName}
                        className="mt-3 flex w-full items-center justify-center gap-2 rounded-xl bg-sky-600 px-3 py-2 text-sm font-semibold text-white transition hover:bg-sky-700 disabled:cursor-not-allowed disabled:bg-sky-300"
                      >
                        <CornerDownRight className="h-4 w-4" />
                        执行同支线插入
                      </button>
                    </div>
                  </section>

                  {isBranchContainer && (
                    <section className="mb-4">
                      <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                        <Plus className="h-4 w-4 text-emerald-600" />
                        新增支线
                      </div>
                      <div className="rounded-2xl border border-slate-200 bg-white/80 p-3">
                        <div className="mb-3 text-xs leading-5 text-slate-500">
                          这里只显式给当前控制节点新增一条 child 分支，不承担沿现有支线串接逻辑的职责。
                        </div>

                        <div className="mb-3 rounded-2xl border border-dashed border-slate-200 bg-slate-50/80 p-3">
                          <div className="flex items-center justify-between gap-3">
                            <div className="text-xs font-semibold text-slate-500">当前分支</div>
                            <div className="text-[11px] text-slate-400">
                              {selectedNode.children.length} 条
                              {branchLimitLabel ? ` / ${branchLimitLabel}` : ''}
                            </div>
                          </div>

                          <div className="mt-2 space-y-2">
                            {selectedNode.children.length === 0 ? (
                              <div className="rounded-xl bg-white px-3 py-2 text-sm text-slate-500">
                                当前还没有分支，可直接新增第 1 条分支。
                              </div>
                            ) : (
                              selectedNode.children.map((child, index) => {
                                const childDisplay = getBehaviorTreeNodeDisplay(child.tagName, child.attributes);
                                return (
                                  <div key={child.id} className="rounded-xl bg-white px-3 py-2 text-sm text-slate-600">
                                    <span className="font-semibold text-slate-700">第 {index + 1} 条</span>
                                    {' · '}
                                    {childDisplay.label}
                                  </div>
                                );
                              })
                            )}
                          </div>
                        </div>

                        <label className="mb-2 block text-xs font-semibold text-slate-500">插入位置</label>
                        <select
                          value={branchInsertIndex}
                          onChange={(event) => setBranchInsertIndex(Number(event.target.value))}
                          className="w-full rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-700 outline-none"
                        >
                          {branchInsertOptions.map((option) => (
                            <option key={option.value} value={option.value}>
                              {option.label}
                            </option>
                          ))}
                        </select>

                        <label className="mb-2 mt-3 block text-xs font-semibold text-slate-500">节点模板</label>
                        <select
                          value={branchInsertTagName}
                          onChange={(event) => setBranchInsertTagName(event.target.value)}
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
                          onClick={() => insertBranch(selectedNode.id, branchInsertIndex, branchInsertTagName)}
                          disabled={!canAddBranch}
                          className="mt-3 flex w-full items-center justify-center gap-2 rounded-xl bg-emerald-600 px-3 py-2 text-sm font-semibold text-white transition hover:bg-emerald-700 disabled:cursor-not-allowed disabled:bg-emerald-300"
                        >
                          <Plus className="h-4 w-4" />
                          新增分支
                        </button>

                        {!canAddBranch && (
                          <div className="mt-2 text-xs leading-5 text-amber-700">
                            当前控制节点已达到分支上限，不能继续新增支线。
                          </div>
                        )}
                      </div>
                    </section>
                  )}

                  <section className="mb-4">
                    <div className="mb-2 flex items-center gap-2 text-sm font-bold text-slate-700">
                      <GitBranch className="h-4 w-4 text-violet-600" />
                      包裹当前节点
                    </div>
                    <div className="rounded-2xl border border-slate-200 bg-white/80 p-3">
                      <label className="mb-2 block text-xs font-semibold text-slate-500">包裹模板</label>
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
                        onClick={() => wrapNode(selectedNode.id, wrapTagName)}
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
