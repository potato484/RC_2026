
import { useMemo, useCallback } from 'react';
import { ReactFlow, Background, Edge, Node, MarkerType } from '@xyflow/react';
import '@xyflow/react/dist/style.css';
import { useStore } from '../store/useStore';
import { CustomNode } from './CustomNode';
import { layoutNodes } from '../utils/btParser';

const nodeTypes = {
  custom: CustomNode,
};

export const TreeVisualizer = () => {
  const { nodes, edges, setActiveNode, toggleNodeCollapse } = useStore();

  const { visibleNodes, visibleEdges } = useMemo(() => {
    // 找出所有被折叠的节点 ID
    const collapsedIds = new Set<string>();
    
    // 递归查找所有应该被隐藏的子节点
    const hiddenIds = new Set<string>();
    
    const findHidden = (parentId: string) => {
      const childEdges = edges.filter(e => e.source === parentId);
      childEdges.forEach(edge => {
        hiddenIds.add(edge.target);
        findHidden(edge.target);
      });
    };

    nodes.forEach(node => {
      if (node.collapsed) {
        collapsedIds.add(node.id);
        findHidden(node.id);
      }
    });

    const vNodes = nodes.filter(n => !hiddenIds.has(n.id));
    const vEdges = edges.filter(e => !hiddenIds.has(e.target));
    
    return { visibleNodes: vNodes, visibleEdges: vEdges };
  }, [nodes, edges]);

  const flowNodes: Node[] = useMemo(() => {
    return layoutNodes(visibleNodes, visibleEdges);
  }, [visibleNodes, visibleEdges]);

  const flowEdges: Edge[] = useMemo(() => visibleEdges.map(edge => {
    const isRunning = nodes.find(n => n.id === edge.target)?.state === 'running';
    return {
      id: edge.id,
      source: edge.source,
      target: edge.target,
      animated: isRunning,
      style: { stroke: isRunning ? '#f59e0b' : '#94a3b8', strokeWidth: isRunning ? 3 : 2 },
      markerEnd: {
        type: MarkerType.ArrowClosed,
        width: 15,
        height: 15,
        color: isRunning ? '#f59e0b' : '#94a3b8',
      },
    };
  }), [nodes, visibleEdges]);

  const onNodeDoubleClick = useCallback((_: React.MouseEvent, node: Node) => {
    toggleNodeCollapse(node.id);
  }, [toggleNodeCollapse]);

  return (
    <div className="w-full h-full glass-panel overflow-hidden bg-white/40">
      <ReactFlow
        nodes={flowNodes}
        edges={flowEdges}
        nodeTypes={nodeTypes}
        onNodeClick={(_, node) => setActiveNode(node.id)}
        onNodeDoubleClick={onNodeDoubleClick}
        onPaneClick={() => setActiveNode(null)}
        fitView
        minZoom={0.1}
        className="bg-transparent"
      >
        <Background color="#cbd5e1" gap={24} size={2} className="opacity-50" />
      </ReactFlow>
    </div>
  );
};
