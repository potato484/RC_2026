import { useMemo } from 'react';
import { ReactFlow, Background, Edge, Node } from '@xyflow/react';
import '@xyflow/react/dist/style.css';
import { useStore } from '../store/useStore';
import { CustomNode } from './CustomNode';
import { nodePositions, mockEdges } from '../mock/treeData';

const nodeTypes = {
  custom: CustomNode,
};

export const TreeVisualizer = () => {
  const { nodes, setActiveNode } = useStore();

  const flowNodes: Node[] = useMemo(() => nodes.map(node => ({
    id: node.id,
    type: 'custom',
    position: nodePositions[node.id as keyof typeof nodePositions] || { x: 0, y: 0 },
    data: { ...node } as Record<string, unknown>,
  })), [nodes]);

  const flowEdges: Edge[] = useMemo(() => mockEdges.map(edge => {
    const isRunning = nodes.find(n => n.id === edge.target)?.state === 'running';
    return {
      id: edge.id,
      source: edge.source,
      target: edge.target,
      animated: isRunning,
      style: { stroke: isRunning ? '#3b82f6' : '#cbd5e1', strokeWidth: 4 },
    };
  }), [nodes]);

  return (
    <div className="w-full h-full glass-panel overflow-hidden">
      <ReactFlow
        nodes={flowNodes}
        edges={flowEdges}
        nodeTypes={nodeTypes}
        onNodeClick={(_, node) => setActiveNode(node.id)}
        onPaneClick={() => setActiveNode(null)}
        fitView
        className="bg-transparent"
      >
        <Background color="#cbd5e1" gap={20} size={2} />
      </ReactFlow>
    </div>
  );
};
