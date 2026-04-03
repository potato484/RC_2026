import type { Node as FlowNode } from '@xyflow/react';
import dagre from 'dagre';
import { BTNode, ParsedTree, ParsedArea } from '../types';
import { getBehaviorTreeNodeDisplay, getBehaviorTreeTreeName, translateName } from './btDisplay';

function compressTree(nodes: BTNode[], edges: { id: string; source: string; target: string }[]) {
  let modified = true;
  while (modified) {
    modified = false;
    
    // 1. 压缩 Decorator 到其子节点
    const decorators = nodes.filter(n => n.type === 'decorator');
    for (const dec of decorators) {
      const outEdgeIndex = edges.findIndex(e => e.source === dec.id);
      if (outEdgeIndex !== -1) {
        const outEdge = edges[outEdgeIndex];
        const childIndex = nodes.findIndex(n => n.id === outEdge.target);
        if (childIndex !== -1) {
          const child = nodes[childIndex];
          
          if (!child.decorators) child.decorators = [];
          child.decorators.push({
            id: dec.id,
            type: dec.type,
            label: dec.label, // 这里的 label 已经是 traverse 中生成的包含参数的 label
            desc: dec.desc
          });
          
          const inEdge = edges.find(e => e.target === dec.id);
          if (inEdge) {
            inEdge.target = child.id;
          }
          child.parentId = dec.parentId;
          
          nodes.splice(nodes.findIndex(n => n.id === dec.id), 1);
          edges.splice(outEdgeIndex, 1);
          
          modified = true;
          break;
        }
      }
    }
  }

  // 重新计算 siblingIndex
  const childrenMap = new Map<string, BTNode[]>();
  nodes.forEach(n => {
    if (n.parentId) {
      if (!childrenMap.has(n.parentId)) childrenMap.set(n.parentId, []);
      childrenMap.get(n.parentId)!.push(n);
    }
  });
  
  childrenMap.forEach((children) => {
    // 假设原本的顺序大致保留
    children.forEach((child, index) => {
      child.siblingIndex = index + 1;
    });
  });
}

export function parseBTXml(xmlString: string, mainTreeId?: string): ParsedArea {
  const parser = new DOMParser();
  const xmlDoc = parser.parseFromString(xmlString, 'text/xml');
  const rootElement = xmlDoc.querySelector('root');
  
  if (!rootElement) throw new Error('Invalid BT XML');

  const treeElements = Array.from(rootElement.querySelectorAll('BehaviorTree'));
  if (treeElements.length === 0) return { mainTreeId: '', trees: {} };

  const parsedTrees: Record<string, ParsedTree> = {};
  
  let targetMainTreeId = mainTreeId;
  if (!targetMainTreeId) {
    const firstTreeId = treeElements[0].getAttribute('ID');
    targetMainTreeId = firstTreeId || 'unknown';
  }

  // 计算树之间的父子关系
  const treeHierarchy = new Map<string, string>(); // childId -> parentId
  treeElements.forEach(treeEl => {
    const parentId = treeEl.getAttribute('ID') || 'unknown';
    const subTrees = Array.from(treeEl.querySelectorAll('SubTree'));
    subTrees.forEach(sub => {
      const childId = sub.getAttribute('ID');
      if (childId) {
        treeHierarchy.set(childId, parentId);
      }
    });
  });

  treeElements.forEach((treeEl) => {
    const treeId = treeEl.getAttribute('ID') || 'unknown';
    const treeName = getBehaviorTreeTreeName(treeId, treeEl.getAttribute('name') || undefined);
    const nodes: BTNode[] = [];
    const edges: { id: string; source: string; target: string }[] = [];
    let idCounter = 0;

    // 预先建立所有行为树定义的索引，方便子树展开时查找
    const treeDefinitions = new Map<string, Element>();
    treeElements.forEach(el => {
      const id = el.getAttribute('ID');
      if (id) treeDefinitions.set(id, el);
    });

    function traverse(element: Element, parentId?: string, siblingIndex: number = 0, subtreeScope?: string) {
      if (element.nodeType !== Node.ELEMENT_NODE) return;
      
      const tagName = element.tagName;
      if (tagName === 'BehaviorTree') {
        const children = Array.from(element.children);
        children.forEach((child, idx) => traverse(child, parentId, idx + 1, subtreeScope));
        return;
      }

      const id = `${treeId}_${idCounter++}`; // 简化并确保 ID 唯一
      
      let type: BTNode['type'] = 'action';
      if (['Sequence', 'ReactiveSequence'].includes(tagName)) type = 'sequence';
      else if (['Fallback', 'ReactiveFallback'].includes(tagName)) type = 'selector';
      else if (['Inverter', 'ForceSuccess', 'ForceFailure', 'Repeat', 'RetryUntilSuccessful', 'KeepRunningUntilFailure', 'Delay'].includes(tagName)) type = 'decorator';
      else if (tagName === 'SubTree') type = 'subtree';

      const attributes = Array.from(element.attributes).reduce<Record<string, string>>((acc, attr) => {
        acc[attr.name] = attr.value;
        return acc;
      }, {});
      const { label, desc } = getBehaviorTreeNodeDisplay(tagName, attributes);

      const node: BTNode = {
        id,
        type,
        label,
        state: 'idle',
        desc,
        parentId,
        siblingIndex,
        treeId: treeId,
        subTreeId: subtreeScope
      };

      nodes.push(node);

      if (parentId) {
        edges.push({ id: `e-${parentId}-${id}`, source: parentId, target: id });
      }

      // 递归处理子节点
      if (tagName === 'SubTree') {
        const subId = element.getAttribute('ID');
        const subDef = subId ? treeDefinitions.get(subId) : null;
        if (subDef) {
          const subName = translateName(subId || '', subId || '');
          const subChildren = Array.from(subDef.children);
          // 如果子树定义里只有一个顶级 Sequence/Fallback，直接展开其内容到 SubTree 节点下
          if (subChildren.length === 1 && (subChildren[0].tagName === 'Sequence' || subChildren[0].tagName === 'Fallback')) {
            const containerChildren = Array.from(subChildren[0].children);
            containerChildren.forEach((child, idx) => traverse(child, id, idx + 1, subName));
          } else {
            subChildren.forEach((child, idx) => traverse(child, id, idx + 1, subName));
          }
        }
      } else {
        Array.from(element.children).forEach((child, idx) => traverse(child, id, idx + 1, subtreeScope));
      }
    }

    traverse(treeEl);
    
    // 应用压缩逻辑，把装饰器和条件挂载到实际执行节点或父节点上
    compressTree(nodes, edges);
    
    parsedTrees[treeId] = { 
      id: treeId, 
      name: treeName, 
      nodes, 
      edges,
      parentTreeId: treeHierarchy.get(treeId)
    };
  });

  return { mainTreeId: targetMainTreeId, trees: parsedTrees };
}

export function layoutNodes(nodes: BTNode[], edges: { id: string; source: string; target: string }[]) {
  const dagreGraph = new dagre.graphlib.Graph();
  dagreGraph.setDefaultEdgeLabel(() => ({}));
  
  // 优化间距：调整布局参数适应横向布局 (LR = Left to Right)
  dagreGraph.setGraph({ 
    rankdir: 'LR', // 从左到右布局
    nodesep: 80,   // 节点间的垂直间距
    ranksep: 160,  // 大幅拉大层级间的水平间距，给连线曲线留出呼吸空间
    edgesep: 20,
    marginx: 20,
    marginy: 20
  });

  const getNodeSize = (type: string) => {
    switch (type) {
      case 'sequence':
      case 'selector':
        return { width: 140, height: 48 };
      case 'decorator':
        return { width: 180, height: 48 };
      case 'action':
      case 'subtree':
      default:
        return { width: 240, height: 80 };
    }
  };

  nodes.forEach((node) => {
    const { height } = getNodeSize(node.type);
    // 使用统一的最大宽度 (240) 让 dagre 计算布局，
    // 这样在同一层级的节点会基于同一个中心点对齐，从而保证左边缘完全对齐
    dagreGraph.setNode(node.id, { width: 240, height });
  });

  edges.forEach((edge) => {
    dagreGraph.setEdge(edge.source, edge.target);
  });

  dagre.layout(dagreGraph);

  const flowNodes: FlowNode[] = nodes.map((node) => {
    const nodeWithPosition = dagreGraph.node(node.id);
    const { height } = getNodeSize(node.type);
    return {
      id: node.id,
      type: 'custom',
      position: {
        // 强制所有节点基于统一的 240 宽度来计算左边界，实现同层级节点严格左对齐
        x: nodeWithPosition.x - 120,
        y: nodeWithPosition.y - height / 2,
      },
      data: { ...node },
    };
  });

  return flowNodes;
}
