import type { Node as FlowNode } from '@xyflow/react';
import dagre from 'dagre';
import { BTNode, ParsedTree, ParsedArea } from '../types';

const actionTranslations: Record<string, { label: string, desc: string }> = {
  'NavToSmartPoint': { label: '导航到点', desc: '控制底盘移动到指定的预设智能点' },
  'NavToMerlinGrid': { label: '导航到梅林格', desc: '根据九宫格编号移动底盘到指定格' },
  'SetNavMode': { label: '设置导航模式', desc: '切换导航的安全/穿越/正常模式' },
  'ScanSurroundings': { label: '扫描周围环境', desc: '启动感知节点，寻找目标环或障碍' },
  'CheckR1Blocking': { label: '检测R1阻挡', desc: '检查路径上是否有我方R1机器人挡路' },
  'SelectNextGrid': { label: '选择下一格', desc: '根据地图信息决策下一步要去哪个梅林格' },
  'GrabKFS': { label: '抓取兑换块', desc: '伸出机械臂抓取面前的KFS（块）' },
  'IncrementKFSCount': { label: '增加计数', desc: '更新已抓取KFS的数量状态' },
  'UpdateMapKFS': { label: '更新地图', desc: '标记该格子上的KFS已被取走' },
  'CheckExitCondition': { label: '检查退出条件', desc: '判断是否已经抓满指定数量的KFS' },
  'StairDescend': { label: '下台阶', desc: '执行下台阶的控制序列' },
  'Delay': { label: '延迟等待', desc: '暂停执行一段时间' },
  'AlwaysSuccess': { label: '强制放行 (忽略错误)', desc: '无论如何都返回成功，通常用于忽略非致命错误' },
  'ScriptCondition': { label: '脚本条件判断', desc: '执行黑板脚本以判断条件真假' },
  'Script': { label: '执行脚本', desc: '更新黑板变量' },
  'FollowManualRobot': { label: '跟随手动机器人', desc: '使用传感器跟随前方的手动机器人' },
  'MechUpDuel': { label: '机械臂升起', desc: '将机械臂升起到对抗高度' },
  'PlaceKFSGrid': { label: '放置兑换块', desc: '在指定格子放下KFS' },
  'WaitUntilTrigger': { label: '等待触发', desc: '等待特定条件或外部信号触发' },
  'GrabTip': { label: '抓取矛头', desc: '抓取矛头准备组装' },
  'CheckManualRobot': { label: '检测手动机器人', desc: '检查手动机器人是否在可组装范围内' },
  'AssembleWeapon': { label: '组装武器', desc: '将部件进行组装动作' }
};

const paramTranslations: Record<string, string> = {
  'MF_SAFE': '梅林安全模式',
  'MF_TRAVERSE': '梅林穿越模式',
  'MF_EXIT': '梅林退出模式',
  'NORMAL': '正常模式',
  'mf_entry': '梅林入口点',
  'mf_entry_back': '梅林入口退避点',
  'mf_grid_2': '梅林2号格',
  'mf_exit': '梅林出口点',
  '{target_grid}': '目标格子变量',
  '{next_action}': '下一动作变量',
};

const translateParam = (param: string) => {
  if (!param) return param;
  if (paramTranslations[param]) return paramTranslations[param];
  if (param.includes('next_action==\'GRAB\'')) return '判断是否去抓取';
  if (param.includes('next_action==\'MOVE\'')) return '判断是否去移动';
  if (param.includes('target_kfs_count:=2')) return '初始化变量(2个块)';
  if (param.includes('current_grid:=')) return `设当前格为${param.split(':=')[1]}`;
  return param;
};

const translateName = (name: string, fallbackLabel: string) => {
  if (!name) return fallbackLabel;
  let translated = name;
  
  const exactMatches: Record<string, string> = {
    'Combat_Sequence': '对抗区主流程',
    'goto_combat': '前往对抗区',
    'place_sequence': '放置流程',
    'grab_tip': '抓取矛头',
    'assemble': '组装',
    'Entry_Seq': '进门流程',
    'Loop_Body': '循环体',
    'GrabKFSSeq': '抓取兑换块流程',
    'MoveToGridSeq': '移动至目标格流程',
    'Exit_Seq': '出门流程',
    'MC_Sequence': '武馆区主流程',
    'MFAreaTree': '梅林区树',
    'MF_Entry': '梅林进门',
    'MF_Loop': '梅林循环',
    'MF_Exit': '梅林出门',
    'MF_Main': '梅林主流程',
    'CombatAreaTree': '对抗区树',
    'MCAreaTree': '武馆区树'
  };

  if (exactMatches[translated]) return exactMatches[translated];

  translated = translated.replace(/ReactiveSequence/g, '自适应顺序流程');
  translated = translated.replace(/ReactiveFallback/g, '自适应备选流程');
  translated = translated.replace(/Sequence/g, '顺序流程');
  translated = translated.replace(/Seq/g, '流程');
  translated = translated.replace(/Main/g, '主流程');
  translated = translated.replace(/Entry/g, '进门');
  translated = translated.replace(/Loop/g, '循环');
  translated = translated.replace(/Exit/g, '出门');
  translated = translated.replace(/Grab/g, '抓取');
  translated = translated.replace(/Move/g, '移动');
  translated = translated.replace(/Init/g, '初始化');
  translated = translated.replace(/Body/g, '主体');
  translated = translated.replace(/MF_/g, '梅林区_');
  translated = translated.replace(/Combat_/g, '对抗区_');
  translated = translated.replace(/MC_/g, '武馆区_');
  
  // If it still contains english letters, return the fallback label (the pure action name)
  if (/[a-zA-Z]/.test(translated)) {
    return fallbackLabel || translated;
  }
  
  return translated;
};

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
            label: dec.label,
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
    if (modified) continue;

    // 2. 压缩 Condition 到其父节点（通常是控制流节点）
    const conditions = nodes.filter(n => n.type === 'condition');
    for (const cond of conditions) {
      const inEdgeIndex = edges.findIndex(e => e.target === cond.id);
      if (inEdgeIndex !== -1) {
        const inEdge = edges[inEdgeIndex];
        const parentIndex = nodes.findIndex(n => n.id === inEdge.source);
        if (parentIndex !== -1) {
          const parent = nodes[parentIndex];
          
          if (!parent.conditions) parent.conditions = [];
          // 如果条件节点上有被压缩的装饰器，也一并带过去
          const decoratorPrefix = cond.decorators ? cond.decorators.map(d => `[${d.label}] `).join('') : '';
          
          parent.conditions.push({
            id: cond.id,
            type: cond.type,
            label: decoratorPrefix + cond.label,
            desc: cond.desc
          });
          
          nodes.splice(nodes.findIndex(n => n.id === cond.id), 1);
          edges.splice(inEdgeIndex, 1);
          
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

  treeElements.forEach((treeEl) => {
    const treeId = treeEl.getAttribute('ID') || 'unknown';
    const treeNameAttr = treeEl.getAttribute('name') || treeId;
    const treeName = translateName(treeNameAttr, treeNameAttr);
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
      const nameAttr = element.getAttribute('name');
      const idAttr = element.getAttribute('ID');
      
      let type: BTNode['type'] = 'action';
      if (['Sequence', 'ReactiveSequence'].includes(tagName)) type = 'sequence';
      else if (['Fallback', 'ReactiveFallback'].includes(tagName)) type = 'selector';
      else if (['Inverter', 'ForceSuccess', 'ForceFailure', 'Repeat', 'RetryUntilSuccessful', 'KeepRunningUntilFailure', 'Delay'].includes(tagName)) type = 'decorator';
      else if (tagName === 'SubTree') type = 'subtree';
      else if (tagName.includes('Condition') || tagName.startsWith('Check')) type = 'condition';

      let rawLabel = nameAttr || idAttr || tagName;
      let fallbackActionLabel = actionTranslations[tagName]?.label || '';
      
      const code = element.getAttribute('code');
      if (tagName === 'Script' && code) {
        fallbackActionLabel = translateParam(code);
      } else if (tagName === 'ScriptCondition' && code) {
        fallbackActionLabel = `检查: ${translateParam(code)}`;
      }

      if (!fallbackActionLabel) {
         if (tagName === 'Sequence') fallbackActionLabel = '顺序流程';
         else if (tagName === 'ReactiveSequence') fallbackActionLabel = '自适应顺序';
         else if (tagName === 'Fallback') fallbackActionLabel = '备选流程';
         else if (tagName === 'ReactiveFallback') fallbackActionLabel = '自适应备选';
         else if (tagName === 'SubTree') fallbackActionLabel = translateName(idAttr || '', idAttr || '');
         else if (tagName === 'RetryUntilSuccessful') fallbackActionLabel = '一直重试';
         else if (tagName === 'KeepRunningUntilFailure') fallbackActionLabel = '死循环 (直到出错)';
         else if (tagName === 'Inverter') fallbackActionLabel = '结果取反 (非)';
         else if (tagName === 'ForceFailure') fallbackActionLabel = '必定失败 (拦截拦截)';
         else fallbackActionLabel = '未知操作';
      }

      let label = translateName(rawLabel, fallbackActionLabel);
      if ((label === 'Sequence' || label === 'Fallback' || (idAttr && label === idAttr)) && fallbackActionLabel) {
        label = fallbackActionLabel;
      }
      
      let desc = '';
      if (actionTranslations[tagName]) {
        desc = actionTranslations[tagName].desc;
        const mode = element.getAttribute('mode');
        const targetName = element.getAttribute('target_name');
        const delay = element.getAttribute('delay_msec');
        const codeAttr = element.getAttribute('code');
        const gridId = element.getAttribute('grid_id') || element.getAttribute('grid_position');
        const distance = element.getAttribute('follow_distance');
        const staticTime = element.getAttribute('static_time');
        const distanceThreshold = element.getAttribute('distance_threshold');
        
        if (mode) desc += ` [模式: ${translateParam(mode)}]`;
        if (targetName) desc += ` [目标: ${translateParam(targetName)}]`;
        if (gridId) desc += ` [目标格: ${translateParam(gridId)}]`;
        if (distance) desc += ` [距离: ${distance}米]`;
        if (distanceThreshold) desc += ` [距离阈值: ${distanceThreshold}米]`;
        if (delay) desc += ` [等待: ${delay}毫秒]`;
        if (staticTime) desc += ` [静止时间: ${staticTime}秒]`;
        if (codeAttr) desc += ` [操作: ${translateParam(codeAttr)}]`;
      } else {
        if (tagName === 'Sequence') desc = '顺序节点 (从左到右依次执行，必须全部成功)';
        else if (tagName === 'ReactiveSequence') desc = '自适应顺序节点 (每步都会重新检查前面已完成的步骤)';
        else if (tagName === 'Fallback') desc = '备选方案节点 (只要有一个成功就停止)';
        else if (tagName === 'ReactiveFallback') desc = '自适应备选节点 (持续监测高优先级条件)';
        else if (tagName === 'SubTree') desc = `${label}`;
        if (!desc) desc = `类型为 ${tagName} 的控制节点`;
      }

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
    
    parsedTrees[treeId] = { id: treeId, name: treeName, nodes, edges };
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
      case 'condition':
      case 'decorator':
        return { width: 180, height: 48 };
      case 'action':
      case 'subtree':
      default:
        return { width: 240, height: 80 };
    }
  };

  nodes.forEach((node) => {
    const { width, height } = getNodeSize(node.type);
    dagreGraph.setNode(node.id, { width, height });
  });

  edges.forEach((edge) => {
    dagreGraph.setEdge(edge.source, edge.target);
  });

  dagre.layout(dagreGraph);

  const flowNodes: FlowNode[] = nodes.map((node) => {
    const nodeWithPosition = dagreGraph.node(node.id);
    const { width, height } = getNodeSize(node.type);
    return {
      id: node.id,
      type: 'custom',
      position: {
        x: nodeWithPosition.x - width / 2,
        y: nodeWithPosition.y - height / 2,
      },
      data: { ...node },
    };
  });

  return flowNodes;
}
