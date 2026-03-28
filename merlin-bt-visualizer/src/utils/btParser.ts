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
  'AlwaysSuccess': { label: '始终成功', desc: '强制返回成功状态' },
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

  translated = translated.replace(/Sequence/g, '流程');
  translated = translated.replace(/Seq/g, '流程');
  translated = translated.replace(/Main/g, '主流程');
  translated = translated.replace(/Entry/g, '进门');
  translated = translated.replace(/Loop/g, '循环');
  translated = translated.replace(/Exit/g, '出门');
  translated = translated.replace(/Grab/g, '抓取');
  translated = translated.replace(/Move/g, '移动');
  translated = translated.replace(/MF_/g, '梅林区_');
  translated = translated.replace(/Combat_/g, '对抗区_');
  translated = translated.replace(/MC_/g, '武馆区_');
  
  // If it still contains english letters, return the fallback label (the pure action name)
  if (/[a-zA-Z]/.test(translated)) {
    return fallbackLabel || translated;
  }
  
  return translated;
};

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

    function traverse(element: Element, parentId?: string, siblingIndex: number = 0) {
      if (element.nodeType !== Node.ELEMENT_NODE) return;
      
      const tagName = element.tagName;
      if (tagName === 'BehaviorTree') {
        const children = Array.from(element.children);
        if (children.length > 0) {
          traverse(children[0]);
        }
        return;
      }

      const id = `${treeId}_node_${idCounter++}`;
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
      
      if (!fallbackActionLabel) {
         if (type === 'sequence') fallbackActionLabel = '顺序执行';
         else if (type === 'selector') fallbackActionLabel = '选择执行';
         else if (type === 'subtree') fallbackActionLabel = '执行子树';
         else if (tagName === 'RetryUntilSuccessful') fallbackActionLabel = '重试直到成功';
         else if (tagName === 'KeepRunningUntilFailure') fallbackActionLabel = '持续运行';
         else if (tagName === 'Inverter') fallbackActionLabel = '状态反转';
         else if (tagName === 'ForceFailure') fallbackActionLabel = '强制失败';
         else fallbackActionLabel = '未知操作';
      }

      let label = translateName(rawLabel, fallbackActionLabel);
      let desc = '';
      
      if (actionTranslations[tagName]) {
        desc = actionTranslations[tagName].desc;
        
        const mode = element.getAttribute('mode');
        const targetName = element.getAttribute('target_name');
        const delay = element.getAttribute('delay_msec');
        const code = element.getAttribute('code');
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
        if (code) desc += ` [操作: ${translateParam(code)}]`;
      } else {
        if (type === 'sequence') { desc = '顺序节点 (从左到右依次执行所有子节点，只要有一个失败，整体就判定为失败)'; }
        if (type === 'selector') { desc = '选择节点 (从左到右依次尝试执行子节点，只要有一个成功，整体就判定为成功)'; }
        if (type === 'subtree') { desc = '子树节点 (跳转并执行另一个行为树的完整流程)'; }
        if (tagName === 'KeepRunningUntilFailure') { desc = '循环节点 (不断重复执行内部的逻辑，直到某一次执行返回失败为止)'; }
        if (tagName === 'RetryUntilSuccessful') { 
          const attempts = element.getAttribute('num_attempts');
          desc = `重试节点 (不断尝试执行，直到成功为止。最多允许重试 ${attempts || '无限'} 次)`; 
        }
        if (tagName === 'Inverter') { desc = '反转节点 (把成功的结果变成失败，把失败的结果变成成功)'; }
        if (tagName === 'ForceFailure') { desc = '强制失败节点 (不管内部执行结果如何，最终一定返回失败)'; }
        
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
        subTreeId: tagName === 'SubTree' ? idAttr || undefined : undefined
      };

      nodes.push(node);

      if (parentId) {
        edges.push({
          id: `e-${parentId}-${id}`,
          source: parentId,
          target: id
        });
      }

      Array.from(element.children).forEach((child, index) => traverse(child, id, index + 1));
      
      // Do NOT expand subtree inline anymore!
    }

    traverse(treeEl);
    
    parsedTrees[treeId] = { id: treeId, name: treeName, nodes, edges };
  });

  return { mainTreeId: targetMainTreeId, trees: parsedTrees };
}

export function layoutNodes(nodes: BTNode[], edges: { id: string; source: string; target: string }[]) {
  const dagreGraph = new dagre.graphlib.Graph();
  dagreGraph.setDefaultEdgeLabel(() => ({}));
  
  // 优化间距：减小水平和垂直间距，让树更紧凑
  // align: 'UL' 或者默认均可，这里使用更紧凑的配置
  dagreGraph.setGraph({ 
    rankdir: 'TB', 
    nodesep: 30, // 缩小兄弟节点之间的水平间距
    ranksep: 60, // 缩小层级之间的垂直间距
    edgesep: 10,
    marginx: 20,
    marginy: 20
  });

  nodes.forEach((node) => {
    // CustomNode dimensions are w-[240px] h-[80px]
    dagreGraph.setNode(node.id, { width: 240, height: 80 });
  });

  edges.forEach((edge) => {
    dagreGraph.setEdge(edge.source, edge.target);
  });

  dagre.layout(dagreGraph);

  const flowNodes: FlowNode[] = nodes.map((node) => {
    const nodeWithPosition = dagreGraph.node(node.id);
    return {
      id: node.id,
      type: 'custom',
      position: {
        x: nodeWithPosition.x - 240 / 2,
        y: nodeWithPosition.y - 80 / 2,
      },
      data: { ...node },
    };
  });

  return flowNodes;
}
