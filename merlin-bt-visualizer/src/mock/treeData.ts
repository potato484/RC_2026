import { BTNode } from '../types';

export const mockNodes: BTNode[] = [
  { id: 'root', type: 'sequence', label: '梅林区巡航', state: 'idle', desc: '执行梅林区的主逻辑' },
  { id: 'check_target', type: 'condition', label: '看到目标没？', state: 'idle', desc: '检查摄像头是否发现目标环', parentId: 'root' },
  { id: 'action_sel', type: 'selector', label: '决定怎么做', state: 'idle', desc: '根据目标状态选择动作', parentId: 'root' },
  { id: 'scan', type: 'action', label: '左右张望', state: 'idle', desc: '转动身体寻找目标', parentId: 'action_sel' },
  { id: 'move', type: 'action', label: '走向目标', state: 'idle', desc: '朝看到的目标前进', parentId: 'action_sel' },
  { id: 'grab', type: 'action', label: '伸手抓取', state: 'idle', desc: '尝试抓取前方的环', parentId: 'action_sel' },
];

export const mockEdges = [
  { id: 'e-root-check', source: 'root', target: 'check_target' },
  { id: 'e-root-sel', source: 'root', target: 'action_sel' },
  { id: 'e-sel-scan', source: 'action_sel', target: 'scan' },
  { id: 'e-sel-move', source: 'action_sel', target: 'move' },
  { id: 'e-sel-grab', source: 'action_sel', target: 'grab' },
];

export const nodePositions = {
  'root': { x: 400, y: 50 },
  'check_target': { x: 200, y: 200 },
  'action_sel': { x: 600, y: 200 },
  'scan': { x: 400, y: 350 },
  'move': { x: 600, y: 350 },
  'grab': { x: 800, y: 350 },
};
