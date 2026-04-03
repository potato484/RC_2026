export const TEAM_LABELS: Record<string, string> = {
  blue: '蓝方',
  red: '红方',
  unknown: '未知',
};

export const MOTION_TYPE_LABELS: Record<string, string> = {
  plane_move: '平面移动',
  ramp_up: '坡道上行',
  ramp_down: '坡道下行',
  stair_up: '阶梯上行',
  stair_down: '阶梯下行',
};

export const FRAME_TYPE_LABELS: Record<string, string> = {
  init: '初始化前沿',
  skip_stale: '跳过过期节点',
  pop: '取出当前最优点',
  blocked_node: '节点阻塞',
  edge_blocked: '边阻塞',
  relax: '更新更优路径',
  keep_best: '保留已有更优路径',
  failed: '搜索失败',
  goal: '到达目标',
};

export const FRAME_MESSAGE_LABELS: Record<string, string> = {
  'planner initialized': '初始化前沿',
  'stale queue entry skipped': '跳过过期队列项',
  'expanded current best node': '展开当前最优节点',
  'goal reached': '到达目标',
  'node blocked by overlay': '节点被覆盖层阻塞',
  'edge blocked by overlay': '边被覆盖层阻塞',
  'better path discovered': '发现更优路径',
  'existing path kept': '保留已有更优路径',
  'candidate blocked by overlay': '候选点被覆盖层阻塞',
  'selected best reachable candidate': '已选择最优可达候选点',
  'route tag advanced by declared edge': '按声明边推进路线标签',
};

export const VIEW_MODE_LABELS: Record<string, string> = {
  orbit: '轨道环绕',
  follow: '目标跟随',
  first_person: '第一人称',
  top_ortho: '顶部正交',
  side_ortho: '侧视正交',
  side_perspective: '侧视透视',
};

export const VIEW_MODE_SHORT_LABELS: Record<string, string> = {
  orbit: '环',
  follow: '跟',
  first_person: '首',
  top_ortho: '俯',
  side_ortho: '侧',
  side_perspective: '侧透',
};

export const TEAM_SHORT_LABELS: Record<string, string> = {
  blue: '蓝',
  red: '红',
};

export const LAYER_LABELS: Record<string, string> = {
  scene: '场景',
  graph: '拓扑',
  keyNodes: '路径节点',
  openSet: '前沿',
  expanded: '已探查',
  tree: '搜索树',
  candidates: '候选轨迹',
  shadows: '阴影',
  blocked: '阻塞区',
};

export const LAYER_SHORT_LABELS: Record<string, string> = {
  scene: '景',
  graph: '图',
  keyNodes: '径',
  openSet: '前',
  expanded: '展',
  tree: '树',
  candidates: '轨',
  shadows: '影',
  blocked: '阻',
};

export const UI_LABELS = {
  appEyebrow: 'RC26 表面路线',
  appTitle: '3D 路线观察台',
  appSubtitle: '只保留任意点到任意点的三维路线生成、搜索过程回放与可选执行。',
  routeTitle: '表面 A* 三维路线',
  routeModeHint: '固定模式为表面 A*。浏览器只负责观察任意点 3D 路线与搜索演绎，执行是可选动作。',
  statusWaiting: '正在加载比赛场地',
  statusLoaded: '场景已就绪，先在场景里点起点和终点',
  statusError: '场景加载失败',
  panelView: '观测视角',
  panelLayers: '图层控制',
  panelPick: '场景选点',
  panelRoute: '当前路线',
  panelTrace: '搜索回放',
  panelSegments: '路线分段',
  btnPickStart: '设起点',
  btnPickGoal: '设终点',
  btnCancelPick: '取消选点',
  btnGenerateRoute: '生成 3D 路线',
  btnExecuteRoute: '执行当前路线',
  btnClearRoute: '清空路线',
  fieldTeam: '阵营',
  fieldHoverPoint: '当前鼠标预览',
  fieldTraceIndex: '回放帧',
  fieldStartNode: '起点投影节点',
  fieldGoalNode: '终点投影节点',
  statFrame: '当前帧',
  statCost: '总代价',
  statPathPoints: '路径点数',
  statSegments: '分段数',
  statFrontier: '前沿点',
  statExpanded: '已探查点',
  hintPickIdle: '点“设起点”或“设终点”后，在地面、坡面或阶梯表面直接点击。',
  hintPickSurfaceStart: '起点选点已开启，点击比赛场地中的任意可见表面。',
  hintPickSurfaceGoal: '终点选点已开启，点击比赛场地中的任意可见表面。',
  hintGenerate: '生成后使用滑块观察表面 A* 的逐帧展开过程。',
  hintExecute: '执行只会把当前起终点路线下发给运行时；机器人需已在起点附近。',
  hintTraceEmpty: '尚未生成路线。',
};

export function formatFramePhase(phase: string): string {
  return FRAME_TYPE_LABELS[phase] ?? phase;
}

export function formatFrameLabel(label: string | undefined, phase: string): string {
  if (!label) {
    return formatFramePhase(phase);
  }
  return FRAME_MESSAGE_LABELS[label] ?? label;
}
