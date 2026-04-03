export const TEAM_LABELS: Record<string, string> = {
  blue: '蓝方',
  red: '红方',
  unknown: '未知',
};

export const MOTION_TYPE_LABELS: Record<string, string> = {
  plane_move: '平面移动',
  ramp_up: '坡道上行',
  ramp_down: '坡道下行',
};

export const NODE_TYPE_LABELS: Record<string, string> = {
  mf_edge_pose: '主区导航点',
  staging: '等待点',
  ramp_entry: '坡道入口点',
  ramp_exit: '坡道出口点',
};

export const GOAL_KIND_LABELS: Record<string, string> = {
  node: '导航点',
  task: '任务',
  route: '预设路线',
};

export const FRAME_TYPE_LABELS: Record<string, string> = {
  init: '初始化前沿',
  skip_stale: '跳过过期队列项',
  pop: '取出当前最优点',
  blocked_node: '节点阻塞',
  edge_blocked: '边阻塞',
  relax: '更新更优路径',
  keep_best: '保留已有更优路径',
  failed: '搜索失败',
  goal: '到达目标',
  route_tag: '展开预设路线',
};

export const WORLD_RENDER_CLASS_LABELS: Record<string, string> = {
  'world-ground': '地面分区',
  'world-marking': '标线/起始区',
  'world-platform': '平台/台面',
  'world-fence': '围栏轮廓',
};

export const TASK_REASON_LABELS: Record<string, string> = {
  candidate_blocked: '候选点已被阻塞',
};

export const VIEW_MODE_LABELS: Record<string, string> = {
  orbit: '轨道环绕 (Orbit)',
  follow: '目标跟随 (Follow)',
  first_person: '第一人称 (First Person)',
  top_ortho: '顶部正交 (Top Ortho)',
  side_ortho: '侧视正交 (Side Ortho)',
  side_perspective: '侧视透视 (Side Perspective)',
};

export const LAYER_LABELS: Record<string, string> = {
  scene: '场景几何 (Scene)',
  graph: '拓扑网络 (Graph)',
  keyNodes: '关键节点 (Key Nodes)',
  openSet: '开放集合 (Open Set)',
  expanded: '已扩展点 (Expanded)',
  tree: '搜索树/候选 (Tree)',
  candidates: '候选轨迹 (Candidates)',
  shadows: '实时阴影 (Shadows)',
  blocked: '阻塞区域 (Blocked)',
};

export const ALGORITHM_LABELS: Record<string, string> = {
  astar: 'A* / 运行时回放',
  rrt: 'RRT',
  dwa: 'Holonomic DWA',
};

export const RUN_MODE_LABELS: Record<string, string> = {
  'offline-sim': '离线仿真 (Offline Sim)',
  'live-ros': '实时只读 (Live ROS)',
};

export const UI_LABELS = {
  appTitle: '3D 战术观测沙盘',
  appSubtitle: '完整 Mesh 场地、真实路径颜色层、多视角相机与 A* / RRT / DWA 仿真共用 WebGPU 观测面。',
  statusWaiting: '正在加载场景',
  statusLoaded: '场景已加载，等待手动设置起点/目标并生成离线运行',
  statusError: '场景加载失败',
  panelConfig: '运行配置',
  panelView: '观测视角',
  panelLayers: '图层控制',
  panelPick: '场景选点',
  panelState: '运行状态',
  panelFrame: '当前帧',
  panelSummary: '运行摘要',
  panelLive: '实时桥接',
  btnGenerateRun: '生成手动离线运行',
  btnStartLive: '启动实时桥接',
  btnPlay: '播放回放',
  btnPause: '暂停',
  btnStep: '单步',
  btnReset: '重置',
  btnPickStart: '在场景中设起点',
  btnPickGoal: '在场景中设目标',
  btnCancelPick: '取消选点',
  fieldTeam: '阵营',
  fieldMode: '模式',
  fieldAlgo: '算法',
  fieldStart: '起点节点',
  fieldGoalKind: '目标类型',
  fieldGoalValue: '目标值',
  fieldHoverNode: '当前吸附预览',
  fieldBlocked: '阻塞节点',
  fieldStrict: 'A* 复用运行时真逻辑',
  fieldSpeed: '播放倍率',
  statRunId: '运行 ID',
  statCursor: '当前进度',
  statState: '状态',
  statFaces: '场景面数',
  liveActiveEdge: '激活边',
  liveGate: '语义门',
  liveCorridor: '走廊 ID',
  liveDistance: '距目标距离',
  legendStart: '起点',
  legendGoal: '终点',
  legendPath: '路径',
  legendOpen: '开放集',
  legendTree: '搜索树/候选',
  hintManualRunOnly: '页面默认不会自动播放；只有手动生成离线运行后，播放/单步/重置才会生效。',
  hintLiveReadonly: '实时模式只读观察 ROS 状态，不接受浏览器选点。',
  hintPickIdle: '可在表单里直接选节点，也可打开场景选点模式；点击场景任意位置会吸附到最近导航点。',
  hintPickStart: '起点选点已开启：点击场景任意位置，会吸附到最近导航点并写入起点。',
  hintPickGoal: '目标选点已开启：点击场景任意位置，会吸附到最近导航点并写入目标节点。',
  hintPickPreviewEmpty: '移动鼠标到场地或节点上方可预览吸附结果。',
  hintRunIdle: '尚未生成离线运行，右侧不会自动出现播放内容。',
  hintFrameIdle: '等待手动生成离线运行',
  hintFrameRunReady: '离线运行已创建，可播放或单步查看。',
  hintSummaryEmpty: '尚无运行摘要',
};

export function withRawLabel(displayName: string, rawValue: string): string {
  if (!displayName || displayName === rawValue) return rawValue;
  return `${displayName} (${rawValue})`;
}
