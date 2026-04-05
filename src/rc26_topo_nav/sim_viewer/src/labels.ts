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

export const FRAME_METRIC_LABELS: Record<string, string> = {
  gCost: '累计代价',
  fCost: '估计总代价',
  stepCost: '单步代价',
  traceMode: '回放模式',
};

const BLOCK_NODE_RE = /^mf_b(\d+)$/;
const SURFACE_NODE_RE = /^sf_(\d+)_(\d+)$/;

const SPECIAL_NODE_LABELS: Record<string, string> = {
  mf_entry_staging: '主区入口等待点',
  mf_exit_staging: '主区出口等待点',
  ramp_entry_south: '南侧坡道入口点',
  ramp_exit_north: '北侧坡道出口点',
  ramp_entry_north: '北侧坡道入口点',
  ramp_exit_south: '南侧坡道出口点',
  sf_start: '表面起点采样点',
  sf_goal: '表面终点采样点',
};

const FAILURE_CODE_LABELS: Record<string, string> = {
  POINT_NOT_TRAVERSABLE: '请求点不在可通行表面上',
  NO_SURFACE_PATH: '没有可达的表面路线',
  TRACE_FAILED: '搜索回放生成失败',
  NO_PATH: '没有可达路径',
  INVALID_TARGET_TYPE: '目标类型无效',
};

const FAILURE_REASON_LABELS: Record<string, string> = {
  'Start point is not on a traversable surface': '起点不在可通行表面上',
  'Goal point is not on a traversable surface': '终点不在可通行表面上',
  'Planner could not find a traversable surface path': '未找到可通行的表面路线',
  'No traversable surface node near the requested point': '请求点附近没有可通行表面节点',
  'Unsupported topo target_type': '不支持的拓扑目标类型',
  'Planner returned empty route': '规划器返回了空路线',
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
  appTitle: '三维路线观察台',
  appSubtitle: '只保留任意点到任意点的三维路线生成、搜索过程回放与可选执行。',
  routeTitle: '表面三维路线',
  routeModeHint: '当前固定为表面路径搜索模式。浏览器只负责观察任意点到任意点的三维路线与搜索演绎，执行为可选动作。',
  statusWaiting: '正在加载比赛场地',
  statusLoaded: '场景已就绪，先在场景里点起点和终点',
  statusError: '场景加载失败',
  panelView: '观测视角',
  panelLayers: '图层控制',
  panelLegend: '颜色说明',
  panelPick: '场景选点',
  panelRoute: '当前路线',
  panelTrace: '搜索回放',
  panelSegments: '路线分段',
  panelPlanningLogs: '规划日志',
  panelTiming: '规划时间',
  btnPickStart: '设起点',
  btnPickGoal: '设终点',
  btnCancelPick: '取消选点',
  btnGenerateRoute: '生成三维路线',
  btnExecuteRoute: '执行当前路线',
  btnClearRoute: '清空路线',
  fieldTeam: '阵营',
  fieldHoverPoint: '当前鼠标预览',
  fieldTraceIndex: '回放帧',
  fieldStartNode: '起点投影节点',
  fieldGoalNode: '终点投影节点',
  statFrame: '当前帧',
  statCost: '总代价',
  statCompletePlanning: '完整规划时间',
  statSurfaceProjection: '投影耗时',
  statSurfacePlanning: '路径搜索耗时',
  statPathExpand: '路径展开耗时',
  statSegmentBuild: '分段生成耗时',
  statPreviewChain: '网页预览链路',
  statTracePlanning: '回放搜索耗时',
  statTraceChain: '回放链路耗时',
  statTotalElapsed: '总链路耗时',
  statPreviewElapsed: '网页预览链路',
  statTraceElapsed: '回放链路耗时',
  statPathPoints: '路径点数',
  statSegments: '分段数',
  statFrontier: '前沿点',
  statExpanded: '已探查点',
  legendOpenSet: '蓝色圆点表示前沿点',
  legendOpenSetHint: '表示这些节点已经进入待扩展队列，但还没有被正式展开。',
  legendExpanded: '黄色方块表示已探查点',
  legendExpandedHint: '表示这些节点已经被规划器取出并完成过扩展。',
  legendLayerHint: '对应顶部“前沿”和“已探查”图层，生成搜索回放后可见。',
  hintPickIdle: '点“设起点”或“设终点”后，在地面、坡面或阶梯表面直接点击。',
  hintPickSurfaceStart: '起点选点已开启，点击比赛场地中的任意可见表面。',
  hintPickSurfaceGoal: '终点选点已开启，点击比赛场地中的任意可见表面。',
  hintGenerate: '先生成表面路线，搜索回放会在后台补齐。',
  hintExecute: '执行只会把当前起终点路线下发给运行时；机器人需已在起点附近。',
  hintTraceEmpty: '尚未生成路线。',
  hintPlanningLogs: '按调用顺序展示本次表面路线生成链路。',
  hintPlanningLogsEmpty: '尚未生成规划日志。',
  emptyValue: '暂无',
  statusPending: '等待生成',
  statusGenerating: '正在生成',
  statusDispatching: '正在下发',
  statusBackgroundReplay: '正在后台生成搜索回放',
  statusReplayPending: '正在补齐回放',
  statusReplayEmpty: '尚无回放',
  statusReplayReady: '路线已生成，搜索回放已补齐。',
  hintTimingFormula: '完整规划时间 = 投影耗时 + 路径搜索耗时 + 路径展开耗时 + 分段生成耗时。',
  hintTimingDiagnostic: '网页预览链路与回放链路只用于页面诊断，不作为主规划指标。',
  metricAdditional: '附加指标',
};

export function formatFramePhase(phase: string): string {
  return FRAME_TYPE_LABELS[phase] ?? '未标注阶段';
}

export function formatFrameLabel(label: string | undefined, phase: string): string {
  if (!label) {
    return formatFramePhase(phase);
  }
  return FRAME_MESSAGE_LABELS[label] ?? formatFramePhase(phase);
}

export function formatFrameMetricLabel(key: string): string {
  return FRAME_METRIC_LABELS[key] ?? UI_LABELS.metricAdditional;
}

function digitsOnlyLabel(rawValue: string, prefix: string): string | null {
  const digits = rawValue.match(/\d+/g);
  if (!digits || digits.length === 0) {
    return null;
  }
  return `${prefix} ${digits.join('-')}`;
}

export function formatNodeLabel(nodeId: string | null | undefined): string {
  if (!nodeId) {
    return UI_LABELS.emptyValue;
  }
  const special = SPECIAL_NODE_LABELS[nodeId];
  if (special) {
    return special;
  }
  const blockMatch = BLOCK_NODE_RE.exec(nodeId);
  if (blockMatch) {
    return `主区 ${blockMatch[1]} 号块导航点`;
  }
  const surfaceMatch = SURFACE_NODE_RE.exec(nodeId);
  if (surfaceMatch) {
    return `表面采样点 ${surfaceMatch[1]}-${surfaceMatch[2]}`;
  }
  return digitsOnlyLabel(nodeId, '导航点') ?? '未命名导航点';
}

export function formatNodeTransitionLabel(fromNodeId: string, toNodeId: string): string {
  return `${formatNodeLabel(fromNodeId)} 到 ${formatNodeLabel(toNodeId)}`;
}

export function formatFailureCode(code: string | null | undefined): string {
  if (!code) {
    return UI_LABELS.emptyValue;
  }
  return FAILURE_CODE_LABELS[code] ?? digitsOnlyLabel(code, '错误代码') ?? '未知错误';
}

export function formatFailureReason(reason: string | null | undefined): string {
  if (!reason) {
    return UI_LABELS.emptyValue;
  }
  const localized = FAILURE_REASON_LABELS[reason];
  if (localized) {
    return localized;
  }
  if (/[\u4e00-\u9fff]/.test(reason)) {
    return reason;
  }
  return '未知失败原因';
}

export function formatFailureSummary(reason: string | null | undefined, code?: string | null): string {
  const localizedReason = formatFailureReason(reason);
  if (localizedReason !== UI_LABELS.emptyValue && localizedReason !== '未知失败原因') {
    return localizedReason;
  }
  const localizedCode = formatFailureCode(code);
  if (localizedCode !== UI_LABELS.emptyValue) {
    return localizedCode;
  }
  return localizedReason;
}

function extractErrorText(error: unknown): string {
  if (error instanceof Error) {
    return error.message.trim();
  }
  if (typeof error === 'string') {
    return error.trim();
  }
  if (error && typeof error === 'object' && 'message' in error && typeof error.message === 'string') {
    return error.message.trim();
  }
  return '';
}

export function formatUnexpectedError(error: unknown, fallback: string): string {
  const detail = extractErrorText(error);
  if (!detail) {
    return fallback;
  }
  if (/[\u4e00-\u9fff]/.test(detail)) {
    return detail;
  }
  const statusMatch = detail.match(/\b([1-9]\d{2})\b/);
  if (statusMatch) {
    return `${fallback}，状态码 ${statusMatch[1]}`;
  }
  return fallback;
}

export function formatPlanningLogFieldValue(label: string, value: string): string {
  if (label === '起点投影节点' || label === '终点投影节点') {
    return formatNodeLabel(value);
  }
  if (label === '失败码') {
    return formatFailureCode(value);
  }
  if (label === '失败原因') {
    return formatFailureReason(value);
  }
  return value;
}
