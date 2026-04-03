export interface BehaviorTreeNodeDisplay {
  label: string;
  desc: string;
}

export interface BehaviorTreeAttributeDisplay {
  rawKey: string;
  rawValue: string;
  label: string;
  value: string;
  summary: string;
}

const actionTranslations: Record<string, { label: string; desc: string }> = {
  NavToSmartPoint: { label: '导航到点', desc: '控制底盘移动到指定的预设智能点' },
  NavToMerlinGrid: { label: '导航到梅林格', desc: '根据九宫格编号移动底盘到指定格' },
  SetNavMode: { label: '设置导航模式', desc: '切换导航的安全/穿越/正常模式' },
  ScanSurroundings: { label: '扫描周围环境', desc: '启动感知节点，寻找目标环或障碍' },
  CheckR1Blocking: { label: '检测R1阻挡', desc: '检查路径上是否有我方R1机器人挡路' },
  SelectNextGrid: { label: '选择下一格', desc: '根据地图信息决策下一步要去哪个梅林格' },
  GrabKFS: { label: '抓取兑换块', desc: '伸出机械臂抓取面前的KFS（块）' },
  IncrementKFSCount: { label: '增加计数', desc: '更新已抓取KFS的数量状态' },
  UpdateMapKFS: { label: '更新地图', desc: '标记该格子上的KFS已被取走' },
  CheckExitCondition: { label: '检查退出条件', desc: '判断是否已经抓满指定数量的KFS' },
  StairDescend: { label: '下台阶', desc: '执行下台阶的控制序列' },
  Delay: { label: '延迟等待', desc: '暂停执行一段时间' },
  AlwaysSuccess: { label: '始终返回成功', desc: '无论如何都返回成功，通常用于忽略非致命错误' },
  ScriptCondition: { label: '脚本条件判断', desc: '执行黑板脚本以判断条件真假' },
  Script: { label: '执行脚本', desc: '更新黑板变量' },
  FollowManualRobot: { label: '跟随手动机器人', desc: '使用传感器跟随前方的手动机器人' },
  MechUpDuel: { label: '机械臂升起', desc: '将机械臂升起到对抗高度' },
  PlaceKFSGrid: { label: '放置兑换块', desc: '在指定格子放下KFS' },
  WaitUntilTrigger: { label: '等待触发', desc: '等待特定条件或外部信号触发' },
  GrabTip: { label: '抓取矛头', desc: '抓取矛头准备组装' },
  CheckManualRobot: { label: '检测手动机器人', desc: '检查手动机器人是否在可组装范围内' },
  AssembleWeapon: { label: '组装武器', desc: '将部件进行组装动作' },
};

const paramTranslations: Record<string, string> = {
  MF_SAFE: '梅林安全模式',
  MF_TRAVERSE: '梅林穿越模式',
  MF_EXIT: '梅林退出模式',
  NORMAL: '正常模式',
  mf_entry: '梅林入口点',
  mf_entry_back: '梅林入口退避点',
  mf_grid_2: '梅林2号格',
  mf_exit: '梅林出口点',
  '{target_grid}': '目标格子变量',
  '{next_action}': '下一动作变量',
};

const displayableAttrs = [
  { keys: ['delay_msec', 'delay'], label: '时长', unit: '毫秒' },
  { keys: ['num_attempts', 'attempts'], label: '重试', unit: '次' },
  { keys: ['count', 'num_cycles', 'num_attempts'], label: '次数', unit: '次' },
  { keys: ['mode'], label: '模式', unit: '' },
  { keys: ['target_name', 'target'], label: '目标', unit: '' },
  { keys: ['grid_id', 'grid_position', 'grid'], label: '目标格', unit: '' },
  { keys: ['follow_distance', 'distance'], label: '距离', unit: '米' },
  { keys: ['distance_threshold', 'threshold'], label: '阈值', unit: '米' },
  { keys: ['static_time'], label: '静止', unit: '秒' },
  { keys: ['code'], label: '代码', unit: '' },
] as const;

const attributeLabelTranslations: Record<string, string> = {
  name: '节点名称',
  id: '节点标识',
  _autoremap: '自动映射',
  mode: '模式',
  target_name: '目标',
  target: '目标',
  grid_id: '目标格',
  grid_position: '目标格',
  grid: '目标格',
  follow_distance: '跟随距离',
  distance: '距离',
  distance_threshold: '阈值',
  threshold: '阈值',
  static_time: '静止时间',
  code: '脚本',
  delay_msec: '时长',
  delay: '时长',
  num_attempts: '重试次数',
  attempts: '重试次数',
  count: '次数',
  num_cycles: '次数',
};

const nodeCategoryTranslations: Record<string, string> = {
  control: '控制节点',
  decorator: '装饰节点',
  leaf: '执行节点',
  subtree: '子树节点',
};

function containsEnglishLetters(text: string): boolean {
  return /[a-zA-Z]/.test(text);
}

function sanitizeDisplayValue(text: string, fallback: string): string {
  if (!text) return fallback;
  return containsEnglishLetters(text) ? fallback : text;
}

function withOptionalUnit(value: string, unit = ''): string {
  return unit ? `${value}${unit}` : value;
}

export function translateParam(param: string): string {
  if (!param) return param;
  if (paramTranslations[param]) return paramTranslations[param];
  if (param.includes("next_action=='GRAB'")) return '判断是否去抓取';
  if (param.includes("next_action=='MOVE'")) return '判断是否去移动';
  if (param.includes('target_kfs_count:=2')) return '初始化变量(2个块)';
  if (param.includes('current_grid:=')) return `设当前格为${param.split(':=')[1]}`;
  return param;
}

export function translateName(name: string, fallbackLabel: string): string {
  if (!name) return fallbackLabel;

  let translated = name;
  const exactMatches: Record<string, string> = {
    Combat_Sequence: '对抗区主流程',
    goto_combat: '前往对抗区',
    place_sequence: '放置流程',
    grab_tip: '抓取矛头',
    assemble: '组装',
    Entry_Seq: '进门流程',
    Loop_Body: '循环体',
    GrabKFSSeq: '抓取兑换块流程',
    MoveToGridSeq: '移动至目标格流程',
    Exit_Seq: '出门流程',
    MC_Sequence: '武馆区主流程',
    MFAreaTree: '梅林区树',
    MF_Entry: '梅林进门',
    MF_Loop: '梅林循环',
    MF_Exit: '梅林出门',
    MF_Main: '梅林主流程',
    CombatAreaTree: '对抗区树',
    MCAreaTree: '武馆区树',
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

  if (/[a-zA-Z]/.test(translated)) {
    if (fallbackLabel && !containsEnglishLetters(fallbackLabel)) {
      return fallbackLabel;
    }
    return '已命名节点';
  }

  return translated;
}

export function getBehaviorTreeTreeName(treeId: string, treeName?: string): string {
  const rawName = treeName || treeId;
  return translateName(rawName, '未命名决策树');
}

function normalizeAttributes(attributes: Record<string, string>): Record<string, string> {
  return Object.entries(attributes).reduce<Record<string, string>>((acc, [key, value]) => {
    acc[key.toLowerCase()] = value;
    return acc;
  }, {});
}

function getFallbackLabel(tagName: string, normalizedAttributes: Record<string, string>): string {
  const idAttr = normalizedAttributes.id || '';
  const code = normalizedAttributes.code;
  let fallbackLabel = actionTranslations[tagName]?.label || '';

  if (tagName === 'Script' && code) {
    fallbackLabel = translateParam(code);
  } else if (tagName === 'ScriptCondition' && code) {
    fallbackLabel = `检查: ${translateParam(code)}`;
  }

  if (fallbackLabel) return fallbackLabel;

  if (tagName === 'Sequence') return '顺序流程';
  if (tagName === 'ReactiveSequence') return '自适应顺序';
  if (tagName === 'Fallback') return '备选流程';
  if (tagName === 'ReactiveFallback') return '自适应备选';
  if (tagName === 'SubTree') return translateName(idAttr, idAttr);
  if (tagName === 'RetryUntilSuccessful') return '一直重试';
  if (tagName === 'KeepRunningUntilFailure') return '死循环 (直到出错)';
  if (tagName === 'Inverter') return '条件取反 (不满足时成功)';
  if (tagName === 'ForceFailure') return '必定失败';
  return '未知操作';
}

function getDefaultDescription(tagName: string, label: string): string {
  if (actionTranslations[tagName]) return actionTranslations[tagName].desc;
  if (tagName === 'Sequence') return '顺序节点 (从左到右依次执行，必须全部成功)';
  if (tagName === 'ReactiveSequence') return '自适应顺序节点 (每步都会重新检查前面已完成的步骤)';
  if (tagName === 'Fallback') return '备选方案节点 (只要有一个成功就停止)';
  if (tagName === 'ReactiveFallback') return '自适应备选节点 (持续监测高优先级条件)';
  if (tagName === 'SubTree') return label;
  if (tagName === 'RetryUntilSuccessful') return '一直重试直到子节点返回成功';
  if (tagName === 'KeepRunningUntilFailure') return '循环执行直到子节点返回失败';
  if (tagName === 'Inverter') return '将子节点的结果取反';
  if (tagName === 'Delay') return '在执行子节点前进行延迟';
  if (tagName === 'Repeat') return '重复执行子节点指定次数';
  return '未映射控制节点';
}

function buildDetailTexts(normalizedAttributes: Record<string, string>): string[] {
  const usedKeys = new Set<string>();

  return displayableAttrs.flatMap((config) => {
    for (const key of config.keys) {
      const rawValue = normalizedAttributes[key];
      if (rawValue === undefined || rawValue === null) continue;
      if (usedKeys.has(key)) continue;

      usedKeys.add(key);
      return [getBehaviorTreeAttributeDisplay(key, rawValue).summary];
    }

    return [];
  });
}

export function getBehaviorTreeNodeCategoryLabel(uiType: string): string {
  return nodeCategoryTranslations[uiType] || '节点';
}

export function getBehaviorTreeAttributeDisplay(
  key: string,
  rawValue: string
): BehaviorTreeAttributeDisplay {
  const normalizedKey = key.toLowerCase();
  const label = attributeLabelTranslations[normalizedKey] || '自定义属性';

  let value = rawValue;

  if (normalizedKey === 'name') {
    value = sanitizeDisplayValue(translateName(rawValue, ''), '已命名节点');
  } else if (normalizedKey === 'id') {
    value = sanitizeDisplayValue(translateName(rawValue, ''), '已配置标识');
  } else if (normalizedKey === '_autoremap') {
    value = ['true', '1', 'yes'].includes(rawValue.toLowerCase()) ? '是' : '否';
  } else if (['mode', 'target_name', 'target', 'grid_id', 'grid_position', 'grid'].includes(normalizedKey)) {
    value = sanitizeDisplayValue(translateParam(rawValue), '已配置目标');
  } else if (['follow_distance', 'distance'].includes(normalizedKey)) {
    value = withOptionalUnit(sanitizeDisplayValue(rawValue, '已配置距离'), '米');
  } else if (['distance_threshold', 'threshold'].includes(normalizedKey)) {
    value = withOptionalUnit(sanitizeDisplayValue(rawValue, '已配置阈值'), '米');
  } else if (normalizedKey === 'static_time') {
    value = withOptionalUnit(sanitizeDisplayValue(rawValue, '已配置时间'), '秒');
  } else if (['delay_msec', 'delay'].includes(normalizedKey)) {
    value = withOptionalUnit(sanitizeDisplayValue(rawValue, '已配置时长'), '毫秒');
  } else if (['num_attempts', 'attempts', 'count', 'num_cycles'].includes(normalizedKey)) {
    value = withOptionalUnit(sanitizeDisplayValue(rawValue, '已配置次数'), '次');
  } else if (normalizedKey === 'code') {
    value = sanitizeDisplayValue(translateParam(rawValue), '已配置脚本');
  } else {
    value = sanitizeDisplayValue(translateParam(rawValue), '已配置属性值');
  }

  return {
    rawKey: key,
    rawValue,
    label,
    value,
    summary: `${label}: ${value}`,
  };
}

export function getBehaviorTreeAttributeDisplays(
  attributes: Record<string, string>
): BehaviorTreeAttributeDisplay[] {
  return Object.entries(attributes).map(([key, value]) =>
    getBehaviorTreeAttributeDisplay(key, value)
  );
}

export function summarizeBehaviorTreeAttributes(
  attributes: Record<string, string>,
  limit = 2
): string[] {
  return getBehaviorTreeAttributeDisplays(attributes)
    .map((attribute) => attribute.summary)
    .slice(0, limit);
}

export function getBehaviorTreeNodeDisplay(
  tagName: string,
  attributes: Record<string, string>
): BehaviorTreeNodeDisplay {
  const normalizedAttributes = normalizeAttributes(attributes);
  const rawLabel = normalizedAttributes.name || normalizedAttributes.id || tagName;
  const idAttr = normalizedAttributes.id || '';
  const fallbackLabel = getFallbackLabel(tagName, normalizedAttributes);

  let label = translateName(rawLabel, fallbackLabel);
  if ((label === 'Sequence' || label === 'Fallback' || (idAttr && label === idAttr)) && fallbackLabel) {
    label = fallbackLabel;
  }

  let desc = getDefaultDescription(tagName, label);
  const details = buildDetailTexts(normalizedAttributes);
  if (details.length > 0) {
    label += ` (${details.join(', ')})`;
    desc += ` [${details.join(', ')}]`;
  }

  return { label, desc };
}
