import { BtNodeRegistryEntry, BtPortSchema } from '../generated/btNodeRegistry';

export interface BtNodeGuideZh {
  overviewZh: string;
  whenToUseZh: string;
  placementZh: string;
  runningZh: string;
  successZh: string;
  failureZh: string;
  pitfallsZh: string;
  exampleZh: string;
}

export interface BtKnowledgeBasePortSchema extends BtPortSchema {
  beginnerHintZh: string;
  exampleValueZh?: string;
}

export interface BtPortDocOverride {
  beginnerHintZh?: string;
  exampleValueZh?: string;
}

interface GuideOverrides {
  overviewZh: string;
  whenToUseZh: string;
  placementZh?: string;
  runningZh?: string;
  successZh?: string;
  failureZh?: string;
  pitfallsZh: string;
  exampleZh: string;
}

const numberTypes = new Set(['int', 'double', 'float', 'uint16']);

const guide = (value: BtNodeGuideZh): BtNodeGuideZh => value;

function buildControlGuide(labelZh: string, overrides: GuideOverrides): BtNodeGuideZh {
  return guide({
    overviewZh: overrides.overviewZh,
    whenToUseZh: overrides.whenToUseZh,
    placementZh:
      overrides.placementZh ??
      `${labelZh} 是一个“管流程顺序”的控制节点。把它放在一段子任务的外层，下面再挂 1 个或多个真正执行的子节点。`,
    runningZh:
      overrides.runningZh ??
      `${labelZh} 自己不会直接控制机构，它只负责安排下面的子节点按规则运行。只要当前还有孩子没结束，它通常就会保持“运行中”。`,
    successZh:
      overrides.successZh ??
      `${labelZh} 在它的调度规则被满足时返回成功。`,
    failureZh:
      overrides.failureZh ??
      `${labelZh} 在它的调度规则无法满足时返回失败。`,
    pitfallsZh: overrides.pitfallsZh,
    exampleZh: overrides.exampleZh,
  });
}

function buildDecoratorGuide(labelZh: string, overrides: GuideOverrides): BtNodeGuideZh {
  return guide({
    overviewZh: overrides.overviewZh,
    whenToUseZh: overrides.whenToUseZh,
    placementZh:
      overrides.placementZh ??
      `${labelZh} 是装饰节点，只能包住 1 个子节点。它不替你做实际动作，而是用来修改这个子节点的结果、节奏或重试方式。`,
    runningZh:
      overrides.runningZh ??
      `${labelZh} 会先把被它包住的那个子节点跑起来，再决定要不要改结果或继续等待。`,
    successZh:
      overrides.successZh ??
      `${labelZh} 在自己的规则满足时返回成功。`,
    failureZh:
      overrides.failureZh ??
      `${labelZh} 在自己的规则不满足时返回失败。`,
    pitfallsZh: overrides.pitfallsZh,
    exampleZh: overrides.exampleZh,
  });
}

function buildActionGuide(labelZh: string, overrides: GuideOverrides): BtNodeGuideZh {
  return guide({
    overviewZh: overrides.overviewZh,
    whenToUseZh: overrides.whenToUseZh,
    placementZh:
      overrides.placementZh ??
      `${labelZh} 是叶子动作节点。把它放在控制节点下面，作为“真正去做一件事”的一步，它自己不能再往下挂子节点。`,
    runningZh:
      overrides.runningZh ??
      `${labelZh} 开始后，通常会先向底层模块下发动作，再等待反馈，所以在动作没做完前会保持“运行中”。`,
    successZh:
      overrides.successZh ??
      `${labelZh} 真的执行完成，并且底层返回正常结果时才会成功。`,
    failureZh:
      overrides.failureZh ??
      `${labelZh} 执行失败、条件不满足、底层拒绝或超时时会返回失败。`,
    pitfallsZh: overrides.pitfallsZh,
    exampleZh: overrides.exampleZh,
  });
}

function buildConditionGuide(labelZh: string, overrides: GuideOverrides): BtNodeGuideZh {
  return guide({
    overviewZh: overrides.overviewZh,
    whenToUseZh: overrides.whenToUseZh,
    placementZh:
      overrides.placementZh ??
      `${labelZh} 是叶子条件节点。把它放在控制节点下面，用来回答“现在能不能继续下一步”，它自己也不能再往下挂子节点。`,
    runningZh:
      overrides.runningZh ??
      `${labelZh} 一般会很快给出结果；如果底层需要短时间观察或等待稳定状态，它也可能短暂保持“运行中”。`,
    successZh:
      overrides.successZh ??
      `${labelZh} 检查到条件成立时返回成功。`,
    failureZh:
      overrides.failureZh ??
      `${labelZh} 检查到条件不成立时返回失败。`,
    pitfallsZh: overrides.pitfallsZh,
    exampleZh: overrides.exampleZh,
  });
}

function buildSubtreeGuide(overrides: GuideOverrides): BtNodeGuideZh {
  return guide({
    overviewZh: overrides.overviewZh,
    whenToUseZh: overrides.whenToUseZh,
    placementZh:
      overrides.placementZh ??
      '子树节点本身不接子节点，而是放在你想“跳去另一棵树继续执行”的位置。它适合把长流程拆成几个更小、更好复用的流程块。',
    runningZh:
      overrides.runningZh ??
      '执行流会先进入被调用的那棵树。只要那棵树还没结束，这个子树节点就会保持“运行中”。',
    successZh:
      overrides.successZh ??
      '被调用的那棵树返回成功时，这个子树节点也返回成功。',
    failureZh:
      overrides.failureZh ??
      '被调用的那棵树返回失败，或你填错了子树 ID 导致调用不了时，这个子树节点会失败。',
    pitfallsZh: overrides.pitfallsZh,
    exampleZh: overrides.exampleZh,
  });
}

function withDefaultExampleValue(port: BtPortSchema, override?: BtPortDocOverride): string | undefined {
  return override?.exampleValueZh ?? port.defaultValue;
}

function buildGenericPortHint(entry: BtNodeRegistryEntry, port: BtPortSchema): string {
  if (port.direction === 'output') {
    return `这个端口主要负责把“${entry.labelZh}”算出来的结果写出去，一般不是让你手填真实业务值。通常保留默认黑板键即可；如果你改了名字，后面读取它的节点也要一起改。`;
  }

  if (port.bindingMode === 'blackboard') {
    return `这个端口默认接黑板值，不建议刚开始就改成别的写法。先确认上游节点会把结果写到这个黑板键里，再让当前节点继续读取它。`;
  }

  if (port.bindingMode === 'both') {
    return `这个端口既能直接填固定值，也能接黑板。刚开始不确定时：如果这个值是上游算出来的，就保留黑板写法；如果它一直是固定常量，就直接手填。`;
  }

  if (port.valueType === 'bool') {
    return `这是一个开关量参数，通常填 true 或 false。第一次配置时可以先沿用默认值，再根据行为是否符合预期来调整。`;
  }

  if (numberTypes.has(port.valueType.trim().toLowerCase())) {
    const defaultHint = port.defaultValue ? `第一次可以先用默认值 ${port.defaultValue}。` : '第一次可以先从一个保守的小值开始。';
    return `${defaultHint} 数值调大通常表示更宽松、更久或更远；数值调小通常表示更严格、更快或更近。调参时一次只改一个数字，方便看出影响。`;
  }

  return `这是一个文本类参数，通常需要填项目里约定好的名字、标签或脚本内容。刚开始不确定时，先照着已有 XML 里的写法填，不要临时发明新名字。`;
}

const portDocOverridesByKey: Record<string, BtPortDocOverride> = {
  'Parallel.success_count': {
    beginnerHintZh:
      '它决定“并行节点要看到几个孩子成功，才算整组成功”。第一次最稳妥的做法是保持 -1，也就是等所有子节点都成功；只有你很清楚要做“部分成功就算通过”时再改。',
    exampleValueZh: '-1',
  },
  'Parallel.failure_count': {
    beginnerHintZh:
      '它决定“看到几个孩子失败，就立刻判整组失败”。第一次通常保持 1，这样任意一个关键子任务失败就会尽快暴露问题。',
    exampleValueZh: '1',
  },
  'ParallelAll.max_failures': {
    beginnerHintZh:
      '它决定“最多允许几个孩子失败”。如果你想让整组流程尽量严格，先保持 1；只有在确实允许部分孩子失败的场景下再调大。',
    exampleValueZh: '1',
  },
  'Switch.variable': {
    beginnerHintZh:
      '这里填用来做分支判断的变量名或黑板键。它的值会和下面每个 case_ 条件逐个比较，所以名字和取值都要和上游保持一致。',
  },
  'Switch.case': {
    beginnerHintZh:
      '这里填某一路分支要匹配到的具体值。只有 variable 的当前值和这里完全匹配时，这一路才会被选中。',
  },
  'Repeat.num_cycles': {
    beginnerHintZh:
      '这里填要重复多少次。第一次调试时建议先用 1 或 2，确认逻辑没问题后再加大，避免一下子重复太多次把问题放大。',
    exampleValueZh: '1',
  },
  'RetryUntilSuccessful.num_attempts': {
    beginnerHintZh:
      '这里填最多重试几次。次数太小可能来不及恢复，次数太大又可能让错误被拖很久；第一次可先从 3 次开始。',
    exampleValueZh: '3',
  },
  'Delay.delay_msec': {
    beginnerHintZh:
      '这里填开始执行前要等多久，单位毫秒。1000 毫秒等于 1 秒。第一次可以先用 1000，看是否真的需要更长等待。',
    exampleValueZh: '1000',
  },
  'Timeout.msec': {
    beginnerHintZh:
      '这里填“最多等多久还没完成就判超时”，单位毫秒。不要一开始就设太小，否则正常动作也可能被误判失败。',
    exampleValueZh: '1000',
  },
  'Script.code': {
    beginnerHintZh:
      '这里写的是 BehaviorTree.CPP 的脚本语法，不是普通自然语言。刚开始最好从现有 XML 里复制一段类似写法再改变量名，不要从空白开始硬写。',
  },
  'ScriptCondition.code': {
    beginnerHintZh:
      '这里必须写能得到 true / false 的判断表达式。先保证表达式只做判断，不要把复杂赋值和判断混在一起，排错会更容易。',
  },
  'SubTree.ID': {
    beginnerHintZh:
      '这里必须填“要跳过去执行的那棵树”的 ID，不是中文名。最稳妥的做法是直接从左侧树列表或已有 XML 里照抄，避免手打拼错。',
  },
  'SubTree._autoremap': {
    beginnerHintZh:
      '保持 true 时，父树和子树里同名端口会自动共用一套黑板值，最适合小白先跑通流程。只有你明确要手动控制端口映射时，再考虑改成 false。',
    exampleValueZh: 'true',
  },
  'CheckManualRobot.distance_threshold': {
    beginnerHintZh:
      '这个值越小，要求手动机器人靠得越近才算通过；越大则越宽松。第一次先保持 0.5 米，现场确认太松或太紧再微调。',
    exampleValueZh: '0.5',
  },
  'CheckManualRobot.static_time': {
    beginnerHintZh:
      '这个值表示“对方至少静止多久才算稳定”。设得太短容易误判刚好经过，设得太长会让流程等太久。第一次先保持 2 秒。',
    exampleValueZh: '2.0',
  },
  'Rotate.angle': {
    beginnerHintZh:
      '这里填要旋转的角度，正负号代表方向。第一次尽量只用已有项目里验证过的角度，比如 90、-90、180、-180。',
  },
  'CheckKFS.grid_id': {
    beginnerHintZh:
      '这里填要检查的梅林格子编号。如果上游已经把目标格写到黑板里，就保留黑板绑定；如果这一步永远检查固定格子，再改成常量。',
  },
  'CheckKFS.expected_state': {
    beginnerHintZh:
      '这里填你希望看到的 KFS 状态枚举值。刚开始不要自造新字符串，直接沿用项目里已经出现过的状态名。',
    exampleValueZh: 'AUTO_KFS',
  },
  'CheckLoad.min_load': {
    beginnerHintZh:
      '这里填“至少装了多少个才算达标”。如果你只是想判断“车上有没有货”，可以从 1 开始。数值越大，条件越严格。',
    exampleValueZh: '0',
  },
  'CheckLoad.max_load': {
    beginnerHintZh:
      '这里填“最多允许装多少个”。它常和最小装载数一起组成一个区间。第一次可以先保持默认上限，再根据任务容量缩紧。',
    exampleValueZh: '3',
  },
  'SelectNextGrid.next_action': {
    beginnerHintZh:
      '这是输出口，通常保持默认黑板键 `{next_action}` 即可，让后面节点直接去读。只有你在同一棵树里同时维护多套“下一步动作”变量时，才需要换名字。',
    exampleValueZh: '{next_action}',
  },
  'SelectNextGrid.target_grid': {
    beginnerHintZh:
      '这是输出口，通常保持默认黑板键 `{target_grid}` 即可。后面要导航或更新地图的节点通常会继续读取它。',
    exampleValueZh: '{target_grid}',
  },
  'UpdateMapKFS.grid_id': {
    beginnerHintZh:
      '这里要告诉节点“准备更新哪一个格子”。如果这一步就是更新刚刚操作过的目标格，通常直接沿用上游传下来的黑板值最省事。',
  },
  'UpdateMapKFS.kfs_type': {
    beginnerHintZh:
      '这里填准备写回地图的 KFS 类型枚举值。第一次不要猜数字含义，先照着现有流程里出现过的值来填。',
    exampleValueZh: '0',
  },
  'NavToTopoNode.node_id': {
    beginnerHintZh:
      '这里填拓扑图里的节点 ID。它不是中文说明文字，而是导航系统认识的内部名字，最好直接复制项目里已有节点名。',
  },
  'NavToTaskPose.task_tag': {
    beginnerHintZh:
      '这里填场地图里已经定义好的任务标签。第一次不要手写新标签，直接用现有任务点名字更稳妥。',
  },
  'NavToTaskPose.grid_id': {
    beginnerHintZh:
      '如果这次导航是奔着某个梅林格子去的，可以在这里传入格子编号。若导航目标只靠 task_tag 就能确定，这个口可以保持空或继续用黑板。',
  },
  'ExecuteTopoRoute.route_tag': {
    beginnerHintZh:
      '这里填预定义路线的标签名。只有当那条路线已经在项目里存在时，这个动作才能按整段路线跑起来。',
  },
  'PlaceKFSGrid.grid_position': {
    beginnerHintZh:
      '这里填九宫格位置编号，通常是 1 到 9。第一次最好配合任务设计图或现有流程确认编号含义，避免左右或上下搞反。',
  },
  'PlaceKFSGrid.layer': {
    beginnerHintZh:
      '这里填要放到第几层。保持 0 表示让底层自己选层，是最适合先跑通的写法；只有你明确要强制某一层时再改成具体数字。',
    exampleValueZh: '0',
  },
  'PlaceKFSGrid.selected_layer': {
    beginnerHintZh:
      '这是输出口，用来把自动决策后实际选中的层号写到黑板里，通常不用改。只有后面节点要读另一个黑板键时才需要一起改名。',
  },
  'GimbalMove.pitch': {
    beginnerHintZh:
      '这里填云台上下抬头或低头的角度。第一次先用项目里已经验证过的角度，不要一下子给特别大的数，避免看不到目标。',
  },
  'GimbalMove.yaw': {
    beginnerHintZh:
      '这里填云台左右转动的角度。和俯仰角一样，第一次最好沿用现有流程里的数值，确认方向没反再调大范围。',
  },
  'FollowManualRobot.follow_distance': {
    beginnerHintZh:
      '这个值表示 R2 想和手动机器人保持多远。设太小容易贴得过近，设太大又可能跟丢。第一次先保持 1.5 米。',
    exampleValueZh: '1.5',
  },
  'FollowManualRobot.lost_timeout': {
    beginnerHintZh:
      '这个值表示“前车丢了多久后就判失败”。如果现场遮挡比较多可以略微调大，但不要大到丢失很久还在盲等。',
    exampleValueZh: '5.0',
  },
  'VisionStart.model_id': {
    beginnerHintZh:
      '这里填要启动的视觉模型或配置名。如果当前任务只有一套默认模型，也可以先留空，让底层走默认配置。',
  },
  'VisionSetModel.model_id': {
    beginnerHintZh:
      '这里填准备切换到的模型或配置 ID。它必须和视觉模块里已经存在的配置一致，不能临时编一个新名字。',
  },
  'WaitVisionTarget.target_attr': {
    beginnerHintZh:
      '这里填你想等到的目标属性，比如某种颜色、类别或识别标签。第一次最好直接复制现有流程里已经出现过的枚举值。',
  },
  'WaitVisionTarget.max_dist': {
    beginnerHintZh:
      '这里限制“多远的目标才算有效”。保持 -1 表示不额外限距离；如果你只想让近处目标触发，再填一个正数距离。',
    exampleValueZh: '-1.0',
  },
  'WaitVisionTarget.timeout': {
    beginnerHintZh:
      '这里填最多等多久，单位毫秒。1000 表示 1 秒。时间太短会经常来不及看到目标，时间太长又会让流程卡住太久。',
    exampleValueZh: '1000',
  },
};

function getPortOverrideKey(tagName: string, portName: string): string[] {
  if (tagName.startsWith('Switch') && portName.startsWith('case_')) {
    return [`Switch.case`, `${tagName}.${portName}`];
  }
  return [`${tagName}.${portName}`];
}

export function buildKnowledgeBasePortSchemas(
  entry: BtNodeRegistryEntry,
  overrides: Record<string, BtPortDocOverride> = {}
): BtKnowledgeBasePortSchema[] {
  return entry.portSchemas.map((port) => {
    const override =
      getPortOverrideKey(entry.tagName, port.name)
        .map((key) => overrides[key] ?? portDocOverridesByKey[key])
        .find(Boolean) ?? null;

    return {
      ...port,
      beginnerHintZh: override?.beginnerHintZh ?? buildGenericPortHint(entry, port),
      exampleValueZh: withDefaultExampleValue(port, override ?? undefined),
    };
  });
}

function buildGuideSearchTokens(guideZh: BtNodeGuideZh): string[] {
  return Object.values(guideZh).filter(Boolean);
}

function buildPortSearchTokens(portSchemas: BtKnowledgeBasePortSchema[]): string[] {
  return portSchemas.flatMap((port) => [
    port.name,
    port.labelZh,
    port.descriptionZh,
    port.beginnerHintZh,
    port.exampleValueZh ?? '',
    port.valueType,
    port.defaultValue ?? '',
  ]);
}

export function buildKnowledgeBaseSearchTokens(
  entry: BtNodeRegistryEntry,
  guideZh: BtNodeGuideZh,
  portSchemas: BtKnowledgeBasePortSchema[]
): string[] {
  return [
    entry.labelZh,
    entry.descriptionZh,
    entry.group,
    entry.tagName,
    ...entry.keywordsZh,
    ...entry.keywordsEn,
    ...buildGuideSearchTokens(guideZh),
    ...buildPortSearchTokens(portSchemas),
  ].filter(Boolean);
}

const nodeGuideByTag: Record<string, BtNodeGuideZh> = {
  Sequence: buildControlGuide('顺序节点', {
    overviewZh: '这是最常用的流程串联节点，相当于“先做 A，再做 B，再做 C”。',
    whenToUseZh: '当一串步骤必须按固定顺序全部做完，而且任何一步失败都应该立刻停下来时，用它最合适。',
    successZh: '只有所有子节点都成功时，它才返回成功。',
    failureZh: '只要有任意一个子节点失败，它就立刻返回失败，后面的步骤不会再执行。',
    pitfallsZh: '不要把“备选方案”挂在顺序节点下面，否则一条失败就整段中断。遇到“试 A 不行再试 B”的场景，要改用回退节点。',
    exampleZh: '例如在梅林区里，先“扫描周围环境”，再“选择下一格动作”，再“导航到任务位姿”，就很适合用顺序节点包起来。',
  }),
  SequenceWithMemory: buildControlGuide('记忆顺序节点', {
    overviewZh: '它和普通顺序节点很像，但会记住自己上次跑到哪一步，不会每次都从头重来。',
    whenToUseZh: '当一串步骤前面已经完成，后面某一步暂时没做完，而你希望下次继续从断点往后跑时，用它最省时间。',
    successZh: '所有子节点最终都完成时，它返回成功。',
    failureZh: '当前轮到的子节点明确失败时，它返回失败；下一次再次进入时，会从失败或未完成的那一步继续。',
    pitfallsZh: '如果前面的前置条件经常变化，就不要用记忆顺序节点，否则它可能跳过本该重新检查的前置步骤。这类场景更适合响应式顺序节点。',
    exampleZh: '例如“先确认入口条件，再执行一串较长动作”，且中间动作可能被打断后恢复时，就适合用记忆顺序节点。',
  }),
  SequenceStar: buildControlGuide('记忆顺序节点', {
    overviewZh: '这是旧版命名的记忆顺序节点，理解方式和 SequenceWithMemory 基本一样。',
    whenToUseZh: '只有在兼容旧 XML 写法或老流程时才需要特别关心它；使用习惯上可以把它当成记忆顺序节点来理解。',
    successZh: '所有子节点最终都完成时返回成功。',
    failureZh: '当前轮到的子节点失败时返回失败，并保留断点信息。',
    pitfallsZh: '它是兼容旧写法的名称，新人阅读时最容易误以为它和普通顺序节点完全一样。实际上它会记住进度，不会每次从头跑。',
    exampleZh: '如果你在旧项目 XML 里看到 SequenceStar，可以直接把它理解成“会记住进度的顺序节点”。',
  }),
  ReactiveSequence: buildControlGuide('响应式顺序节点', {
    overviewZh: '它会频繁回头重查前面的条件，相当于一边推进动作，一边不断确认“前提还成立吗”。',
    whenToUseZh: '当流程前面有守护条件，例如“只要前方没挡住就继续前进”，并且这些条件可能在执行过程中随时变化时，用它最合适。',
    successZh: '当前面条件始终成立，并且后面的子节点最终都成功时，它返回成功。',
    failureZh: '只要前面的任意守护条件突然不成立，它就会立刻返回失败或切回前面重新检查。',
    pitfallsZh: '不要把有明显副作用、每跑一次都很重的动作放在最前面，否则响应式重查时会被反复触发。前面更适合放轻量的条件判断。',
    exampleZh: '例如“先检查 R1 没挡路，再导航，再执行抓取”，并且导航过程中也想持续盯着挡路条件时，就适合响应式顺序节点。',
  }),
  Fallback: buildControlGuide('回退节点', {
    overviewZh: '它负责按顺序尝试多套备选方案，前一条不行就换下一条。',
    whenToUseZh: '当你想表达“先试首选方案，不行再试备胎方案”时，用它最清楚。',
    successZh: '只要有任意一个子节点成功，它就立刻返回成功，并停止继续尝试后面的方案。',
    failureZh: '所有子节点都失败时，它才返回失败。',
    pitfallsZh: '不要把“必须全做完”的步骤放进回退节点，否则前面一条成功后，后面的步骤就永远不会执行。必须全部完成的流程应该用顺序节点。',
    exampleZh: '例如“先走主路线，不通再走备用路线”，或者“先抓首选目标，不行再抓备用目标”就适合回退节点。',
  }),
  ReactiveFallback: buildControlGuide('响应式回退节点', {
    overviewZh: '它是会“抢占”的回退节点，高优先级分支一旦重新满足，就能立刻打断当前较低优先级方案。',
    whenToUseZh: '当你需要一边执行备选方案，一边随时观察更高优先级条件是否重新成立时，用它最合适。',
    successZh: '当前最高优先级且满足条件的那条分支成功时，它返回成功。',
    failureZh: '所有分支都不成立或都失败时，它返回失败。',
    pitfallsZh: '低优先级分支里的长动作可能会被高优先级条件反复打断，所以低优先级方案最好能安全重入，不要要求一次跑到底。',
    exampleZh: '例如“优先处理紧急目标；如果紧急目标还没出现，就先做常规巡逻”，并且一旦紧急目标出现就要立刻切回去处理。',
  }),
  Parallel: buildControlGuide('并行节点', {
    overviewZh: '它会同时推进多个子节点，再根据你设置的成功阈值和失败阈值统一结算。',
    whenToUseZh: '当几件事可以同时做，比如“边移动边等待某个状态”或“同时监听多个条件”时，可以考虑并行节点。',
    successZh: '达到成功阈值 success_count 后，它返回成功。把 success_count 设为 -1 时，表示所有子节点都成功才算成功。',
    failureZh: '达到失败阈值 failure_count 后，它返回失败。',
    pitfallsZh: '并行节点最容易出错的地方是阈值配置。阈值太宽松会过早通过，阈值太严格又可能永远等不到。第一次配置时先用最保守的默认值。',
    exampleZh: '例如一边执行导航，一边让另一个子节点持续观察超时或特殊条件，就可以考虑用并行节点来包住两条支线。',
  }),
  ParallelAll: buildControlGuide('全并行节点', {
    overviewZh: '它会让多个子节点一起跑，但更强调“等大家都结束后再统一算总账”。',
    whenToUseZh: '当你想并行做几件事，而且必须等整组都结束后才能判断下一步时，用它更贴近需求。',
    successZh: '所有子节点都结束，并且失败数量没有超过 max_failures 时，它返回成功。',
    failureZh: '失败子节点数量超过 max_failures 时，它返回失败。',
    pitfallsZh: '只要里面有一个特别慢的子节点，整组都会被拖住，所以不要把“很快能结束”和“可能长期运行”的任务随便混在一个全并行节点里。',
    exampleZh: '例如同时做几项互不依赖的准备动作，只有准备动作整体结束后，才允许进入下一个大阶段。',
  }),
  IfThenElse: buildControlGuide('条件分支节点', {
    overviewZh: '它就是行为树里的“如果……那么……否则……”。',
    whenToUseZh: '当流程必须先看一个条件，再在“满足”和“不满足”两条路之间二选一时，用它最直观。',
    placementZh:
      '把它放在你需要做分支判断的位置。第 1 个子节点应该是条件，第 2 个是条件成立后执行的分支，第 3 个可选，用来放条件不成立时执行的备用分支。',
    successZh: '被选中的那条分支成功时，它返回成功。',
    failureZh: '条件本身报错，或被选中的那条分支失败时，它返回失败。',
    pitfallsZh: '子节点顺序千万别挂反。最常见错误是把动作放在第 1 个位置，结果它被当成“条件节点”来使用，整段含义就乱了。',
    exampleZh: '例如“如果退出条件成立，就走收尾分支；否则继续主任务”就很适合 IfThenElse。',
  }),
  WhileDoElse: buildControlGuide('循环分支节点', {
    overviewZh: '它表示“只要条件成立，就反复做主体；条件不成立时，再走另一条路”。',
    whenToUseZh: '当你有一个会持续重复的主体动作，并且这个重复要由前置条件控制时，用它会比手写很多重复节点更清楚。',
    placementZh:
      '第 1 个子节点放循环条件，第 2 个放循环主体，第 3 个可选，用来放条件不成立后要执行的收尾或备用分支。',
    successZh: '条件结束后，如果走到备用分支且该分支成功，它返回成功；或者实现上的主体循环完整结束后也可能返回成功。',
    failureZh: '条件检查异常、主体失败，或备用分支失败时，它返回失败。',
    pitfallsZh: '循环主体必须能自然结束或被外部条件终止，否则很容易形成一直跑不完的循环。设计时一定要先想清楚“什么时候停”。',
    exampleZh: '例如“只要还在等待某个机会，就继续重复扫描；不再需要等待时，走退出分支”。',
  }),
  RoundRobin: buildControlGuide('轮询节点', {
    overviewZh: '它会轮流把不同子节点当作“第一个候选人”来尝试，避免永远只偏向同一路。',
    whenToUseZh: '当几套方案地位差不多，你不希望每次都从同一个方案开始试时，用轮询节点更公平。',
    successZh: '某一轮里有子节点成功时，它返回成功。',
    failureZh: '这一轮把所有子节点都试完仍然没有成功时，它返回失败。',
    pitfallsZh: '轮询节点不是“顺序必须全做完”的工具，也不是“高优先级抢占”的工具。它更适合多个平级候选方案轮着尝试。',
    exampleZh: '例如几个平级抓取姿态都可以试，但不想永远固定先试第一个时，可以用轮询节点轮着开头。',
  }),
  Inverter: buildDecoratorGuide('结果反转装饰器', {
    overviewZh: '它会把里面那个子节点的“成功”和“失败”对调。',
    whenToUseZh: '当你已经有一个现成条件，但业务语义想表达“不要满足这个条件”时，用它比重新写一个相反条件更省事。',
    successZh: '子节点失败时，它返回成功。',
    failureZh: '子节点成功时，它返回失败。',
    pitfallsZh: '它只会反转成功和失败，不会把“运行中”变成别的状态。很多新手以为它能把所有状态都反过来，这是错误的。',
    exampleZh: '例如已有节点能判断“前方被挡住”，那想表达“前方没有被挡住”时，就可以在外面包一个 Inverter。',
  }),
  ForceSuccess: buildDecoratorGuide('强制成功装饰器', {
    overviewZh: '不管里面那个子节点最终是成功还是失败，出来后都会被改写成成功。',
    whenToUseZh: '当你只想“顺手做一下”，失败了也不想影响主流程时，可以用它把失败吞掉。',
    successZh: '子节点结束后，它总是返回成功。',
    failureZh: '这个装饰器几乎不会以失败结束，除非底层实现本身发生异常。',
    pitfallsZh: '它会把真正的问题隐藏掉，所以不要随手包在关键动作外面。只有你明确接受“失败也继续往下走”时才应该使用。',
    exampleZh: '例如某个可选清理动作失败也没关系，那可以外面包一个 ForceSuccess，避免主流程被中断。',
  }),
  ForceFailure: buildDecoratorGuide('强制失败装饰器', {
    overviewZh: '不管里面那个子节点最终是什么结果，出来后都会被改写成失败。',
    whenToUseZh: '当你想借一个节点“做动作”，但无论结果如何都要让外层把它当成失败处理时，可以用它。',
    successZh: '这个装饰器通常不会返回成功。',
    failureZh: '子节点结束后，它会统一返回失败。',
    pitfallsZh: '它会强行把结果压成失败，所以很容易让流程读起来反直觉。除非你非常确定外层就是要一个失败信号，否则不要滥用。',
    exampleZh: '例如某个探测动作执行完后，你就是想让外层回退去试别的方案，就可以考虑用 ForceFailure 包起来。',
  }),
  Repeat: buildDecoratorGuide('重复装饰器', {
    overviewZh: '它会把同一个子节点按设定次数重复执行。',
    whenToUseZh: '当你想固定重复几次同一动作，而不是手动复制很多份同样节点时，用它更清楚。',
    successZh: '子节点按要求成功执行完指定次数后，它返回成功。',
    failureZh: '重复过程中，只要子节点失败，它就提前返回失败。',
    pitfallsZh: '不要拿它去做“无限循环”。它更适合固定次数的重复任务；如果次数太大，出问题时会很难看出到底卡在第几次。',
    exampleZh: '例如想连续尝试 3 次轻量探测动作，再决定是否进入下一步，就可以用 Repeat 包住那个探测节点。',
  }),
  RetryUntilSuccessful: buildDecoratorGuide('重试直到成功装饰器', {
    overviewZh: '它会在子节点失败时继续重试，直到成功或用完重试次数。',
    whenToUseZh: '当某个动作偶尔会因为瞬时原因失败，但重试几次通常能恢复时，这个装饰器很实用。',
    successZh: '只要某一次尝试成功，它就立刻返回成功。',
    failureZh: '重试次数全部用完还没成功时，它返回失败。',
    pitfallsZh: '不要把它包在“必然失败”的动作外面，否则只是在浪费时间。先确认失败确实有机会通过重试恢复，再决定要不要用。',
    exampleZh: '例如等待手动机器人就位后组装时，偶发失败可以再试几次，就很适合用 RetryUntilSuccessful。',
  }),
  KeepRunningUntilFailure: buildDecoratorGuide('持续运行直到失败装饰器', {
    overviewZh: '它会把子节点的成功也继续当成“还要接着跑”，直到子节点真正失败才停下来。',
    whenToUseZh: '当你想让某个检查或动作持续重复工作，只有在失败时才结束整段逻辑时，可以用它。',
    successZh: '这个装饰器通常不会返回成功；子节点成功时，它会继续保持运行中。',
    failureZh: '只有子节点失败时，它才结束并返回失败。',
    pitfallsZh: '很多新人会以为子节点成功后外层也会成功，其实不会。只要名字里有 “UntilFailure”，就要先理解它是“成功也继续跑”。',
    exampleZh: '例如让一个轻量检查动作持续工作，直到某次检查明确失败后再让外层感知到异常。',
  }),
  Delay: buildDecoratorGuide('延时装饰器', {
    overviewZh: '它会先等一段时间，再真正开始执行里面的子节点。',
    whenToUseZh: '当你需要给机械动作、环境稳定或对方就位留一点缓冲时间时，可以在目标节点外面包一个 Delay。',
    successZh: '等待结束后，如果子节点成功，它也返回成功。',
    failureZh: '等待结束后，如果子节点失败，它也返回失败。',
    pitfallsZh: 'Delay 只是在开始前等一下，不会自动解决底层动作超时问题。真正想限制“最长执行多久”，应该用 Timeout。',
    exampleZh: '例如想在云台移动前先等 1 秒，让上一动作完全稳定下来，就可以在外面包一个 Delay。',
  }),
  Timeout: buildDecoratorGuide('超时装饰器', {
    overviewZh: '它会给子节点加一个最晚完成时间，超过这个时间还没结束就视为超时。',
    whenToUseZh: '当你担心某个动作或等待节点可能卡太久，想给主流程加一道保险时，用 Timeout 很合适。',
    successZh: '子节点在规定时间内成功完成时，它返回成功；如果子节点在时限内失败，也会按失败返回。',
    failureZh: '子节点超时未完成，或在时限内本身失败时，它返回失败。',
    pitfallsZh: '超时时间不要一开始就设得特别短，否则正常动作也会被你自己判死。先从一个偏宽松的时间开始，再逐步缩紧。',
    exampleZh: '例如等待视觉目标时，不想一直等下去，就可以在外面再包一个 Timeout 给整段等待上限。',
  }),
  Script: buildActionGuide('脚本赋值节点', {
    overviewZh: '它不是机械动作，而是用一小段脚本直接读写黑板变量。',
    whenToUseZh: '当你只想在行为树里做一些轻量变量处理、状态改写或简单赋值时，用它比专门写一个新动作节点更方便。',
    runningZh: '大多数情况下它会很快执行完成，不会长时间停在运行中。',
    successZh: '脚本成功执行并完成变量读写时，它返回成功。',
    failureZh: '脚本语法错误、表达式求值失败或引用了错误变量时，它可能失败。',
    pitfallsZh: '不要把太复杂的业务逻辑都塞进脚本节点里，否则树面上看不出真正发生了什么。脚本节点更适合做小而明确的变量处理。',
    exampleZh: '例如把某个黑板变量重置为默认值，或把上一步结果整理成后面节点更容易读取的格式。',
  }),
  ScriptCondition: buildConditionGuide('脚本条件节点', {
    overviewZh: '它用脚本表达式回答“当前条件成立吗”。',
    whenToUseZh: '当现成条件节点不够用，而你又只需要一个轻量判断式时，可以用脚本条件节点临时补上。',
    runningZh: '它通常会很快完成判断，不会长时间停在运行中。',
    successZh: '表达式结果为 true 时，它返回成功。',
    failureZh: '表达式结果为 false，或表达式本身写错时，它返回失败。',
    pitfallsZh: '脚本条件最容易出错的地方是变量名和语法。第一次写时最好先从简单判断开始，别一上来就堆一大串复杂逻辑。',
    exampleZh: '例如判断 `next_action == "GRAB"`，根据黑板里的动作名决定后续要走哪条流程。',
  }),
  AlwaysSuccess: buildActionGuide('恒成功节点', {
    overviewZh: '它什么也不做，只是到了这里就直接返回成功。',
    whenToUseZh: '当你需要一个“占位成功”、一个空实现，或者想让某条支线无条件通过时，可以使用它。',
    runningZh: '它几乎不会停留在运行中，通常立刻结束。',
    successZh: '执行到它时会直接成功。',
    failureZh: '正常情况下它不会失败。',
    pitfallsZh: '它太容易让流程看起来“总能过”，所以不要用它掩盖真实缺失的动作。只有你真的需要一个占位成功时再用。',
    exampleZh: '例如先搭树形结构占位置，后续再把 AlwaysSuccess 换成真正动作节点。',
  }),
  AlwaysFailure: buildActionGuide('恒失败节点', {
    overviewZh: '它什么也不做，只是到了这里就直接返回失败。',
    whenToUseZh: '当你想故意让外层走失败分支，或给某条分支放一个明确的“这里不允许通过”占位节点时，可以用它。',
    runningZh: '它通常立刻结束，不会长时间运行。',
    successZh: '正常情况下它不会成功。',
    failureZh: '执行到它时会直接失败。',
    pitfallsZh: '它会把外层流程强制带到失败分支，所以不要在主流程里随手留下它，除非你就是想让这条路必定失败。',
    exampleZh: '例如某条支线只是为了占位提醒“这里未来要实现备用失败逻辑”，可以先放 AlwaysFailure。',
  }),
  SubTree: buildSubtreeGuide({
    overviewZh: '它会把执行流切到另一棵行为树里，等那棵树跑完再把结果带回来。',
    whenToUseZh: '当一段流程已经足够长、足够独立，或者你想在多个地方复用同一套流程时，用子树拆分最清楚。',
    successZh: '被调用子树成功结束时，它返回成功。',
    failureZh: '被调用子树失败、ID 填错，或端口映射有问题导致子树无法正常工作时，它返回失败。',
    pitfallsZh: '最常见问题是把中文树名当成 ID 填进去，或者改了子树端口名字却忘了同步上游黑板变量。子树节点看起来简单，实际最怕“名字对不上”。',
    exampleZh: '例如把“梅林主流程”拆成入口、循环、退出三棵树后，就可以在主树里通过 SubTree 节点逐段调用。',
  }),
  GrabTip: buildActionGuide('抓取矛头', {
    overviewZh: '它会控制武馆区机构去抓起矛头，是武馆流程里的核心执行动作之一。',
    whenToUseZh: '当机器人已经到达抓取位置，并且流程确实进入“拿矛头”阶段时，用这个节点真正发起抓取。',
    successZh: '机构完成抓取，系统确认矛头已经抓起时返回成功。',
    failureZh: '没抓到、机构动作报错、姿态不对或超时时返回失败。',
    pitfallsZh: '不要在没到位或前置姿态没准备好的情况下直接调用它，否则看起来像“节点失败”，本质上却是前置流程没铺好。',
    exampleZh: '武馆区常见写法是：先进入武馆主流程，再执行“抓取矛头”，然后再等待手动机器人配合完成组装。',
  }),
  AssembleWeapon: buildActionGuide('组装武器', {
    overviewZh: '它会驱动机构完成武器组装动作。',
    whenToUseZh: '当手动机器人已经靠近、姿态稳定，并且抓矛头步骤已经完成后，再调用它开始组装。',
    successZh: '组装动作完整结束并确认成功时返回成功。',
    failureZh: '手动机器人没就位、机构动作失败或超时时返回失败。',
    pitfallsZh: '它通常不是孤零零单独使用，而是放在“检查手动机器人就位”之后。如果前面的检查不做，失败率会明显上升。',
    exampleZh: '武馆区典型写法是：`CheckManualRobot -> AssembleWeapon`，再外层包一个 `RetryUntilSuccessful` 增强容错。',
  }),
  CheckManualRobot: buildConditionGuide('检查手动机器人', {
    overviewZh: '它负责判断手动机器人是否已经靠近，并且稳定停在可组装的位置。',
    whenToUseZh: '只要后面的动作依赖手动机器人配合，例如组装武器前，就应该先过这一关。',
    successZh: '手动机器人距离足够近，而且连续静止达到设定时间时返回成功。',
    failureZh: '距离太远、停得不稳、还在移动，或一直等不到合格状态时返回失败。',
    pitfallsZh: '新手最常见的问题是把距离阈值设得太严或静止时间设得太短，导致流程要么一直过不去，要么刚好经过也被误判成已就位。',
    exampleZh: '在武馆区里，常见模式是“检查手动机器人 -> 通过后执行组装武器”。',
  }),
  StairClimb: buildActionGuide('上楼梯', {
    overviewZh: '它会驱动机器人执行上楼梯动作，把平台高度切到更高层。',
    whenToUseZh: '当流程下一步任务明确在更高平台，且当前已经处于允许上楼梯的位置时，才应该调用它。',
    successZh: '机器人顺利完成上楼梯并到达目标高度时返回成功。',
    failureZh: '上楼梯动作失败、被卡住、姿态不对或超时时返回失败。',
    pitfallsZh: '楼梯动作对前置姿态要求高。不要把它当普通平地导航来理解，最好在调用前确保机器人真的在楼梯入口附近。',
    exampleZh: '例如梅林区从下层进入上层抓取区域前，可以先执行上楼梯动作，再继续后面的机构任务。',
  }),
  StairDescend: buildActionGuide('下楼梯', {
    overviewZh: '它会驱动机器人执行下楼梯动作，把机器人带回下层区域。',
    whenToUseZh: '当上层任务结束，需要安全回到底层继续后续流程时，用它完成下降。',
    successZh: '机器人顺利完成下楼梯并回到底层时返回成功。',
    failureZh: '下降过程失败、路径姿态不对或超时时返回失败。',
    pitfallsZh: '和上楼梯一样，它不是普通导航动作。调用前应先保证机器人处在正确的楼梯起始位置，避免一开始就失败。',
    exampleZh: '例如上层抓取结束后，需要回到底层继续导航或退出流程，就可以安排下楼梯动作。',
  }),
  GrabKFS: buildActionGuide('抓取 KFS', {
    overviewZh: '它会控制机构把当前目标格里的 KFS 抓到车上。',
    whenToUseZh: '当地图和导航都已经把机器人带到目标格附近，并确认这一格确实值得抓取时，才调用它。',
    successZh: '成功抓到目标 KFS 并完成收取时返回成功。',
    failureZh: '没抓到、目标状态不对、机构动作失败或超时时返回失败。',
    pitfallsZh: '如果前面没有先确认格子状态，或者导航没真正到位，抓取 KFS 失败时很难第一眼看出根因，所以最好和状态检查、定位动作配套使用。',
    exampleZh: '梅林区常见模式是：先扫描环境并选出目标格，再导航到目标格，最后执行抓取 KFS。',
  }),
  MechUpMerlin: buildActionGuide('梅林机构上升', {
    overviewZh: '它会把机构切到梅林区任务需要的抬升姿态。',
    whenToUseZh: '当后面动作需要更高姿态，例如准备跨越、抓取或避让时，可以先调用它做姿态准备。',
    successZh: '机构成功达到目标上升姿态时返回成功。',
    failureZh: '机构没到位、动作报错或超时时返回失败。',
    pitfallsZh: '不要把它和真正抓取动作混为一谈。它更像“准备姿态”，成功不代表任务已经完成，只代表后面动作有了更好的起手姿势。',
    exampleZh: '例如抓取 KFS 前，先做梅林机构上升，再根据当前任务进入抓取或旋转动作。',
  }),
  MechDownMerlin: buildActionGuide('梅林机构下降', {
    overviewZh: '它会把机构切回更低的梅林任务姿态，方便进入另一种动作或通行状态。',
    whenToUseZh: '当前面任务结束，后面的通行、退出或下一步抓取需要更低姿态时，用它进行姿态回收。',
    successZh: '机构顺利下降到目标姿态时返回成功。',
    failureZh: '机构下降失败、被卡住或超时时返回失败。',
    pitfallsZh: '别把它理解成“任务结束节点”。它只负责姿态切换，真正后续要做什么，仍然要靠外层流程继续安排。',
    exampleZh: '例如完成高位动作后，先让机构下降回安全姿态，再继续导航到下一处任务点。',
  }),
  Rotate: buildActionGuide('旋转动作', {
    overviewZh: '它会按给定角度调整机器人或机构当前朝向。',
    whenToUseZh: '当后面的抓取、导航或机构动作对朝向有要求时，可以先用它把方向调到位。',
    successZh: '旋转角度执行完成并确认到位时返回成功。',
    failureZh: '角度执行失败、被打断或超时时返回失败。',
    pitfallsZh: '新手最容易把正负号搞反。第一次用新角度时，最好先在已有验证过的几个角度里挑选，别直接填随手猜的数。',
    exampleZh: '例如抓某个侧向目标前，先旋转 90 度对准，再继续抓取动作。',
  }),
  CheckKFS: buildConditionGuide('检查 KFS 状态', {
    overviewZh: '它会读取某个格子的 KFS 状态，判断是否和你期望的一样。',
    whenToUseZh: '当流程需要先确认“这个格子值不值得去”“里面是不是目标类型的 KFS”时，就应该先用它做判断。',
    successZh: '指定格子的当前 KFS 状态和 expected_state 匹配时返回成功。',
    failureZh: '格子状态不匹配、格子编号无效或状态暂时拿不到时返回失败。',
    pitfallsZh: '如果 grid_id 来自黑板，上游必须先把目标格写进去；否则节点表面上像“状态不对”，实际上可能只是没拿到正确格子编号。',
    exampleZh: '例如先判断目标格是不是 `AUTO_KFS`，只有成立时才继续去抓取。',
  }),
  CheckLoad: buildConditionGuide('检查装载数量', {
    overviewZh: '它会检查车上当前装了多少个 KFS，并判断这个数量是否落在允许区间里。',
    whenToUseZh: '当流程需要根据当前负载决定“还能不能继续抓”“是不是该去放置”时，用它最直接。',
    successZh: '当前装载数介于最小值和最大值之间时返回成功。',
    failureZh: '当前装载数小于最小值或大于最大值时返回失败。',
    pitfallsZh: '最常见问题是把最小值和最大值写反，或者区间太窄导致流程永远过不去。第一次配置时建议先用比较宽的区间。',
    exampleZh: '例如当车上还没满载时继续抓取；一旦装载数接近上限，就改走放置或退出流程。',
  }),
  ScanSurroundings: buildActionGuide('扫描周围环境', {
    overviewZh: '它会刷新周围环境、格子状态和决策所需的感知信息。',
    whenToUseZh: '当后面的判断依赖“最新地图状态”时，通常应该先做一次扫描，再让决策节点基于新数据工作。',
    runningZh: '它可能需要短时间收集和整理状态，所以会短暂保持运行中，直到刷新完成。',
    successZh: '周围环境和状态刷新完成后返回成功。',
    failureZh: '刷新失败、数据拿不到或底层模块报错时返回失败。',
    pitfallsZh: '不要以为扫描一次后整段流程里数据就永远新鲜。如果环境变化快，关键决策前最好重新扫描一次。',
    exampleZh: '例如梅林区常见写法是：先扫描周围环境，再选择下一格动作，避免用旧数据做决策。',
  }),
  SelectNextGrid: buildActionGuide('选择下一格动作', {
    overviewZh: '它会根据当前地图和任务状态，帮你算出下一步该做什么，以及目标格子是哪一个。',
    whenToUseZh: '当流程进入“接下来该去哪、该抓哪一格”的决策阶段时，用它生成后续节点要消费的结果。',
    runningZh: '它通常会很快做完决策并写出结果，不会长时间运行。',
    successZh: '成功算出下一步动作和目标格，并把结果写到黑板后返回成功。',
    failureZh: '当前状态不足以做决策、地图数据异常或底层决策失败时返回失败。',
    pitfallsZh: '它的重点不是“自己干活”，而是“给后面节点产出结果”。如果后面节点读的黑板键名和这里写出去的不一致，流程就会像断了一样。',
    exampleZh: '典型写法是：先扫描周围环境，再执行选择下一格动作，然后导航节点读取 `{target_grid}` 去目标格。',
  }),
  CheckExitCondition: buildConditionGuide('检查退出条件', {
    overviewZh: '它会判断当前梅林流程是不是已经到了该收尾退出的时候。',
    whenToUseZh: '当一段循环任务需要定期检查“还要不要继续干”时，把它放在分支前做退出判断最清楚。',
    successZh: '退出条件已经满足时返回成功，表示可以走收尾或退出流程。',
    failureZh: '退出条件还没满足时返回失败，表示主流程还要继续。',
    pitfallsZh: '不要把它当成“任务成功节点”。它只是告诉你“该不该退出”，不是告诉你“任务已经一定完成得很好”。',
    exampleZh: '例如在梅林主循环里，先检查退出条件；满足就走退出分支，不满足就继续下一轮决策。',
  }),
  CheckR1Blocking: buildConditionGuide('检查 R1 阻挡', {
    overviewZh: '它会判断前方是不是被手动机器人 R1 挡住了。',
    whenToUseZh: '当后续动作依赖前方通路畅通，例如准备进入某段通道或开始抓取前，就可以先用它做守护条件。',
    successZh: '检测到 R1 确实在阻挡时返回成功。',
    failureZh: '没挡住、状态不明确或未检测到阻挡时返回失败。',
    pitfallsZh: '这个节点的“成功”语义是“挡住了”，很容易和“通路正常”搞反。想表达“没挡住”，通常要在外面包一个 Inverter。',
    exampleZh: '例如某条分支专门处理“被 R1 挡住”的情况，就可以先用 CheckR1Blocking 作入口条件。',
  }),
  IncrementKFSCount: buildActionGuide('累加 KFS 数量', {
    overviewZh: '它会把“车上当前 KFS 数量”加一，用来同步抓取后的内部状态。',
    whenToUseZh: '每次成功抓到一个 KFS 后，都应该尽快更新计数，让后面判断装载数的节点能看到最新值。',
    runningZh: '它通常会很快完成，不会长时间运行。',
    successZh: '计数更新完成后返回成功。',
    failureZh: '黑板写入失败或内部状态更新异常时返回失败。',
    pitfallsZh: '如果你抓到了 KFS 却忘了更新数量，后面的 CheckLoad 一类节点就会拿到旧数据，流程判断会越来越偏。',
    exampleZh: '典型用法是：`GrabKFS` 成功后，紧接着调用 `IncrementKFSCount` 更新车载数量。',
  }),
  UpdateMapKFS: buildActionGuide('更新地图中的 KFS 状态', {
    overviewZh: '它会把某个目标格子的 KFS 状态重新写回地图，让后续决策看到最新现场情况。',
    whenToUseZh: '当你已经对某个格子做了抓取、放置或状态改变后，应该尽快用它把地图同步更新。',
    successZh: '地图中的目标格状态成功更新后返回成功。',
    failureZh: '格子编号错误、状态值不合法或地图更新失败时返回失败。',
    pitfallsZh: '它更新的是“决策用地图”，不是自动推断出来的真实世界。动作做完后如果不手动同步，后面节点仍可能按照旧地图继续决策。',
    exampleZh: '例如抓取某格的 KFS 成功后，马上把那个格子的状态更新为“已被取走”。',
  }),
  NavToTopoNode: buildActionGuide('导航到拓扑节点', {
    overviewZh: '它会让机器人导航到指定的拓扑节点位置。',
    whenToUseZh: '当任务目标已经能明确写成一个拓扑节点 ID，而不是临时计算的位置时，用这个动作最直接。',
    successZh: '机器人成功到达指定拓扑节点时返回成功。',
    failureZh: '导航规划失败、执行失败、节点不存在或超时时返回失败。',
    pitfallsZh: '这里填的是系统里的拓扑节点 ID，不是中文备注名。第一次配置时最好直接复制已有节点名，避免因为名字写错导致导航失败。',
    exampleZh: '例如流程里明确要求“先去入口节点等待”，就可以用 NavToTopoNode 指向那个入口节点。',
  }),
  NavToTaskPose: buildActionGuide('导航到任务位姿', {
    overviewZh: '它会让机器人走到某个任务标签或目标格所对应的位置。',
    whenToUseZh: '当目标位置不是固定拓扑点，而是和任务标签、格子编号之类的业务信息绑定时，用它更灵活。',
    successZh: '机器人成功到达目标任务位姿时返回成功。',
    failureZh: '标签或格子解释失败、导航失败或超时时返回失败。',
    pitfallsZh: '如果 task_tag 和 grid_id 同时都在用，一定要先搞清楚当前底层更信哪一个；否则你以为在去 A 点，实际可能被另一个参数带去了别处。',
    exampleZh: '例如选择下一格动作后，把 `{target_grid}` 交给 NavToTaskPose，让机器人自动走到对应格位。',
  }),
  ExecuteTopoRoute: buildActionGuide('执行拓扑路径', {
    overviewZh: '它会按预定义路线标签，一次性执行整段拓扑路径。',
    whenToUseZh: '当路线本身已经提前规划好，而且你更想复用“固定路线模板”而不是临场算点到点导航时，用它很方便。',
    successZh: '整段预定义路线执行完成时返回成功。',
    failureZh: '路线标签不存在、路径执行被打断或超时时返回失败。',
    pitfallsZh: '它依赖项目里已经存在的路线标签，不是现填一个新名字就能跑。第一次使用前先确认这条路线已经在系统里定义好。',
    exampleZh: '例如从出发位到某个比赛区域的固定进场路线，可以直接配置成一条 topo route 反复复用。',
  }),
  MechUpDuel: buildActionGuide('对抗机构抬升', {
    overviewZh: '它会把对抗区机构切到需要的抬升姿态，为放置或跟随机构动作做准备。',
    whenToUseZh: '当后面的对抗区动作需要更高机构姿态时，先执行它进行姿态准备。',
    successZh: '机构成功到达对抗区抬升姿态时返回成功。',
    failureZh: '机构抬升失败、姿态没到位或超时时返回失败。',
    pitfallsZh: '它只是准备动作，不代表放置已经完成。不要把“机构抬升成功”误当成“对抗任务已经成功”。',
    exampleZh: '例如准备把 KFS 放到九宫格前，先抬升机构，再进入真正的放置动作。',
  }),
  PlaceKFSGrid: buildActionGuide('放置 KFS 到九宫格', {
    overviewZh: '它会把 KFS 放到指定九宫格位置，并可按需要放到特定层。',
    whenToUseZh: '当流程已经确认目标九宫格位置，并且机器人姿态允许放置时，用它发起真正的放置动作。',
    successZh: 'KFS 成功放到目标九宫格后返回成功。',
    failureZh: '目标位置不对、层号不可达、机构动作失败或超时时返回失败。',
    pitfallsZh: '最常见问题是九宫格编号和层号含义搞混。位置编号管“放哪一格”，层号管“堆在第几层”，两者不是一回事。',
    exampleZh: '例如先由上游决策选出九宫格 5 号位，再调用 PlaceKFSGrid 执行放置，并把实际层号写回黑板。',
  }),
  PlaceKFSGround: buildActionGuide('放置 KFS 到地面', {
    overviewZh: '它会把 KFS 放到指定地面位置，而不是放进九宫格层位。',
    whenToUseZh: '当任务要求把 KFS 直接放到地面指定位置，而不是堆叠到格子层里时，用这个动作。',
    successZh: 'KFS 成功放到目标地面位置时返回成功。',
    failureZh: '地面目标位不合适、动作失败或超时时返回失败。',
    pitfallsZh: '不要把它和 PlaceKFSGrid 混用。前者针对地面放置，后者针对九宫格分层放置，业务含义完全不同。',
    exampleZh: '例如对抗区某个流程要求直接卸到地面缓冲区时，就应该使用 PlaceKFSGround。',
  }),
  GimbalMove: buildActionGuide('云台移动', {
    overviewZh: '它会把云台转到目标俯仰角和偏航角，让相机或机构朝向目标方向。',
    whenToUseZh: '当后面的视觉识别、瞄准或观察步骤依赖一个特定视角时，先用它调整云台方向。',
    successZh: '云台成功转到目标角度时返回成功。',
    failureZh: '角度执行失败、超时或云台控制异常时返回失败。',
    pitfallsZh: '俯仰和偏航经常一起出现，新手最容易把“上下角度”和“左右角度”填反。第一次配置时先分别小幅调整，确认方向没反再继续。',
    exampleZh: '例如在等待视觉目标前，先把云台转到更容易看到目标的角度，再启动等待动作。',
  }),
  FollowManualRobot: buildActionGuide('跟随手动机器人', {
    overviewZh: '它会让自动机器人在对抗区跟随手动机器人前进，并尽量保持设定距离。',
    whenToUseZh: '当对抗区流程需要 R2 跟着手动机器人走，而不是自己独立规划路线时，用它最贴切。',
    successZh: '跟随阶段按要求完成，或底层判定本次跟随任务正常结束时返回成功。',
    failureZh: '前车丢失太久、跟随控制失败或超时时返回失败。',
    pitfallsZh: '跟随距离和丢失超时是两个最关键参数。距离太小容易贴得过近，超时太短又容易因为短暂遮挡就失败。',
    exampleZh: '例如对抗区需要贴着手动机器人移动到指定区域时，可以直接用 FollowManualRobot 承担这一段。',
  }),
  VisionStart: buildActionGuide('启动视觉', {
    overviewZh: '它会启动视觉识别模块，并在需要时切到指定模型。',
    whenToUseZh: '当后面的流程要依赖视觉结果时，先明确启动视觉，避免后续等待目标时其实根本没人工作。',
    successZh: '视觉模块正常启动并准备好对外提供识别结果时返回成功。',
    failureZh: '模型切换失败、视觉模块启动失败或超时时返回失败。',
    pitfallsZh: '不要默认视觉一直开着。流程里明确写一个 VisionStart，能让阅读者和调试者都更清楚“从哪一步开始需要视觉”。',
    exampleZh: '例如想先切到某个模型，再等待视觉目标出现，就可以先执行 VisionStart。',
  }),
  VisionStop: buildActionGuide('停止视觉', {
    overviewZh: '它会停止视觉识别，把相关算力和资源释放出来。',
    whenToUseZh: '当后面的流程暂时不再需要视觉，或者你想在切换任务阶段主动收掉视觉模块时，用它结束视觉工作。',
    successZh: '视觉模块正常停止并完成资源释放时返回成功。',
    failureZh: '视觉模块停止失败或状态异常时返回失败。',
    pitfallsZh: '不要一看到识别完成就立刻停视觉，先确认后面真的不再依赖视觉，否则刚关掉又得马上重启，流程会变得很绕。',
    exampleZh: '例如等待目标并完成抓取后，后面流程暂时不再需要识别，就可以加一个 VisionStop 收尾。',
  }),
  VisionSetModel: buildActionGuide('切换视觉模型', {
    overviewZh: '它会把视觉模块切换到另一套模型或配置，让后面的识别逻辑按新规则工作。',
    whenToUseZh: '当同一场流程里前后需要识别不同目标，或者同一视觉模块要切到另一组参数时，用它做显式切换。',
    successZh: '目标模型切换完成并准备好工作时返回成功。',
    failureZh: '模型 ID 不存在、切换失败或超时时返回失败。',
    pitfallsZh: '切模型后，最好明确知道后面节点期待的目标类型是否已经同步变化。不要切了模型却还沿用旧的目标属性条件。',
    exampleZh: '例如先识别一种目标，完成后再切到另一套模型继续识别下一类目标。',
  }),
  WaitVisionTarget: buildActionGuide('等待视觉目标', {
    overviewZh: '它会持续等待，直到视觉模块看到符合条件的目标为止。',
    whenToUseZh: '当你不是立刻拿当前识别结果，而是想“等到出现某种目标再继续”时，用这个节点表达最自然。',
    successZh: '在超时前看到了符合属性和距离限制的目标时返回成功。',
    failureZh: '等到超时仍没看到目标，或视觉状态异常时返回失败。',
    pitfallsZh: '它依赖视觉已经在工作。若前面没启动视觉或模型切错，WaitVisionTarget 看起来像“目标一直不来”，其实可能是根本没人识别。',
    exampleZh: '例如先启动视觉，再等待某类目标出现；一旦识别到了，后面流程就开始抓取或瞄准。',
  }),
};

export function buildKnowledgeBaseGuideZh(entry: BtNodeRegistryEntry): BtNodeGuideZh {
  if (entry.tagName.startsWith('Switch')) {
    return buildControlGuide(`多路切换节点（${entry.tagName.replace('Switch', '')} 路）`, {
      overviewZh: '它会先读一个判定值，再把流程切到和这个值匹配的那一路分支。',
      whenToUseZh: '当你已经有一个明确变量值，并且希望根据不同取值走到不同子流程时，用 Switch 系列最直观。',
      placementZh:
        '把它放在“根据变量值分路”的位置。它前面通常要先有节点把 variable 写好，下面再按 case_1、case_2 等条件值挂对应分支，最后可选加一条默认分支。',
      successZh: '匹配到的那条分支成功时，它返回成功。',
      failureZh: '没有任何 case 匹配且没有默认分支，或匹配到的分支失败时，它返回失败。',
      pitfallsZh: '最常见问题是 variable 的真实值和 case_ 条件值写得不一致，比如大小写、空格或枚举名不同，结果所有分支都匹配不上。',
      exampleZh: '例如上游先把 `{next_action}` 写成 `GRAB`、`MOVE`、`EXIT` 之一，再用 Switch 节点把不同动作分到不同子流程。',
    });
  }

  return (
    nodeGuideByTag[entry.tagName] ??
    buildActionGuide(entry.labelZh, {
      overviewZh: `${entry.labelZh} 是一个需要结合当前项目语义理解的节点。它的短摘要是：${entry.descriptionZh}`,
      whenToUseZh: '当你已经确认它正好对应当前这一步业务动作或判断时再使用；第一次接触时，优先参考已有 XML 里它前后通常和哪些节点连在一起。',
      pitfallsZh: '如果你只看名字还不确定它到底做什么，不要直接放进主流程里。先在知识库里看参数、前后节点和现有 XML 使用位置，再决定要不要用。',
      exampleZh: '最稳妥的学习方式是先在现有树里找到一个已经使用过它的例子，照着那个上下文理解它的职责。',
    })
  );
}

export function buildDynamicSubtreeGuideZh(treeLabel: string): BtNodeGuideZh {
  return buildSubtreeGuide({
    overviewZh: `这个节点会跳到“${treeLabel}”那棵树继续执行，相当于把一大段流程折叠成了一个可复用步骤。`,
    whenToUseZh: `当你已经在当前文档里定义好了“${treeLabel}”这棵子树，并且想在当前位置复用它时，就应该使用这个知识项对应的 SubTree 调用。`,
    successZh: `“${treeLabel}” 这棵树整体成功结束时，这个子树调用也返回成功。`,
    failureZh: `“${treeLabel}” 这棵树失败、子树 ID 填错，或端口映射没对上时，这个调用会失败。`,
    pitfallsZh: '动态子树最常见问题不是逻辑错，而是“名字没对上”或“黑板变量没接上”。只要改过子树 ID、端口名或黑板键，就要把调用处一起检查一遍。',
    exampleZh: `例如当前主流程只想在这里“调用 ${treeLabel} 去完成一整段子任务”，那就可以放一个对应的 SubTree 节点，而不是把那棵树里的所有节点全展开写在这里。`,
  });
}

export function buildSubtreePortDocOverrides(treeId: string): Record<string, BtPortDocOverride> {
  return {
    'SubTree.ID': {
      beginnerHintZh: `这里直接填当前文档里那棵子树的 ID。对于这个知识项来说，最稳妥的写法就是 ${treeId}，不要改成中文名。`,
      exampleValueZh: treeId,
    },
  };
}
