import { startTransition, useEffect, useMemo, useRef, useState } from 'react';
import clsx from 'clsx';

import { createRun, controlRun, fetchSceneManifest, mapGoalPayload, openLiveSocket, openRunSocket, startLive } from './api';
import { SceneCanvas } from './components/SceneCanvas';
import { DEBUG_KEY_LABELS, ALGORITHM_LABELS, GOAL_KIND_LABELS, NODE_TYPE_LABELS, RUN_MODE_LABELS, TEAM_LABELS, TEAM_SHORT_LABELS, UI_LABELS, VIEW_MODE_LABELS, VIEW_MODE_SHORT_LABELS, withRawLabel } from './labels';
import { deriveBlockedGridIds, deriveLayerControls, parseBlockedNodes } from './layerModel';
import { useSimStore } from './store';
import type { GoalKind, GraphNode, PickMode, RunFrameMessage, RunMetaMessage, Team, ViewMode } from './types';

function nodeById(nodes: GraphNode[], nodeId: string): GraphNode | undefined {
  return nodes.find((node) => node.id === nodeId);
}

function hasLiveSnapshotContent(event: { activeEdge?: string; gateStatus?: string; trackingState?: { corridorId?: string } | null } | null | undefined): boolean {
  return Boolean(
    event?.activeEdge ||
    event?.gateStatus ||
    event?.trackingState?.corridorId,
  );
}

function nodeDisplayLabel(node: GraphNode | null | undefined): string {
  if (!node) {
    return 'N/A';
  }
  return `${node.id} · ${NODE_TYPE_LABELS[node.type] || node.type}`;
}

function formatDebugValue(value: unknown): string {
  if (typeof value === 'number') {
    return Number.isInteger(value) ? String(value) : value.toFixed(3);
  }
  if (typeof value === 'boolean') {
    return value ? 'true' : 'false';
  }
  if (value == null) {
    return 'N/A';
  }
  if (typeof value === 'object') {
    return JSON.stringify(value);
  }
  return String(value);
}

function formatDebugKey(key: string): string {
  return DEBUG_KEY_LABELS[key] ?? key;
}

function formatRunState(runId: string | null, runState: string): string {
  if (!runId) {
    return '未创建';
  }
  if (runState === 'playing') {
    return '播放中';
  }
  if (runState === 'paused') {
    return '已暂停';
  }
  if (runState === 'finished') {
    return '已结束';
  }
  return runState;
}

function GoalSelector({
  goalKind,
  goalValue,
  setGoalKind,
  setGoalValue,
}: {
  goalKind: GoalKind;
  goalValue: string;
  setGoalKind: (value: GoalKind) => void;
  setGoalValue: (value: string) => void;
}) {
  const scene = useSimStore((state) => state.scene);

  const goalOptions = useMemo(() => {
    if (!scene) {
      return [] as Array<{ value: string; label: string }>;
    }
    if (goalKind === 'task') {
      return scene.tasks.map((task) => ({ value: task.task_tag, label: task.task_tag }));
    }
    if (goalKind === 'route') {
      return scene.routes.map((route) => ({ value: route.route_tag, label: route.route_tag }));
    }
    return scene.graphNodes.map((node) => ({
      value: node.id,
      label: `${node.id} · ${NODE_TYPE_LABELS[node.type] || node.type}`,
    }));
  }, [goalKind, scene]);

  useEffect(() => {
    if (!scene) {
      return;
    }
    if (goalKind === 'node' && !goalOptions.some((option) => option.value === goalValue)) {
      setGoalValue(scene.defaults.goalNode);
      return;
    }
    if (goalKind === 'task' && goalOptions.length > 0 && !goalOptions.some((option) => option.value === goalValue)) {
      setGoalValue(goalOptions[0].value);
      return;
    }
    if (goalKind === 'route' && goalOptions.length > 0 && !goalOptions.some((option) => option.value === goalValue)) {
      setGoalValue(goalOptions[0].value);
    }
  }, [goalKind, goalOptions, goalValue, scene, setGoalValue]);

  return (
    <>
      <label className="field-label" htmlFor="goal-kind">
        {UI_LABELS.fieldGoalKind}
      </label>
      <select id="goal-kind" className="field-input" value={goalKind} onChange={(event) => setGoalKind(event.target.value as GoalKind)}>
        {Object.entries(GOAL_KIND_LABELS).map(([key, label]) => (
          <option key={key} value={key}>{label}</option>
        ))}
      </select>

      <label className="field-label" htmlFor="goal-value">
        {UI_LABELS.fieldGoalValue}
      </label>
      <select id="goal-value" className="field-input" value={goalValue} onChange={(event) => setGoalValue(event.target.value)}>
        {goalOptions.map((option) => (
          <option key={option.value} value={option.value}>
            {option.label}
          </option>
        ))}
      </select>
    </>
  );
}

export default function App() {
  const scene = useSimStore((state) => state.scene);
  const loadingScene = useSimStore((state) => state.loadingScene);
  const team = useSimStore((state) => state.team);
  const algorithm = useSimStore((state) => state.algorithm);
  const mode = useSimStore((state) => state.mode);
  const goalKind = useSimStore((state) => state.goalKind);
  const strictRuntime = useSimStore((state) => state.strictRuntime);
  const animationSpeed = useSimStore((state) => state.animationSpeed);
  const startNode = useSimStore((state) => state.startNode);
  const goalValue = useSimStore((state) => state.goalValue);
  const blockedNodeText = useSimStore((state) => state.blockedNodeText);
  const viewMode = useSimStore((state) => state.viewMode);
  const layers = useSimStore((state) => state.layers);
  const runId = useSimStore((state) => state.runId);
  const runState = useSimStore((state) => state.runState);
  const cursor = useSimStore((state) => state.cursor);
  const frameCount = useSimStore((state) => state.frameCount);
  const currentFrame = useSimStore((state) => state.currentFrame);
  const runSummary = useSimStore((state) => state.runSummary);
  const liveEvent = useSimStore((state) => state.liveEvent);
  const statusMessage = useSimStore((state) => state.statusMessage);

  const setTeam = useSimStore((state) => state.setTeam);
  const setAlgorithm = useSimStore((state) => state.setAlgorithm);
  const setMode = useSimStore((state) => state.setMode);
  const setGoalKind = useSimStore((state) => state.setGoalKind);
  const setStrictRuntime = useSimStore((state) => state.setStrictRuntime);
  const setAnimationSpeed = useSimStore((state) => state.setAnimationSpeed);
  const setSceneLoading = useSimStore((state) => state.setSceneLoading);
  const setScene = useSimStore((state) => state.setScene);
  const setStartNode = useSimStore((state) => state.setStartNode);
  const setGoalValue = useSimStore((state) => state.setGoalValue);
  const setBlockedNodeText = useSimStore((state) => state.setBlockedNodeText);
  const setViewMode = useSimStore((state) => state.setViewMode);
  const toggleLayer = useSimStore((state) => state.toggleLayer);
  const setRunMeta = useSimStore((state) => state.setRunMeta);
  const setRunFrame = useSimStore((state) => state.setRunFrame);
  const setLiveEvent = useSimStore((state) => state.setLiveEvent);
  const setStatusMessage = useSimStore((state) => state.setStatusMessage);
  const resetRun = useSimStore((state) => state.resetRun);

  const runSocketRef = useRef<WebSocket | null>(null);
  const liveSocketRef = useRef<WebSocket | null>(null);
  const [pickMode, setPickMode] = useState<PickMode>('idle');
  const [hoveredNodeId, setHoveredNodeId] = useState<string | null>(null);
  const [debugOpen, setDebugOpen] = useState(false);

  useEffect(() => {
    let active = true;
    setSceneLoading(true);
    fetchSceneManifest(team)
      .then((manifest) => {
        if (!active) {
          return;
        }
        startTransition(() => {
          setScene(manifest);
        });
      })
      .catch((error) => {
        if (!active) {
          return;
        }
        setSceneLoading(false);
        setStatusMessage(`${UI_LABELS.statusError}: ${String(error)}`);
      });
    return () => {
      active = false;
    };
  }, [setScene, setSceneLoading, setStatusMessage, team]);

  useEffect(() => {
    return () => {
      runSocketRef.current?.close();
      liveSocketRef.current?.close();
    };
  }, []);

  useEffect(() => {
    setPickMode('idle');
    setHoveredNodeId(null);
  }, [team, mode]);

  useEffect(() => {
    if (pickMode === 'idle') {
      setHoveredNodeId(null);
    }
  }, [pickMode]);

  const blockedNodeIds = useMemo(() => parseBlockedNodes(blockedNodeText), [blockedNodeText]);
  const blockedGridIds = useMemo(
    () => deriveBlockedGridIds({ scene, mode, liveEvent, blockedNodeIds }),
    [scene, mode, liveEvent, blockedNodeIds],
  );
  const layerControls = useMemo(
    () => deriveLayerControls({ layers, mode, algorithm, frame: currentFrame, blockedGridIds }),
    [layers, mode, algorithm, currentFrame, blockedGridIds],
  );

  const startNodeData = scene ? nodeById(scene.graphNodes, startNode) ?? null : null;
  const goalNodeData = scene && goalKind === 'node' ? nodeById(scene.graphNodes, goalValue) ?? null : null;
  const hoveredNodeData = scene && hoveredNodeId ? nodeById(scene.graphNodes, hoveredNodeId) ?? null : null;
  const startPose = startNodeData?.pose ?? null;
  const goalPose =
    scene && goalKind === 'node'
      ? goalNodeData?.pose ?? null
      : (currentFrame && currentFrame.bestPath.points.length > 0
          ? currentFrame.bestPath.points[currentFrame.bestPath.points.length - 1]
          : null);
  const hoverPose = hoveredNodeData?.pose ?? null;

  const effectiveAlgorithm = currentFrame?.algorithm ?? algorithm;
  const primaryLayerControls = layerControls.filter((control) => control.group === 'primary' && control.visible);
  const appearanceLayerControls = layerControls.filter((control) => control.group === 'advanced');
  const manualRunHint = mode === 'offline-sim' ? UI_LABELS.hintManualRunOnly : UI_LABELS.hintLiveReadonly;
  const pickHint =
    mode !== 'offline-sim'
      ? UI_LABELS.hintLiveReadonly
      : pickMode === 'start'
        ? UI_LABELS.hintPickStart
        : pickMode === 'goal'
          ? UI_LABELS.hintPickGoal
          : UI_LABELS.hintPickIdle;
  const frameMetrics = Object.entries(currentFrame?.metrics ?? {});
  const summaryEntries = Object.entries(runSummary ?? {});
  const liveEntries = [
    { key: 'activeEdge', value: liveEvent?.activeEdge ?? 'N/A' },
    { key: 'gateStatus', value: liveEvent?.gateStatus ?? 'N/A' },
    { key: 'corridorId', value: liveEvent?.trackingState?.corridorId ?? 'N/A' },
    { key: 'distanceToGoal', value: liveEvent?.trackingState?.distanceToGoal ?? null },
  ];
  const runStateText = formatRunState(runId, runState);
  const progressText = runId ? `${cursor} / ${Math.max(frameCount - 1, 0)}` : (mode === 'live-ros' ? 'LIVE' : '待运行');
  const blockedSummary = blockedGridIds.length > 0
    ? `${blockedGridIds.length} 个阻塞区`
    : mode === 'live-ros'
      ? '等待实时阻塞区'
      : blockedNodeIds.length > 0
        ? '当前配置未映射到可视阻塞区'
        : '未配置';
  const liveSummary = liveEvent
    ? [liveEvent.activeEdge, liveEvent.gateStatus].filter(Boolean).join(' · ') || '实时桥接已启动'
    : '尚未启动';
  const frameTitle = currentFrame?.label ?? (runId ? UI_LABELS.hintFrameRunReady : UI_LABELS.hintFrameIdle);
  const viewModes: ViewMode[] = ['orbit', 'follow', 'first_person', 'top_ortho', 'side_perspective'];
  const hasError = statusMessage.includes('失败') || statusMessage.includes('error');
  const isRunning = runState === 'playing';

  function restoreManualStatus() {
    setStatusMessage(runId ? UI_LABELS.hintFrameRunReady : UI_LABELS.statusLoaded);
  }

  function handlePickModeChange(nextMode: Exclude<PickMode, 'idle'>) {
    if (!scene || mode !== 'offline-sim') {
      return;
    }
    if (nextMode === 'goal' && goalKind !== 'node') {
      setGoalKind('node');
    }
    setPickMode(nextMode);
    setHoveredNodeId(null);
    setStatusMessage(nextMode === 'start' ? UI_LABELS.hintPickStart : UI_LABELS.hintPickGoal);
  }

  function handleCancelPick() {
    setPickMode('idle');
    restoreManualStatus();
  }

  function handleHoverNodeChange(nodeId: string | null) {
    setHoveredNodeId(nodeId);
  }

  function handleScenePick(nodeId: string) {
    if (!scene || pickMode === 'idle') {
      return;
    }

    const pickedNode = nodeById(scene.graphNodes, nodeId);
    if (!pickedNode) {
      return;
    }

    if (pickMode === 'start') {
      setStartNode(nodeId);
      setStatusMessage(`起点已吸附到 ${nodeDisplayLabel(pickedNode)}，可直接生成离线运行`);
    } else {
      if (goalKind !== 'node') {
        setGoalKind('node');
      }
      setGoalValue(nodeId);
      setStatusMessage(`目标已吸附到 ${nodeDisplayLabel(pickedNode)}，可直接生成离线运行`);
    }

    setPickMode('idle');
  }

  async function handleCreateRun() {
    if (!scene) {
      return;
    }
    runSocketRef.current?.close();
    setPickMode('idle');
    setHoveredNodeId(null);
    resetRun();
    try {
      const response = await createRun({
        algorithm,
        mode: 'offline-sim',
        team,
        start_node: startNode,
        strict_runtime: strictRuntime,
        animation_speed: animationSpeed,
        blocked_nodes: blockedNodeIds,
        blocked_edges: [],
        ...mapGoalPayload(goalKind, goalValue),
      });
      setRunMeta(response.runId, {
        state: response.state,
        cursor: 0,
        frameCount: response.frameCount,
        summary: response.summary,
      });
      const socket = openRunSocket(response.runId, {
        onMeta: (message: RunMetaMessage) => {
          setRunMeta(response.runId, message);
        },
        onFrame: (message: RunFrameMessage) => {
          setRunFrame({
            state: message.state,
            cursor: message.cursor,
            frameCount: message.frameCount,
            summary: message.summary,
            frame: message.frame,
          });
        },
        onError: (message: string) => setStatusMessage(message),
      });
      runSocketRef.current = socket;
    } catch (error) {
      setStatusMessage(`创建运行失败: ${String(error)}`);
    }
  }

  async function handleRunControl(action: 'play' | 'pause' | 'step' | 'reset') {
    if (!runId) {
      return;
    }
    try {
      const response = await controlRun(runId, action, undefined, animationSpeed);
      setRunFrame({
        state: response.state,
        cursor: response.cursor,
      });
    } catch (error) {
      setStatusMessage(`运行控制失败: ${String(error)}`);
    }
  }

  async function handleLiveStart() {
    liveSocketRef.current?.close();
    setPickMode('idle');
    setHoveredNodeId(null);
    try {
      const response = await startLive();
      if (hasLiveSnapshotContent(response.snapshot)) {
        setLiveEvent(response.snapshot!);
      } else {
        setStatusMessage('实时 ROS 只读桥接已启动');
      }
      liveSocketRef.current = openLiveSocket({
        onEvent: (event) => setLiveEvent(event),
        onError: (message: string) => setStatusMessage(message),
      });
    } catch (error) {
      setStatusMessage(`实时桥接启动失败: ${String(error)}`);
    }
  }

  return (
    <div className="app-shell">
      <header className="topbar">
        <div>
          <p className="eyebrow">RC26 TOPO NAV</p>
          <h1 className="title">{UI_LABELS.appTitle}</h1>
          <p className="subtitle">{UI_LABELS.appSubtitle}</p>
        </div>
        <div className="topbar-actions">
          <button
            className={clsx('secondary-button', 'debug-toggle', debugOpen && 'debug-toggle-active')}
            type="button"
            aria-expanded={debugOpen}
            aria-controls="debug-panel"
            onClick={() => setDebugOpen((value) => !value)}
          >
            {UI_LABELS.btnDebugPanel}
          </button>
          <div className="status-pill">
            <span className={clsx('status-dot', hasError && 'error', isRunning && 'warning')} />
            {statusMessage}
          </div>
        </div>
      </header>

      <main className="viewer-layout">
        <section className="panel panel-canvas panel-canvas-shell">
          <div className="command-strip">
            <div className="command-group" aria-label={UI_LABELS.fieldTeam}>
              {(['blue', 'red'] as Team[]).map((id) => (
                <button
                  key={id}
                  type="button"
                  className={clsx('mode-button', team === id && 'mode-button-active')}
                  onClick={() => setTeam(id)}
                  disabled={loadingScene}
                  aria-label={TEAM_LABELS[id]}
                  title={TEAM_LABELS[id]}
                >
                  {TEAM_SHORT_LABELS[id]}
                </button>
              ))}
            </div>
            <div className="command-group" aria-label={UI_LABELS.fieldMode}>
              {(['offline-sim', 'live-ros'] as const).map((id) => (
                <button
                  key={id}
                  type="button"
                  className={clsx('mode-button', mode === id && 'mode-button-active')}
                  onClick={() => setMode(id)}
                >
                  {RUN_MODE_LABELS[id]}
                </button>
              ))}
            </div>
            {mode === 'offline-sim' && (
              <div className="command-group" aria-label={UI_LABELS.fieldAlgo}>
                {(['astar', 'rrt', 'dwa'] as const).map((id) => (
                  <button
                    key={id}
                    type="button"
                    className={clsx('mode-button', algorithm === id && 'mode-button-active')}
                    onClick={() => setAlgorithm(id)}
                  >
                    {id === 'astar' ? 'A*' : id.toUpperCase()}
                  </button>
                ))}
              </div>
            )}
            <div className="command-note">{manualRunHint}</div>
          </div>

          <div className="canvas-frame">
            <SceneCanvas
              scene={scene}
              frame={currentFrame}
              liveEvent={liveEvent}
              viewMode={viewMode}
              layers={layers}
              startPose={startPose}
              goalPose={goalPose}
              hoverPose={hoverPose}
              blockedGridIds={blockedGridIds}
              pickMode={mode === 'offline-sim' ? pickMode : 'idle'}
              onHoverNodeChange={handleHoverNodeChange}
              onPickNode={handleScenePick}
            />

            <div className="canvas-overlay">
              <div className="canvas-overlay-card canvas-overlay-card-strong">
                <strong>{mode === 'live-ros' ? RUN_MODE_LABELS[mode] : ALGORITHM_LABELS[effectiveAlgorithm]}</strong>
                <span>{runStateText} · {progressText}</span>
              </div>
              <div className={clsx('canvas-overlay-card', 'canvas-overlay-emphasis', pickMode !== 'idle' && 'canvas-overlay-active')}>
                {hoveredNodeData && pickMode !== 'idle'
                  ? `${UI_LABELS.fieldHoverNode}: ${nodeDisplayLabel(hoveredNodeData)}`
                  : pickHint}
              </div>
              {!debugOpen && (
                <div className="canvas-overlay-card">
                  {UI_LABELS.hintDebugClosed}
                </div>
              )}
            </div>

            <div className="canvas-hud canvas-hud-top-left">
              <div className="hud-stack">
                <div className="hud-card">
                  <span className="hud-section-title">{UI_LABELS.fieldStart}</span>
                  <strong>{nodeDisplayLabel(startNodeData)}</strong>
                </div>
                <div className="hud-card">
                  <span className="hud-section-title">{UI_LABELS.fieldGoalValue}</span>
                  <strong>{goalKind === 'node' ? nodeDisplayLabel(goalNodeData) : withRawLabel(GOAL_KIND_LABELS[goalKind], goalValue || 'N/A')}</strong>
                </div>
                <div className="hud-card">
                  <span className="hud-section-title">{mode === 'live-ros' ? UI_LABELS.panelLive : UI_LABELS.panelState}</span>
                  <strong>{mode === 'live-ros' ? liveSummary : `${runStateText} · ${progressText}`}</strong>
                </div>
              </div>
            </div>

            <div className="canvas-hud canvas-hud-top-right">
              <div className="hud-card hud-card-wide">
                <span className="hud-section-title">{UI_LABELS.panelLayers}</span>
                <div className="layer-dock">
                  {primaryLayerControls.map((control) => (
                    <button
                      key={control.key}
                      type="button"
                      className={clsx(
                        'layer-button',
                        `layer-button-${control.tone}`,
                        control.active && 'layer-button-active',
                      )}
                      aria-pressed={control.active}
                      aria-label={control.label}
                      title={control.reason ? `${control.label} · ${control.reason}` : control.label}
                      onClick={() => toggleLayer(control.key)}
                      disabled={!control.enabled}
                    >
                      <span className={clsx('layer-mark', `layer-mark-${control.key}`)} aria-hidden="true" />
                      <span className="layer-text">{control.shortLabel}</span>
                    </button>
                  ))}
                </div>
              </div>

              <div className="hud-card hud-card-wide">
                <span className="hud-section-title">{UI_LABELS.panelView}</span>
                <div className="micro-button-row">
                  {viewModes.map((id) => (
                    <button
                      key={id}
                      type="button"
                      className={clsx('mode-button', 'micro-mode-button', viewMode === id && 'mode-button-active')}
                      aria-label={VIEW_MODE_LABELS[id]}
                      title={VIEW_MODE_LABELS[id]}
                      onClick={() => setViewMode(id)}
                    >
                      {VIEW_MODE_SHORT_LABELS[id]}
                    </button>
                  ))}
                </div>
              </div>
            </div>

            <div className="canvas-hud canvas-hud-bottom-left">
              <div className="hud-card hud-card-wide">
                <span className="hud-section-title">{UI_LABELS.panelPick}</span>
                <div className="button-grid pick-dock">
                  <button
                    className={clsx('mode-button', pickMode === 'start' && 'mode-button-active')}
                    type="button"
                    onClick={() => handlePickModeChange('start')}
                    disabled={!scene || mode !== 'offline-sim'}
                  >
                    {UI_LABELS.btnPickStart}
                  </button>
                  <button
                    className={clsx('mode-button', pickMode === 'goal' && 'mode-button-active')}
                    type="button"
                    onClick={() => handlePickModeChange('goal')}
                    disabled={!scene || mode !== 'offline-sim'}
                  >
                    {UI_LABELS.btnPickGoal}
                  </button>
                  <button
                    className="secondary-button pick-cancel-button"
                    type="button"
                    onClick={handleCancelPick}
                    disabled={pickMode === 'idle'}
                  >
                    {UI_LABELS.btnCancelPick}
                  </button>
                </div>
              </div>
            </div>

            <div className="canvas-hud canvas-hud-bottom-right">
              <div className="hud-card hud-card-wide">
                <span className="hud-section-title">{mode === 'offline-sim' ? UI_LABELS.panelState : UI_LABELS.panelLive}</span>
                {mode === 'offline-sim' ? (
                  <div className="button-row run-dock">
                    <button
                      className="primary-button"
                      type="button"
                      onClick={handleCreateRun}
                      disabled={!scene || loadingScene}
                    >
                      {UI_LABELS.btnGenerateRun}
                    </button>
                    <button className="secondary-button" type="button" onClick={() => handleRunControl(runState === 'playing' ? 'pause' : 'play')} disabled={!runId}>
                      {runState === 'playing' ? UI_LABELS.btnPause : UI_LABELS.btnPlay}
                    </button>
                    <button className="secondary-button" type="button" onClick={() => handleRunControl('step')} disabled={!runId || runState === 'playing'}>
                      {UI_LABELS.btnStep}
                    </button>
                    <button className="secondary-button" type="button" onClick={() => handleRunControl('reset')} disabled={!runId}>
                      {UI_LABELS.btnReset}
                    </button>
                  </div>
                ) : (
                  <div className="button-row run-dock">
                    <button className="primary-button" type="button" onClick={handleLiveStart}>
                      {UI_LABELS.btnStartLive}
                    </button>
                  </div>
                )}
              </div>
            </div>
          </div>
        </section>

        {debugOpen && (
          <section id="debug-panel" className="panel debug-panel">
            <div className="debug-grid">
              <section className="panel-section">
                <h2>{UI_LABELS.panelFallback}</h2>
                <label className="field-label" htmlFor="start-node">{UI_LABELS.fieldStart}</label>
                <select id="start-node" className="field-input" value={startNode} onChange={(event) => setStartNode(event.target.value)}>
                  {scene?.graphNodes.map((node) => (
                    <option key={node.id} value={node.id}>
                      {node.id} · {NODE_TYPE_LABELS[node.type] || node.type}
                    </option>
                  ))}
                </select>

                <GoalSelector goalKind={goalKind} goalValue={goalValue} setGoalKind={setGoalKind} setGoalValue={setGoalValue} />

                <label className="field-label" htmlFor="blocked-nodes">{UI_LABELS.fieldBlocked}</label>
                <input
                  id="blocked-nodes"
                  className="field-input"
                  value={blockedNodeText}
                  onChange={(event) => setBlockedNodeText(event.target.value)}
                  placeholder="例如 mf_b4,mf_b5"
                />

                <label className="toggle-row">
                  <input type="checkbox" checked={strictRuntime} onChange={(event) => setStrictRuntime(event.target.checked)} />
                  <span>{UI_LABELS.fieldStrict}</span>
                </label>

                <label className="field-label" htmlFor="speed">{UI_LABELS.fieldSpeed}</label>
                <input
                  id="speed"
                  className="range-input"
                  type="range"
                  min={0.5}
                  max={4}
                  step={0.5}
                  value={animationSpeed}
                  onChange={(event) => setAnimationSpeed(Number(event.target.value))}
                />
                <div className="range-readout">{animationSpeed.toFixed(1)}x</div>
              </section>

              <section className="panel-section">
                <h2>{UI_LABELS.panelAppearance}</h2>
                <div className="metrics-list">
                  {appearanceLayerControls.map((control) => (
                    <label key={control.key} className="toggle-row">
                      <input type="checkbox" checked={control.active} onChange={() => toggleLayer(control.key)} />
                      <span>{control.label}</span>
                    </label>
                  ))}
                </div>

                <div className="debug-divider" />

                <h2>{UI_LABELS.panelView}</h2>
                <div className="button-grid">
                  {viewModes.map((id) => (
                    <button
                      key={id}
                      className={clsx('mode-button', viewMode === id && 'mode-button-active')}
                      type="button"
                      onClick={() => setViewMode(id)}
                    >
                      {VIEW_MODE_LABELS[id]}
                    </button>
                  ))}
                </div>

                <div className="debug-divider" />

                <h2>{UI_LABELS.panelState}</h2>
                <div className="stats-grid">
                  <div className="stat-card">
                    <span className="stat-label">{UI_LABELS.statRunId}</span>
                    <span className="stat-value">{runId ? runId.slice(0, 8) : 'N/A'}</span>
                  </div>
                  <div className="stat-card">
                    <span className="stat-label">{UI_LABELS.statCursor}</span>
                    <span className="stat-value">{progressText}</span>
                  </div>
                  <div className="stat-card">
                    <span className="stat-label">{UI_LABELS.statState}</span>
                    <span className="stat-value">{runStateText}</span>
                  </div>
                  <div className="stat-card">
                    <span className="stat-label">{UI_LABELS.fieldBlocked}</span>
                    <span className="stat-value">{blockedSummary}</span>
                  </div>
                </div>
              </section>

              <section className="panel-section">
                <h2>{UI_LABELS.panelFrame}</h2>
                <div className="trace-card">
                  <div className="trace-title">{frameTitle}</div>
                  <div className="trace-meta">
                    <span>{ALGORITHM_LABELS[effectiveAlgorithm] || effectiveAlgorithm}</span>
                    <span>{currentFrame?.phase ?? (runId ? '等待播放' : '空闲')}</span>
                  </div>
                  <div className="metrics-list">
                    {frameMetrics.length === 0 && <div className="empty-note">{runId ? UI_LABELS.hintFrameRunReady : UI_LABELS.hintRunIdle}</div>}
                    {frameMetrics.map(([key, value]) => (
                      <div key={key} className="metric-row">
                        <span>{formatDebugKey(key)}</span>
                        <strong>{formatDebugValue(value)}</strong>
                      </div>
                    ))}
                  </div>
                </div>

                <div className="debug-divider" />

                <h2>{UI_LABELS.panelSummary}</h2>
                <div className="metrics-list">
                  {summaryEntries.length === 0 && <div className="empty-note">{UI_LABELS.hintSummaryEmpty}</div>}
                  {summaryEntries.map(([key, value]) => (
                    <div key={key} className="metric-row">
                      <span>{formatDebugKey(key)}</span>
                      <strong>{formatDebugValue(value)}</strong>
                    </div>
                  ))}
                </div>

                <div className="debug-divider" />

                <h2>{UI_LABELS.panelLive}</h2>
                <div className="metrics-list">
                  {liveEntries.map((entry) => (
                    <div key={entry.key} className="metric-row">
                      <span>{formatDebugKey(entry.key)}</span>
                      <strong>{formatDebugValue(entry.value)}</strong>
                    </div>
                  ))}
                </div>
              </section>
            </div>
          </section>
        )}
      </main>
    </div>
  );
}
