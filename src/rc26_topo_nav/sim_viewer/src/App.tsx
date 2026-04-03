import { startTransition, useEffect, useMemo, useRef, useState } from 'react';
import clsx from 'clsx';

import { createRun, controlRun, fetchSceneManifest, mapGoalPayload, openLiveSocket, openRunSocket, startLive } from './api';
import { SceneCanvas } from './components/SceneCanvas';
import { useSimStore } from './store';
import { 
  UI_LABELS, 
  TEAM_LABELS, 
  GOAL_KIND_LABELS, 
  RUN_MODE_LABELS, 
  ALGORITHM_LABELS,
  VIEW_MODE_LABELS,
  LAYER_LABELS,
  NODE_TYPE_LABELS,
  withRawLabel 
} from './labels';
import type { GoalKind, GraphNode, PickMode, RunFrameMessage, RunMetaMessage, Team, ViewMode } from './types';

function nodeById(nodes: GraphNode[], nodeId: string): GraphNode | undefined {
  return nodes.find((node) => node.id === nodeId);
}

function parseBlockedNodes(value: string): string[] {
  return value
    .split(',')
    .map((item) => item.trim())
    .filter(Boolean);
}

function nodeDisplayLabel(node: GraphNode | null | undefined): string {
  if (!node) {
    return 'N/A';
  }
  return `${node.id} · ${NODE_TYPE_LABELS[node.type] || node.type}`;
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
      label: `${node.id} · ${NODE_TYPE_LABELS[node.type] || node.type}` 
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

  useEffect(() => {
    let active = true;
    setSceneLoading(true);
    fetchSceneManifest(team)
      .then((manifest) => {
        if (!active) return;
        startTransition(() => {
          setScene(manifest);
        });
      })
      .catch((error) => {
        if (!active) return;
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

  const manualRunHint = mode === 'offline-sim' ? UI_LABELS.hintManualRunOnly : UI_LABELS.hintLiveReadonly;
  const pickHint =
    mode !== 'offline-sim'
      ? UI_LABELS.hintLiveReadonly
      : pickMode === 'start'
        ? UI_LABELS.hintPickStart
        : pickMode === 'goal'
          ? UI_LABELS.hintPickGoal
          : UI_LABELS.hintPickIdle;
  const frameTitle =
    currentFrame?.label ??
    (runId ? UI_LABELS.hintFrameRunReady : UI_LABELS.hintFrameIdle);
  const frameMetrics = Object.entries(currentFrame?.metrics ?? {});
  const summaryEntries = Object.entries(runSummary ?? {});

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
    if (!scene) return;
    runSocketRef.current?.close();
    setPickMode('idle');
    setHoveredNodeId(null);
    resetRun();
    const blockedNodes = parseBlockedNodes(blockedNodeText);
    try {
      const response = await createRun({
        algorithm,
        mode: 'offline-sim',
        team,
        start_node: startNode,
        strict_runtime: strictRuntime,
        animation_speed: animationSpeed,
        blocked_nodes: blockedNodes,
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
    if (!runId) return;
    try {
      await controlRun(runId, action, undefined, animationSpeed);
    } catch (error) {
      setStatusMessage(`运行控制失败: ${String(error)}`);
    }
  }

  async function handleLiveStart() {
    liveSocketRef.current?.close();
    setPickMode('idle');
    setHoveredNodeId(null);
    try {
      await startLive();
      liveSocketRef.current = openLiveSocket({
        onEvent: (event) => setLiveEvent(event),
        onError: (message: string) => setStatusMessage(message),
      });
      setStatusMessage('实时 ROS 只读桥接已启动');
    } catch (error) {
      setStatusMessage(`实时桥接启动失败: ${String(error)}`);
    }
  }

  const layerKeys = Object.keys(layers) as Array<keyof typeof layers>;
  const viewModes: ViewMode[] = ['orbit', 'follow', 'first_person', 'top_ortho', 'side_perspective'];

  const hasError = statusMessage.includes('失败') || statusMessage.includes('error');
  const isRunning = runState === 'playing';

  return (
    <div className="app-shell">
      <header className="topbar">
        <div>
          <p className="eyebrow">RC26 TOPO NAV</p>
          <h1 className="title">{UI_LABELS.appTitle}</h1>
          <p className="subtitle">{UI_LABELS.appSubtitle}</p>
        </div>
        <div className="status-pill">
          <span className={clsx("status-dot", hasError && "error", isRunning && "warning")} />
          {statusMessage}
        </div>
      </header>

      <main className="workspace-grid">
        <aside className="panel panel-left">
          <section className="panel-section">
            <h2>{UI_LABELS.panelConfig}</h2>
            <label className="field-label" htmlFor="team-select">{UI_LABELS.fieldTeam}</label>
            <select id="team-select" className="field-input" value={team} onChange={(event) => setTeam(event.target.value as Team)}>
              {Object.entries(TEAM_LABELS).map(([key, label]) => (
                <option key={key} value={key}>{label}</option>
              ))}
            </select>

            <label className="field-label" htmlFor="mode-select">{UI_LABELS.fieldMode}</label>
            <select id="mode-select" className="field-input" value={mode} onChange={(event) => setMode(event.target.value as typeof mode)}>
              {Object.entries(RUN_MODE_LABELS).map(([key, label]) => (
                <option key={key} value={key}>{label}</option>
              ))}
            </select>

            <label className="field-label" htmlFor="algorithm-select">{UI_LABELS.fieldAlgo}</label>
            <select id="algorithm-select" className="field-input" value={algorithm} onChange={(event) => setAlgorithm(event.target.value as typeof algorithm)}>
              {Object.entries(ALGORITHM_LABELS).map(([key, label]) => (
                <option key={key} value={key}>{label}</option>
              ))}
            </select>

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

            <div className="button-row" style={{ marginTop: 16 }}>
              <button className="primary-button" type="button" onClick={handleCreateRun} disabled={!scene || loadingScene || mode !== 'offline-sim'} style={{ flex: 1 }}>
                {UI_LABELS.btnGenerateRun}
              </button>
              <button className="secondary-button" type="button" onClick={handleLiveStart} disabled={mode !== 'live-ros'} style={{ flex: 1 }}>
                {UI_LABELS.btnStartLive}
              </button>
            </div>
            <p className="inline-hint">{manualRunHint}</p>
          </section>

          <section className="panel-section">
            <h2>{UI_LABELS.panelPick}</h2>
            <div className="trace-card pick-card">
              <div className="metric-row">
                <span>{UI_LABELS.fieldStart}</span>
                <strong>{nodeDisplayLabel(startNodeData)}</strong>
              </div>
              <div className="metric-row">
                <span>{UI_LABELS.fieldGoalValue}</span>
                <strong>{goalKind === 'node' ? nodeDisplayLabel(goalNodeData) : withRawLabel(GOAL_KIND_LABELS[goalKind], goalValue || 'N/A')}</strong>
              </div>
              <div className="metric-row">
                <span>{UI_LABELS.fieldHoverNode}</span>
                <strong>{hoveredNodeData ? nodeDisplayLabel(hoveredNodeData) : UI_LABELS.hintPickPreviewEmpty}</strong>
              </div>
            </div>
            <p className="inline-hint">{pickHint}</p>
            <div className="button-grid pick-button-grid">
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
          </section>

          <section className="panel-section">
            <h2>{UI_LABELS.panelView}</h2>
            <div className="button-grid">
              {viewModes.map((id) => (
                <button
                  key={id}
                  className={clsx('mode-button', viewMode === id && 'mode-button-active')}
                  type="button"
                  onClick={() => setViewMode(id as ViewMode)}
                >
                  {VIEW_MODE_LABELS[id]}
                </button>
              ))}
            </div>
          </section>

          <section className="panel-section">
            <h2>{UI_LABELS.panelLayers}</h2>
            <div className="layer-grid">
              {layerKeys.map((layer) => (
                <label key={layer} className="toggle-row">
                  <input type="checkbox" checked={layers[layer]} onChange={() => toggleLayer(layer)} />
                  <span>{LAYER_LABELS[layer] || layer}</span>
                </label>
              ))}
            </div>
          </section>
        </aside>

        <section className="panel panel-canvas">
          <div className="canvas-toolbar">
            <div className="toolbar-stack">
              <div className="legend">
                <span className="legend-chip legend-start">{UI_LABELS.legendStart}</span>
                <span className="legend-chip legend-goal">{UI_LABELS.legendGoal}</span>
                <span className="legend-chip legend-path">{UI_LABELS.legendPath}</span>
                <span className="legend-chip legend-open">{UI_LABELS.legendOpen}</span>
                <span className="legend-chip legend-tree">{UI_LABELS.legendTree}</span>
              </div>
              <div className="toolbar-note">{manualRunHint}</div>
            </div>
            <div className="button-row">
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
              pickMode={mode === 'offline-sim' ? pickMode : 'idle'}
              onHoverNodeChange={handleHoverNodeChange}
              onPickNode={handleScenePick}
            />
            <div className="canvas-overlay">
              <div className="canvas-overlay-card">
                {runId ? UI_LABELS.hintFrameRunReady : UI_LABELS.hintRunIdle}
              </div>
              <div className={clsx('canvas-overlay-card', 'canvas-overlay-emphasis', pickMode !== 'idle' && 'canvas-overlay-active')}>
                {hoveredNodeData && pickMode !== 'idle'
                  ? `${UI_LABELS.fieldHoverNode}: ${nodeDisplayLabel(hoveredNodeData)}`
                  : pickHint}
              </div>
            </div>
          </div>
        </section>

        <aside className="panel panel-right">
          <section className="panel-section" style={{ marginTop: 0, paddingTop: 0, borderTop: 'none' }}>
            <h2>{UI_LABELS.panelState}</h2>
            <div className="stats-grid">
              <div className="stat-card">
                <span className="stat-label">{UI_LABELS.statRunId}</span>
                <span className="stat-value">{runId ? runId.slice(0, 8) : 'N/A'}</span>
              </div>
              <div className="stat-card">
                <span className="stat-label">{UI_LABELS.statCursor}</span>
                <span className="stat-value">
                  {cursor} / {Math.max(frameCount - 1, 0)}
                </span>
              </div>
              <div className="stat-card">
                <span className="stat-label">{UI_LABELS.statState}</span>
                <span className="stat-value">
                  {runId
                    ? runState === 'playing'
                      ? '播放中'
                      : runState === 'paused'
                        ? '已暂停'
                        : runState === 'finished'
                          ? '已结束'
                          : runState
                    : '未创建'}
                </span>
              </div>
              <div className="stat-card">
                <span className="stat-label">{UI_LABELS.statFaces}</span>
                <span className="stat-value">{scene?.sceneFeatures.length ?? 0}</span>
              </div>
            </div>
          </section>

          <section className="panel-section">
            <h2>{UI_LABELS.panelFrame}</h2>
            <div className="trace-card">
              <div className="trace-title">{frameTitle}</div>
              <div className="trace-meta">
                <span>{currentFrame?.algorithm ? ALGORITHM_LABELS[currentFrame.algorithm] || currentFrame.algorithm : ALGORITHM_LABELS[algorithm] || algorithm}</span>
                <span>{currentFrame?.phase ?? (runId ? '等待播放 (queued)' : '空闲 (idle)')}</span>
              </div>
              <div className="metrics-list">
                {frameMetrics.length === 0 && <div className="empty-note">{runId ? UI_LABELS.hintFrameRunReady : UI_LABELS.hintRunIdle}</div>}
                {frameMetrics.map(([key, value]) => (
                  <div key={key} className="metric-row">
                    <span>{key}</span>
                    <strong>{String(value)}</strong>
                  </div>
                ))}
              </div>
            </div>
          </section>

          <section className="panel-section">
            <h2>{UI_LABELS.panelSummary}</h2>
            <div className="metrics-list">
              {summaryEntries.length === 0 && <div className="empty-note">{UI_LABELS.hintSummaryEmpty}</div>}
              {summaryEntries.map(([key, value]) => (
                <div key={key} className="metric-row">
                  <span>{key}</span>
                  <strong>{typeof value === 'object' ? JSON.stringify(value) : String(value)}</strong>
                </div>
              ))}
            </div>
          </section>

          <section className="panel-section">
            <h2>{UI_LABELS.panelLive}</h2>
            <div className="metrics-list">
              <div className="metric-row">
                <span>{UI_LABELS.liveActiveEdge}</span>
                <strong>{liveEvent?.activeEdge ?? 'N/A'}</strong>
              </div>
              <div className="metric-row">
                <span>{UI_LABELS.liveGate}</span>
                <strong>{liveEvent?.gateStatus ?? 'N/A'}</strong>
              </div>
              <div className="metric-row">
                <span>{UI_LABELS.liveCorridor}</span>
                <strong>{liveEvent?.trackingState?.corridorId ?? 'N/A'}</strong>
              </div>
              <div className="metric-row">
                <span>{UI_LABELS.liveDistance}</span>
                <strong>{liveEvent?.trackingState?.distanceToGoal?.toFixed?.(3) ?? 'N/A'}</strong>
              </div>
            </div>
          </section>
        </aside>
      </main>
    </div>
  );
}
