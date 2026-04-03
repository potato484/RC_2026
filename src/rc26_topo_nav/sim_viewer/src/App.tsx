import { startTransition, useEffect, useMemo, useState } from 'react';
import clsx from 'clsx';

import { executeSurfaceRoute, fetchSceneManifest, traceSurfaceRoute } from './api';
import { SceneCanvas } from './components/SceneCanvas';
import {
  formatFrameLabel,
  formatFramePhase,
  MOTION_TYPE_LABELS,
  TEAM_LABELS,
  TEAM_SHORT_LABELS,
  UI_LABELS,
  VIEW_MODE_LABELS,
  VIEW_MODE_SHORT_LABELS,
} from './labels';
import { deriveLayerControls } from './layerModel';
import { useSimStore } from './store';
import type {
  PickMode,
  PlannerFrame,
  PlannerTraceFrame,
  Pose3,
  RouteTraceSummary,
  SceneManifest,
  SurfaceRouteSegment,
  Team,
  ViewMode,
} from './types';

function formatPose(pose: Pose3 | null): string {
  if (!pose) {
    return 'N/A';
  }
  return `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.z.toFixed(2)}`;
}

function formatMetricValue(value: unknown): string {
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

function buildTraceNodePoseMap(
  scene: SceneManifest | null,
  traceNodePoses: Record<string, Pose3>,
): Map<string, Pose3> {
  const nodePoseMap = new Map<string, Pose3>();
  if (!scene) {
    return nodePoseMap;
  }
  scene.graphNodes.forEach((node) => {
    nodePoseMap.set(node.id, node.pose);
  });
  Object.entries(traceNodePoses).forEach(([nodeId, pose]) => {
    nodePoseMap.set(nodeId, pose);
  });
  return nodePoseMap;
}

function hydratePlannerFrame(
  frame: PlannerTraceFrame | null,
  nodePoseMap: Map<string, Pose3>,
): PlannerFrame | null {
  if (!frame) {
    return null;
  }

  const openSet = frame.openSet.flatMap((entry) => {
    const pose = entry.pose ?? nodePoseMap.get(entry.nodeId);
    if (!pose) {
      return [];
    }
    return [{ ...entry, pose }];
  });
  const expandedNodes = frame.expandedNodes.flatMap((entry) => {
    const pose = entry.pose ?? nodePoseMap.get(entry.nodeId);
    if (!pose) {
      return [];
    }
    return [{ ...entry, pose }];
  });
  const bestPathPoints =
    frame.bestPath.points && frame.bestPath.points.length > 0
      ? frame.bestPath.points
      : frame.bestPath.nodeIds.flatMap((nodeId) => {
          const pose = nodePoseMap.get(nodeId);
          return pose ? [pose] : [];
        });

  return {
    ...frame,
    openSet,
    expandedNodes,
    bestPath: {
      ...frame.bestPath,
      points: bestPathPoints,
    },
  };
}

function RouteStats({
  title,
  value,
}: {
  title: string;
  value: string;
}) {
  return (
    <div className="stat-card">
      <span className="stat-label">{title}</span>
      <span className="stat-value">{value}</span>
    </div>
  );
}

export default function App() {
  const scene = useSimStore((state) => state.scene);
  const loadingScene = useSimStore((state) => state.loadingScene);
  const team = useSimStore((state) => state.team);
  const viewMode = useSimStore((state) => state.viewMode);
  const layers = useSimStore((state) => state.layers);
  const statusMessage = useSimStore((state) => state.statusMessage);

  const setTeam = useSimStore((state) => state.setTeam);
  const setSceneLoading = useSimStore((state) => state.setSceneLoading);
  const setScene = useSimStore((state) => state.setScene);
  const setViewMode = useSimStore((state) => state.setViewMode);
  const toggleLayer = useSimStore((state) => state.toggleLayer);
  const setStatusMessage = useSimStore((state) => state.setStatusMessage);

  const [pickMode, setPickMode] = useState<PickMode>('idle');
  const [surfaceStartPick, setSurfaceStartPick] = useState<Pose3 | null>(null);
  const [surfaceGoalPick, setSurfaceGoalPick] = useState<Pose3 | null>(null);
  const [surfaceHoverPose, setSurfaceHoverPose] = useState<Pose3 | null>(null);
  const [surfaceProjectedStart, setSurfaceProjectedStart] = useState<Pose3 | null>(null);
  const [surfaceProjectedGoal, setSurfaceProjectedGoal] = useState<Pose3 | null>(null);
  const [surfacePath, setSurfacePath] = useState<Pose3[]>([]);
  const [surfaceSegments, setSurfaceSegments] = useState<SurfaceRouteSegment[]>([]);
  const [traceFrames, setTraceFrames] = useState<PlannerTraceFrame[]>([]);
  const [traceNodePoses, setTraceNodePoses] = useState<Record<string, Pose3>>({});
  const [traceSummary, setTraceSummary] = useState<RouteTraceSummary | null>(null);
  const [traceIndex, setTraceIndex] = useState(0);
  const [isGenerating, setIsGenerating] = useState(false);
  const [isExecuting, setIsExecuting] = useState(false);

  const traceNodePoseMap = useMemo(
    () => buildTraceNodePoseMap(scene, traceNodePoses),
    [scene, traceNodePoses],
  );
  const currentFrame = useMemo(
    () => hydratePlannerFrame(
      traceFrames.length > 0 ? traceFrames[Math.min(traceIndex, traceFrames.length - 1)] : null,
      traceNodePoseMap,
    ),
    [traceFrames, traceIndex, traceNodePoseMap],
  );

  const layerControls = useMemo(
    () => deriveLayerControls({ layers, frame: currentFrame }),
    [layers, currentFrame],
  );
  const primaryLayerControls = layerControls.filter((control) => control.group === 'primary' && control.visible);
  const appearanceLayerControls = layerControls.filter((control) => control.group === 'advanced' && control.visible);

  const startPose = surfaceProjectedStart ?? surfaceStartPick;
  const goalPose = surfaceProjectedGoal ?? surfaceGoalPick;
  const viewModes: ViewMode[] = ['orbit', 'top_ortho', 'side_perspective'];
  const frameMetrics = Object.entries(currentFrame?.metrics ?? {}).filter(([key]) => key !== 'traceMode');
  const traceTitle = currentFrame ? formatFrameLabel(currentFrame.label, currentFrame.phase) : UI_LABELS.hintTraceEmpty;
  const traceProgressText = traceFrames.length > 0 ? `${traceIndex + 1} / ${traceFrames.length}` : '0 / 0';
  const traceFrameTotal = traceSummary?.framesCount ?? traceFrames.length;
  const traceFrameOverview =
    traceSummary?.framesSampled && traceFrameTotal > traceFrames.length
      ? `${traceFrames.length} / ${traceFrameTotal}`
      : `${traceFrames.length}`;
  const traceProgressDetail =
    traceSummary?.framesSampled && traceFrameTotal > traceFrames.length
      ? `${traceProgressText}（原始 ${traceFrameTotal} 帧）`
      : traceProgressText;
  const hasError = statusMessage.includes('失败') || statusMessage.includes('error');
  const busy = isGenerating || isExecuting;
  const routeReady = surfacePath.length > 0 && traceFrames.length > 0;
  const pickHint =
    pickMode === 'surface_start'
      ? UI_LABELS.hintPickSurfaceStart
      : pickMode === 'surface_goal'
        ? UI_LABELS.hintPickSurfaceGoal
        : UI_LABELS.hintPickIdle;

  function clearGeneratedRoute() {
    setSurfaceProjectedStart(null);
    setSurfaceProjectedGoal(null);
    setSurfacePath([]);
    setSurfaceSegments([]);
    setTraceFrames([]);
    setTraceNodePoses({});
    setTraceSummary(null);
    setTraceIndex(0);
  }

  function clearAllRoute() {
    setPickMode('idle');
    setSurfaceHoverPose(null);
    setSurfaceStartPick(null);
    setSurfaceGoalPick(null);
    clearGeneratedRoute();
    setStatusMessage(UI_LABELS.statusLoaded);
  }

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
    setPickMode('idle');
    setSurfaceHoverPose(null);
    setSurfaceStartPick(null);
    setSurfaceGoalPick(null);
    clearGeneratedRoute();
  }, [team]);

  function handlePickModeChange(nextMode: 'surface_start' | 'surface_goal') {
    if (!scene) {
      return;
    }
    setPickMode(nextMode);
    setSurfaceHoverPose(null);
    setStatusMessage(nextMode === 'surface_start' ? UI_LABELS.hintPickSurfaceStart : UI_LABELS.hintPickSurfaceGoal);
  }

  function handleCancelPick() {
    setPickMode('idle');
    setSurfaceHoverPose(null);
    setStatusMessage(routeReady ? UI_LABELS.hintGenerate : UI_LABELS.hintPickIdle);
  }

  function handleSurfaceScenePick(pose: Pose3) {
    if (pickMode !== 'surface_start' && pickMode !== 'surface_goal') {
      return;
    }

    clearGeneratedRoute();
    if (pickMode === 'surface_start') {
      setSurfaceStartPick(pose);
      setStatusMessage('起点已记录，继续在场景中设置终点');
    } else {
      setSurfaceGoalPick(pose);
      setStatusMessage('终点已记录，可以生成 3D 路线');
    }
    setPickMode('idle');
  }

  async function handleGenerateRoute() {
    if (!surfaceStartPick || !surfaceGoalPick) {
      setStatusMessage('先在场景里设置起点和终点');
      return;
    }

    setIsGenerating(true);
    try {
      const response = await traceSurfaceRoute({
        team,
        start_pick_world: surfaceStartPick,
        goal_pick_world: surfaceGoalPick,
      });
      if (!response.success) {
        clearGeneratedRoute();
        setStatusMessage(`3D 路线生成失败: ${response.failure_reason || response.failure_code}`);
        return;
      }

      setSurfaceProjectedStart(response.projected_start);
      setSurfaceProjectedGoal(response.projected_goal);
      setSurfacePath(response.path_points);
      setSurfaceSegments(response.segments);
      setTraceSummary(response.summary);
      setTraceFrames(response.frames);
      setTraceNodePoses(response.node_poses ?? {});
      setTraceIndex(Math.max(response.frames.length - 1, 0));
      const sampledHint =
        response.summary.framesSampled && (response.summary.framesCount ?? response.frames.length) > response.frames.length
          ? `，回放已压缩为 ${response.frames.length} / ${response.summary.framesCount} 帧`
          : `，${response.frames.length} 帧`;
      setStatusMessage(`3D 路线已生成，共 ${response.segments.length} 段${sampledHint}`);
    } catch (error) {
      clearGeneratedRoute();
      setStatusMessage(`3D 路线生成失败: ${String(error)}`);
    } finally {
      setIsGenerating(false);
    }
  }

  async function handleExecuteRoute() {
    if (!surfaceStartPick || !surfaceGoalPick) {
      setStatusMessage('先在场景里设置起点和终点');
      return;
    }
    if (!routeReady) {
      setStatusMessage('先生成 3D 路线，再决定是否执行');
      return;
    }

    setIsExecuting(true);
    try {
      const response = await executeSurfaceRoute({
        team,
        start_pick_world: surfaceStartPick,
        goal_pick_world: surfaceGoalPick,
      });
      if (response.preview.success) {
        setSurfaceProjectedStart(response.preview.projected_start);
        setSurfaceProjectedGoal(response.preview.projected_goal);
        setSurfacePath(response.preview.path_points);
        setSurfaceSegments(response.preview.segments);
      }
      if (!response.accepted) {
        const reason = response.preview.failure_reason || response.preview.failure_code || 'goal rejected';
        setStatusMessage(`路线未被接受: ${reason}`);
        return;
      }
      setStatusMessage('路线已下发给运行时，机器人需已经在起点附近');
    } catch (error) {
      setStatusMessage(`路线执行失败: ${String(error)}`);
    } finally {
      setIsExecuting(false);
    }
  }

  return (
    <div className="app-shell">
      <header className="topbar">
        <div>
          <p className="eyebrow">{UI_LABELS.appEyebrow}</p>
          <h1 className="title">{UI_LABELS.appTitle}</h1>
          <p className="subtitle">{UI_LABELS.appSubtitle}</p>
        </div>
        <div className="topbar-actions">
          <div className="status-pill">
            <span className={clsx('status-dot', hasError && 'error', busy && 'warning')} />
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
                  disabled={loadingScene || isGenerating || isExecuting}
                  aria-label={TEAM_LABELS[id]}
                  title={TEAM_LABELS[id]}
                >
                  {TEAM_SHORT_LABELS[id]}
                </button>
              ))}
            </div>
            <div className="command-note">
              {UI_LABELS.routeModeHint}
            </div>
          </div>

          <div className="canvas-frame">
            <SceneCanvas
              scene={scene}
              frame={currentFrame}
              liveEvent={null}
              viewMode={viewMode}
              layers={layers}
              startPose={startPose}
              goalPose={goalPose}
              hoverPose={surfaceHoverPose}
              manualPath={[]}
              blockedGridIds={[]}
              pickMode={pickMode}
              onHoverWorldChange={setSurfaceHoverPose}
              onPickWorld={handleSurfaceScenePick}
            />

            <div className="canvas-overlay">
              <div className="canvas-overlay-card canvas-overlay-card-strong">
                <strong>{UI_LABELS.routeTitle}</strong>
                <span>{routeReady ? `已生成 ${traceFrameOverview} 帧搜索回放` : '等待生成路线'}</span>
              </div>
              <div className={clsx('canvas-overlay-card', 'canvas-overlay-emphasis', pickMode !== 'idle' && 'canvas-overlay-active')}>
                {pickMode === 'idle'
                  ? UI_LABELS.hintGenerate
                  : pickHint}
              </div>
            </div>

            <div className="canvas-hud canvas-hud-top-left">
              <div className="hud-stack">
                <div className="hud-card">
                  <span className="hud-section-title">起点</span>
                  <strong>{formatPose(startPose)}</strong>
                </div>
                <div className="hud-card">
                  <span className="hud-section-title">终点</span>
                  <strong>{formatPose(goalPose)}</strong>
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
                  {appearanceLayerControls.map((control) => (
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
                      title={control.label}
                      onClick={() => toggleLayer(control.key)}
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
                    className={clsx('mode-button', pickMode === 'surface_start' && 'mode-button-active')}
                    type="button"
                    onClick={() => handlePickModeChange('surface_start')}
                    disabled={!scene || loadingScene || isGenerating || isExecuting}
                  >
                    {UI_LABELS.btnPickStart}
                  </button>
                  <button
                    className={clsx('mode-button', pickMode === 'surface_goal' && 'mode-button-active')}
                    type="button"
                    onClick={() => handlePickModeChange('surface_goal')}
                    disabled={!scene || loadingScene || isGenerating || isExecuting}
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
                <span className="hud-section-title">{UI_LABELS.panelRoute}</span>
                <div className="button-row run-dock">
                  <button
                    className="primary-button"
                    type="button"
                    onClick={handleGenerateRoute}
                    disabled={!scene || loadingScene || !surfaceStartPick || !surfaceGoalPick || isGenerating}
                  >
                    {isGenerating ? '生成中...' : UI_LABELS.btnGenerateRoute}
                  </button>
                  <button
                    className="secondary-button"
                    type="button"
                    onClick={handleExecuteRoute}
                    disabled={!routeReady || isExecuting}
                  >
                    {isExecuting ? '下发中...' : UI_LABELS.btnExecuteRoute}
                  </button>
                  <button
                    className="secondary-button"
                    type="button"
                    onClick={clearAllRoute}
                    disabled={loadingScene}
                  >
                    {UI_LABELS.btnClearRoute}
                  </button>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className="panel debug-panel">
          <div className="debug-grid">
            <section className="panel-section">
              <h2>{UI_LABELS.panelRoute}</h2>
              <div className="trace-card">
                <div className="trace-title">点击与投影结果</div>
                <div className="metrics-list">
                  <div className="metric-row">
                    <span>请求起点</span>
                    <strong>{formatPose(surfaceStartPick)}</strong>
                  </div>
                  <div className="metric-row">
                    <span>请求终点</span>
                    <strong>{formatPose(surfaceGoalPick)}</strong>
                  </div>
                  <div className="metric-row">
                    <span>投影起点</span>
                    <strong>{formatPose(surfaceProjectedStart)}</strong>
                  </div>
                  <div className="metric-row">
                    <span>投影终点</span>
                    <strong>{formatPose(surfaceProjectedGoal)}</strong>
                  </div>
                  <div className="metric-row">
                    <span>{UI_LABELS.fieldStartNode}</span>
                    <strong>{traceSummary?.projectedStartNodeId ?? 'N/A'}</strong>
                  </div>
                  <div className="metric-row">
                    <span>{UI_LABELS.fieldGoalNode}</span>
                    <strong>{traceSummary?.projectedGoalNodeId ?? 'N/A'}</strong>
                  </div>
                </div>
              </div>

              <div className="debug-divider" />

              <div className="stats-grid">
                <RouteStats title={UI_LABELS.statPathPoints} value={String(surfacePath.length)} />
                <RouteStats title={UI_LABELS.statSegments} value={String(surfaceSegments.length)} />
                <RouteStats
                  title={UI_LABELS.statCost}
                  value={traceSummary?.totalCost == null ? 'N/A' : formatMetricValue(traceSummary.totalCost)}
                />
                <RouteStats title={UI_LABELS.statFrame} value={traceProgressDetail} />
              </div>
            </section>

            <section className="panel-section">
              <h2>{UI_LABELS.panelTrace}</h2>
              <div className="trace-card">
                <div className="trace-title">{traceTitle}</div>
                <div className="trace-meta">
                  <span>{currentFrame ? formatFramePhase(currentFrame.phase) : '等待生成'}</span>
                  <span>{`帧 ${traceProgressDetail}`}</span>
                </div>

                <div className="debug-divider" />

                <label className="field-label" htmlFor="trace-index">{UI_LABELS.fieldTraceIndex}</label>
                <input
                  id="trace-index"
                  className="range-input"
                  type="range"
                  min={0}
                  max={Math.max(traceFrames.length - 1, 0)}
                  step={1}
                  value={Math.min(traceIndex, Math.max(traceFrames.length - 1, 0))}
                  onChange={(event) => setTraceIndex(Number(event.target.value))}
                  disabled={traceFrames.length === 0}
                />
                <div className="range-readout">{traceProgressDetail}</div>

                <div className="metrics-list">
                  <div className="metric-row">
                    <span>{UI_LABELS.statFrontier}</span>
                    <strong>{currentFrame?.openSet.length ?? 0}</strong>
                  </div>
                  <div className="metric-row">
                    <span>{UI_LABELS.statExpanded}</span>
                    <strong>{currentFrame?.expandedNodes.length ?? 0}</strong>
                  </div>
                  {frameMetrics.length === 0 && <div className="empty-note">{UI_LABELS.hintTraceEmpty}</div>}
                  {frameMetrics.map(([key, value]) => (
                    <div key={key} className="metric-row">
                      <span>{key}</span>
                      <strong>{formatMetricValue(value)}</strong>
                    </div>
                  ))}
                </div>
              </div>
            </section>

            <section className="panel-section">
              <h2>{UI_LABELS.panelSegments}</h2>
              <div className="trace-card">
                <div className="trace-title">路线语义分段</div>
                <div className="trace-meta">
                  <span>{UI_LABELS.hintExecute}</span>
                </div>
                <div className="metrics-list">
                  {surfaceSegments.length === 0 && <div className="empty-note">{UI_LABELS.hintTraceEmpty}</div>}
                  {surfaceSegments.map((segment) => (
                    <div key={segment.segment_id} className="metric-row">
                      <span>{`${segment.from_node_id} -> ${segment.to_node_id}`}</span>
                      <strong>
                        {`${MOTION_TYPE_LABELS[segment.motion_type] ?? segment.motion_type} · ${segment.point_count} 点`}
                      </strong>
                    </div>
                  ))}
                </div>
              </div>
            </section>
          </div>
        </section>
      </main>
    </div>
  );
}
