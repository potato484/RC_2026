import { startTransition, useEffect, useMemo, useRef, useState } from 'react';
import clsx from 'clsx';

import {
  executeSurfaceRoute,
  fetchLocalPlannerScenarios,
  fetchSceneManifest,
  previewSurfaceRoute,
  startLiveBridge,
  traceLocalPlannerScenario,
  traceSurfaceRouteFromNodes,
} from './api';
import { SceneCanvas } from './components/SceneCanvas';
import {
  formatFrameLabel,
  formatFrameMetricLabel,
  formatFramePhase,
  formatFailureSummary,
  formatNodeLabel,
  formatNodeTransitionLabel,
  formatPlanningLogFieldValue,
  formatUnexpectedError,
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
  LocalPlannerScenario,
  LiveEvent,
  PlanningLogEntry,
  PickMode,
  PlannerFrame,
  PlannerTraceFrame,
  Pose3,
  RouteTraceSummary,
  SceneManifest,
  SurfaceRouteSegment,
  SurfaceRoutePlanningTiming,
  SurfaceRoutePreviewResponse,
  Team,
  ViewMode,
} from './types';

function formatPose(pose: Pose3 | null): string {
  if (!pose) {
    return UI_LABELS.emptyValue;
  }
  return `${pose.x.toFixed(2)}，${pose.y.toFixed(2)}，${pose.z.toFixed(2)}`;
}

function formatMetricValue(value: unknown): string {
  if (typeof value === 'number') {
    return Number.isInteger(value) ? String(value) : value.toFixed(3);
  }
  if (typeof value === 'boolean') {
    return value ? '是' : '否';
  }
  if (value == null) {
    return UI_LABELS.emptyValue;
  }
  if (typeof value === 'object') {
    return '已记录';
  }
  return String(value);
}

function formatStringList(values: string[] | undefined): string {
  if (!values || values.length === 0) {
    return UI_LABELS.emptyValue;
  }
  return values.join('，');
}

function formatElapsedMs(value: number | null | undefined): string {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    return UI_LABELS.emptyValue;
  }
  return `${value.toFixed(2)} 毫秒`;
}

function formatLevel(level: number | null | undefined): string {
  if (typeof level !== 'number' || !Number.isFinite(level)) {
    return UI_LABELS.emptyValue;
  }
  if (level <= 0) {
    return '正常';
  }
  if (level === 1) {
    return '注意';
  }
  if (level === 2) {
    return '告警';
  }
  return `严重(${level})`;
}

function formatSeverity(severity: number | null | undefined): string {
  if (typeof severity !== 'number' || !Number.isFinite(severity)) {
    return UI_LABELS.emptyValue;
  }
  if (severity <= 1) {
    return '信息';
  }
  if (severity === 2) {
    return '提醒';
  }
  if (severity === 3) {
    return '告警';
  }
  return `严重(${severity})`;
}

function resolveTimingMs(...values: Array<number | null | undefined>): number | null {
  for (const value of values) {
    if (typeof value === 'number' && Number.isFinite(value)) {
      return value;
    }
  }
  return null;
}

function formatPlanningLogLevel(level: PlanningLogEntry['level']): string {
  if (level === 'error') {
    return '失败';
  }
  if (level === 'warn') {
    return '告警';
  }
  return '完成';
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

function buildPreviewSummary(
  response: SurfaceRoutePreviewResponse,
  requestedStart: Pose3,
  requestedGoal: Pose3,
): RouteTraceSummary {
  return {
    projectedStartNodeId: response.projected_start_node_id,
    projectedGoalNodeId: response.projected_goal_node_id,
    requestedStart,
    requestedGoal,
    surfaceProjectionMs: response.planning_timing_ms?.surfaceProjection ?? null,
    surfacePlanningMs: response.planning_timing_ms?.surfacePlanning ?? null,
    surfacePathExpandMs: response.planning_timing_ms?.surfacePathExpand ?? null,
    surfaceSegmentBuildMs: response.planning_timing_ms?.surfaceSegmentBuild ?? null,
    surfaceCompletePlanningMs: response.planning_timing_ms?.surfaceCompletePlanning ?? null,
    previewElapsedMs: response.planning_timing_ms?.surfaceRouteCli ?? null,
    tracePlanningMs: null,
    traceElapsedMs: null,
    totalElapsedMs: response.planning_timing_ms?.surfaceRouteCli ?? null,
  };
}

function previewDisplayPath(response: SurfaceRoutePreviewResponse): Pose3[] {
  if (response.success) {
    return response.path_points;
  }
  return response.fallback_path_points ?? [];
}

function previewDisplaySegments(response: SurfaceRoutePreviewResponse): SurfaceRouteSegment[] {
  if (response.success) {
    return response.segments;
  }
  return response.fallback_segments ?? [];
}

function previewHasReferenceRoute(response: SurfaceRoutePreviewResponse): boolean {
  return !response.success && previewDisplayPath(response).length > 0;
}

function mergePlanningTiming(
  current: SurfaceRoutePlanningTiming | null,
  next: SurfaceRoutePlanningTiming | null | undefined,
): SurfaceRoutePlanningTiming | null {
  if (!current && !next) {
    return null;
  }
  const merged: SurfaceRoutePlanningTiming = {
    ...(current ?? {}),
    ...(next ?? {}),
  };
  const previewChain = merged.surfaceRouteCli;
  const traceChain = merged.plannerTraceCli;
  if (typeof previewChain === 'number' && Number.isFinite(previewChain)) {
    if (typeof traceChain === 'number' && Number.isFinite(traceChain)) {
      merged.surfaceRouteTraceTotal = Number((previewChain + traceChain).toFixed(2));
    } else {
      merged.surfaceRouteTraceTotal = previewChain;
    }
  }
  return merged;
}

function buildLiveEventsUrl(): string {
  const rawBase = (import.meta.env.VITE_API_BASE_URL ?? '').trim().replace(/\/$/, '');
  if (rawBase.startsWith('http://') || rawBase.startsWith('https://')) {
    const url = new URL(rawBase);
    url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
    url.pathname = `${url.pathname.replace(/\/$/, '')}/api/live/events`;
    url.search = '';
    return url.toString();
  }

  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const basePath = rawBase.startsWith('/') ? rawBase : '';
  return `${protocol}//${window.location.host}${basePath}/api/live/events`;
}

export default function App() {
  const scene = useSimStore((state) => state.scene);
  const loadingScene = useSimStore((state) => state.loadingScene);
  const team = useSimStore((state) => state.team);
  const viewMode = useSimStore((state) => state.viewMode);
  const layoutPresetId = useSimStore((state) => state.layoutPresetId);
  const layers = useSimStore((state) => state.layers);
  const statusMessage = useSimStore((state) => state.statusMessage);

  const setTeam = useSimStore((state) => state.setTeam);
  const setSceneLoading = useSimStore((state) => state.setSceneLoading);
  const setScene = useSimStore((state) => state.setScene);
  const setViewMode = useSimStore((state) => state.setViewMode);
  const applyLayoutPreset = useSimStore((state) => state.applyLayoutPreset);
  const toggleLayer = useSimStore((state) => state.toggleLayer);
  const setLayerVisible = useSimStore((state) => state.setLayerVisible);
  const setStatusMessage = useSimStore((state) => state.setStatusMessage);

  const [pickMode, setPickMode] = useState<PickMode>('idle');
  const [surfaceStartPick, setSurfaceStartPick] = useState<Pose3 | null>(null);
  const [surfaceGoalPick, setSurfaceGoalPick] = useState<Pose3 | null>(null);
  const [surfaceHoverPose, setSurfaceHoverPose] = useState<Pose3 | null>(null);
  const [surfaceProjectedStart, setSurfaceProjectedStart] = useState<Pose3 | null>(null);
  const [surfaceProjectedGoal, setSurfaceProjectedGoal] = useState<Pose3 | null>(null);
  const [surfacePath, setSurfacePath] = useState<Pose3[]>([]);
  const [surfaceSegments, setSurfaceSegments] = useState<SurfaceRouteSegment[]>([]);
  const [routeAccepted, setRouteAccepted] = useState(false);
  const [traceFrames, setTraceFrames] = useState<PlannerTraceFrame[]>([]);
  const [traceNodePoses, setTraceNodePoses] = useState<Record<string, Pose3>>({});
  const [traceSummary, setTraceSummary] = useState<RouteTraceSummary | null>(null);
  const [planningTiming, setPlanningTiming] = useState<SurfaceRoutePlanningTiming | null>(null);
  const [planningLogs, setPlanningLogs] = useState<PlanningLogEntry[]>([]);
  const [traceIndex, setTraceIndex] = useState(0);
  const [isGenerating, setIsGenerating] = useState(false);
  const [isTraceLoading, setIsTraceLoading] = useState(false);
  const [isExecuting, setIsExecuting] = useState(false);
  const [liveEvent, setLiveEvent] = useState<LiveEvent | null>(null);
  const [localPlannerScenarios, setLocalPlannerScenarios] = useState<LocalPlannerScenario[]>([]);
  const [selectedLocalPlannerScenario, setSelectedLocalPlannerScenario] = useState('');
  const [isLocalPlannerTraceLoading, setIsLocalPlannerTraceLoading] = useState(false);
  const [localPlannerTraceSummary, setLocalPlannerTraceSummary] = useState<{
    snapshotLabel: string;
    finalStatus: string;
    finalReason: string;
    candidateCount: number;
  } | null>(null);
  const routeRequestSeq = useRef(0);

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
    () => deriveLayerControls({ layers, frame: currentFrame, scene, liveEvent }),
    [layers, currentFrame, scene, liveEvent],
  );
  const primaryLayerControls = layerControls.filter((control) => control.group === 'primary' && control.visible);
  const appearanceLayerControls = layerControls.filter((control) => control.group === 'advanced' && control.visible);

  const startPose = surfaceProjectedStart ?? surfaceStartPick;
  const goalPose = surfaceProjectedGoal ?? surfaceGoalPick;
  const viewModes: ViewMode[] = ['orbit', 'follow', 'top_ortho', 'side_perspective'];
  const viewerTitle = scene?.viewerMeta?.viewer_title || UI_LABELS.appTitle;
  const viewerSubtitle = scene?.viewerMeta?.viewer_subtitle || UI_LABELS.appSubtitle;
  const layoutPresets = scene?.layoutPresets ?? [];
  const activeLayoutPreset = layoutPresets.find((preset) => preset.id === layoutPresetId) ?? null;
  const activePhaseZone =
    scene?.semanticZones?.find((zone) => zone.phase_key === liveEvent?.btSnapshot?.activeSubtreeId) ?? null;
  const activeKeepoutCount =
    liveEvent?.blockOverlay?.filter((cell) => cell.keepoutActive).length ?? 0;
  const blockedGridIds = useMemo(
    () =>
      (liveEvent?.blockOverlay ?? [])
        .filter((cell) => cell.keepoutActive)
        .map((cell) => cell.gridId),
    [liveEvent?.blockOverlay],
  );
  const liveRobotPose = liveEvent?.controlState?.pose ?? currentFrame?.robotPose ?? null;
  const frameMetrics = Object.entries(currentFrame?.metrics ?? {})
    .filter(([key]) => key !== 'traceMode')
    .map(([key, value]) => ({
      key,
      label: formatFrameMetricLabel(key),
      value,
    }));
  const traceTitle = currentFrame
    ? formatFrameLabel(currentFrame.label, currentFrame.phase)
    : isTraceLoading
      ? UI_LABELS.statusBackgroundReplay
      : UI_LABELS.hintTraceEmpty;
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
  const routeReady = surfacePath.length > 0;
  const routeRejected = routeReady && !routeAccepted;
  const traceReady = traceFrames.length > 0;
  const traceStatusText = routeReady
    ? routeRejected
      ? '车体约束拒绝该路线，当前显示参考路径'
      : isTraceLoading
      ? '路线已生成，回放补齐中'
      : traceReady
        ? `已生成 ${traceFrameOverview} 帧搜索回放`
        : '路线已生成，等待回放'
    : '等待生成路线';
  const surfaceCompletePlanningMs = resolveTimingMs(
    traceSummary?.surfaceCompletePlanningMs,
    planningTiming?.surfaceCompletePlanning,
  );
  const surfaceProjectionMs = resolveTimingMs(traceSummary?.surfaceProjectionMs, planningTiming?.surfaceProjection);
  const surfacePlanningMs = resolveTimingMs(traceSummary?.surfacePlanningMs, planningTiming?.surfacePlanning);
  const surfacePathExpandMs = resolveTimingMs(traceSummary?.surfacePathExpandMs, planningTiming?.surfacePathExpand);
  const surfaceSegmentBuildMs = resolveTimingMs(traceSummary?.surfaceSegmentBuildMs, planningTiming?.surfaceSegmentBuild);
  const tracePlanningMs = resolveTimingMs(traceSummary?.tracePlanningMs, planningTiming?.tracePlanning);
  const previewChainMs = resolveTimingMs(traceSummary?.previewElapsedMs, planningTiming?.surfaceRouteCli);
  const traceChainMs = resolveTimingMs(traceSummary?.traceElapsedMs, planningTiming?.plannerTraceCli);
  const totalElapsedMs = resolveTimingMs(traceSummary?.totalElapsedMs, planningTiming?.surfaceRouteTraceTotal);
  const timingHeadlineParts = [
    surfaceCompletePlanningMs == null ? null : `${UI_LABELS.statCompletePlanning} ${formatElapsedMs(surfaceCompletePlanningMs)}`,
    previewChainMs == null ? null : `${UI_LABELS.statPreviewChain} ${formatElapsedMs(previewChainMs)}`,
    tracePlanningMs == null ? null : `${UI_LABELS.statTracePlanning} ${formatElapsedMs(tracePlanningMs)}`,
    traceChainMs == null ? null : `${UI_LABELS.statTraceChain} ${formatElapsedMs(traceChainMs)}`,
  ].filter((value): value is string => value != null);
  const timingHeadline = timingHeadlineParts.length > 0 ? timingHeadlineParts.join('，') : '等待生成时间摘要';
  const hasError = statusMessage.includes('失败') || statusMessage.includes('错误') || statusMessage.includes('error');
  const busy = isGenerating || isTraceLoading || isExecuting;
  const pickHint =
    pickMode === 'surface_start'
      ? UI_LABELS.hintPickSurfaceStart
      : pickMode === 'surface_goal'
        ? UI_LABELS.hintPickSurfaceGoal
        : UI_LABELS.hintPickIdle;
  const activeEventList = (liveEvent?.visualizationEvents ?? []).filter((event) => event.active);
  const btEventList = liveEvent?.btEvents ?? [];
  const liveTracking = liveEvent?.trackingState ?? null;
  const livePlanner = liveEvent?.localPlannerState ?? null;
  const liveRecovery = liveEvent?.recoveryState ?? null;
  const liveSemantic = liveEvent?.semanticSummary ?? null;
  const liveMotionMode = liveEvent?.motionModeState ?? null;
  const liveLocalizationHealth = liveEvent?.localizationHealth ?? null;
  const liveLocalizationBackend = liveEvent?.localizationBackendStatus ?? null;
  const liveOperatorStatus = liveEvent?.operatorStatus ?? null;
  const liveMechanismState = liveEvent?.mechanismState ?? null;
  const liveBtSnapshot = liveEvent?.btSnapshot ?? null;

  function clearGeneratedRoute() {
    routeRequestSeq.current += 1;
    setSurfaceProjectedStart(null);
    setSurfaceProjectedGoal(null);
    setSurfacePath([]);
    setSurfaceSegments([]);
    setRouteAccepted(false);
    setTraceFrames([]);
    setTraceNodePoses({});
    setTraceSummary(null);
    setPlanningTiming(null);
    setPlanningLogs([]);
    setTraceIndex(0);
    setIsTraceLoading(false);
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
        setStatusMessage(`${UI_LABELS.statusError}: ${formatUnexpectedError(error, '场景加载请求失败')}`);
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

  useEffect(() => {
    let active = true;
    fetchLocalPlannerScenarios()
      .then((scenarios) => {
        if (!active) {
          return;
        }
        setLocalPlannerScenarios(scenarios);
        setSelectedLocalPlannerScenario((current) => current || scenarios[0]?.name || '');
      })
      .catch(() => {
        if (active) {
          setLocalPlannerScenarios([]);
        }
      });
    return () => {
      active = false;
    };
  }, []);

  useEffect(() => {
    let closed = false;
    let socket: WebSocket | null = null;

    startLiveBridge()
      .then((response) => {
        if (closed) {
          return;
        }
        if (response.snapshot) {
          startTransition(() => {
            setLiveEvent(response.snapshot ?? null);
          });
        }
        socket = new WebSocket(buildLiveEventsUrl());
        socket.onmessage = (event) => {
          try {
            const payload = JSON.parse(event.data) as LiveEvent;
            startTransition(() => {
              setLiveEvent(payload);
            });
          } catch {
            // Ignore malformed live events.
          }
        };
      })
      .catch(() => {
        // Live bridge is optional for offline trace usage.
      });

    return () => {
      closed = true;
      socket?.close();
    };
  }, []);

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
      setStatusMessage('终点已记录，可以生成三维路线');
    }
    setPickMode('idle');
  }

  async function handleGenerateRoute() {
    if (!surfaceStartPick || !surfaceGoalPick) {
      setStatusMessage('先在场景里设置起点和终点');
      return;
    }

    // Route generation should always surface the resulting preview, even if the
    // active layout preset previously hid the route layer.
    setLayerVisible('route', true);

    const requestId = routeRequestSeq.current + 1;
    routeRequestSeq.current = requestId;
    let previewResponse: SurfaceRoutePreviewResponse | null = null;
    setIsGenerating(true);
    setIsTraceLoading(false);
    setSurfaceProjectedStart(null);
    setSurfaceProjectedGoal(null);
    setSurfacePath([]);
    setSurfaceSegments([]);
    setRouteAccepted(false);
    setTraceFrames([]);
    setTraceNodePoses({});
    setTraceSummary(null);
    setTraceIndex(0);
    setPlanningLogs([]);
    setPlanningTiming(null);
    try {
      const preview = await previewSurfaceRoute({
        team,
        start_pick_world: surfaceStartPick,
        goal_pick_world: surfaceGoalPick,
      });
      previewResponse = preview;
      if (routeRequestSeq.current !== requestId) {
        return;
      }

      setPlanningTiming(preview.planning_timing_ms ?? null);
      setPlanningLogs(preview.planning_logs ?? []);
      if (!preview.success) {
        const fallbackPath = previewDisplayPath(preview);
        const fallbackSegments = previewDisplaySegments(preview);
        setSurfaceProjectedStart(preview.projected_start ?? null);
        setSurfaceProjectedGoal(preview.projected_goal ?? null);
        setSurfacePath(fallbackPath);
        setSurfaceSegments(fallbackSegments);
        setRouteAccepted(false);
        setTraceFrames([]);
        setTraceNodePoses({});
        setTraceSummary(null);
        setTraceIndex(0);
        if (previewHasReferenceRoute(preview)) {
          const backendLabel = preview.fallback_planner_backend?.trim() || 'legacy';
          setStatusMessage(
            `三维路线未通过车体约束: ${formatFailureSummary(preview.failure_reason, preview.failure_code)}，已显示 ${backendLabel} 参考路线`,
          );
          return;
        }
        setStatusMessage(`三维路线生成失败: ${formatFailureSummary(preview.failure_reason, preview.failure_code)}`);
        return;
      }

      setSurfaceProjectedStart(preview.projected_start);
      setSurfaceProjectedGoal(preview.projected_goal);
      setSurfacePath(preview.path_points);
      setSurfaceSegments(preview.segments);
      setRouteAccepted(true);
      setTraceSummary(buildPreviewSummary(preview, surfaceStartPick, surfaceGoalPick));
      const currentSurfacePlanningMs = resolveTimingMs(
        preview.planning_timing_ms?.surfaceCompletePlanning,
      );
      const currentPreviewChainMs = resolveTimingMs(
        preview.planning_timing_ms?.surfaceRouteCli,
      );
      const planningHint =
        currentSurfacePlanningMs == null ? '' : `，完整规划 ${formatElapsedMs(currentSurfacePlanningMs)}`;
      const elapsedHint =
        currentPreviewChainMs == null ? '' : `，网页预览链路 ${formatElapsedMs(currentPreviewChainMs)}`;
      setStatusMessage(`三维路线已生成，共 ${preview.segments.length} 段${planningHint}${elapsedHint}，正在后台生成搜索回放`);
    } catch (error) {
      if (routeRequestSeq.current === requestId) {
        const requestErrorMessage = formatUnexpectedError(error, '浏览器到规划服务的请求失败');
        setSurfaceProjectedStart(null);
        setSurfaceProjectedGoal(null);
        setSurfacePath([]);
        setSurfaceSegments([]);
        setRouteAccepted(false);
        setTraceFrames([]);
        setTraceNodePoses({});
        setTraceSummary(null);
        setTraceIndex(0);
        setPlanningTiming(null);
        setPlanningLogs([
          {
            stage: 'browser_request',
            level: 'error',
            title: '浏览器请求失败',
            message: requestErrorMessage,
            elapsed_ms: null,
            fields: [],
          },
        ]);
        setStatusMessage(`三维路线生成失败: ${requestErrorMessage}`);
      }
    } finally {
      if (routeRequestSeq.current === requestId) {
        setIsGenerating(false);
      }
    }

    if (routeRequestSeq.current !== requestId) {
      return;
    }
    if (!previewResponse?.success) {
      return;
    }

    const startNodeId = previewResponse.projected_start_node_id;
    const goalNodeId = previewResponse.projected_goal_node_id;
    if (!startNodeId || !goalNodeId) {
      setStatusMessage('三维路线已生成，但缺少投影节点，无法补齐搜索回放');
      return;
    }

    setIsTraceLoading(true);
    try {
      const trace = await traceSurfaceRouteFromNodes({
        team,
        surface_graph_file: previewResponse.surface_graph_file,
        start_node_id: startNodeId,
        goal_node_id: goalNodeId,
        requested_start: surfaceStartPick,
        requested_goal: surfaceGoalPick,
      });
      if (routeRequestSeq.current !== requestId) {
        return;
      }

      setTraceFrames(trace.frames);
      setTraceNodePoses(trace.node_poses ?? {});
      setTraceIndex(Math.max(trace.frames.length - 1, 0));
      setPlanningTiming((current) => mergePlanningTiming(current, trace.planning_timing_ms ?? null));
      setPlanningLogs((current) => [...current, ...(trace.planning_logs ?? [])]);
      setTraceSummary((current) => {
        const previewSummary = current ?? buildPreviewSummary(previewResponse, surfaceStartPick, surfaceGoalPick);
        const mergedSummary: RouteTraceSummary = {
          ...previewSummary,
          ...trace.summary,
        };
        const previewChain = resolveTimingMs(previewSummary.previewElapsedMs, previewResponse.planning_timing_ms?.surfaceRouteCli);
        const traceChain = resolveTimingMs(trace.summary.traceElapsedMs, trace.planning_timing_ms?.plannerTraceCli);
        mergedSummary.previewElapsedMs = previewChain;
        mergedSummary.traceElapsedMs = traceChain;
        mergedSummary.totalElapsedMs =
          previewChain == null ? traceChain : traceChain == null ? previewChain : Number((previewChain + traceChain).toFixed(2));
        mergedSummary.surfaceProjectionMs = previewSummary.surfaceProjectionMs ?? null;
        mergedSummary.surfacePlanningMs = previewSummary.surfacePlanningMs ?? null;
        mergedSummary.surfacePathExpandMs = previewSummary.surfacePathExpandMs ?? null;
        mergedSummary.surfaceSegmentBuildMs = previewSummary.surfaceSegmentBuildMs ?? null;
        mergedSummary.surfaceCompletePlanningMs = previewSummary.surfaceCompletePlanningMs ?? null;
        return mergedSummary;
      });

      if (!trace.success) {
        setStatusMessage(`三维路线已生成，但搜索回放失败: ${formatFailureSummary(trace.failure_reason, trace.failure_code)}`);
        return;
      }

      const sampledHint =
        trace.summary.framesSampled && (trace.summary.framesCount ?? trace.frames.length) > trace.frames.length
          ? `，回放已压缩为 ${trace.frames.length} / ${trace.summary.framesCount} 帧`
          : `，回放 ${trace.frames.length} 帧`;
      setStatusMessage(`三维路线与搜索回放已就绪，共 ${previewResponse.segments.length} 段${sampledHint}`);
    } catch (error) {
      if (routeRequestSeq.current !== requestId) {
        return;
      }
      const traceErrorMessage = formatUnexpectedError(error, '浏览器到回放服务的请求失败');
      setPlanningLogs((current) => [
        ...current,
        {
          stage: 'browser_trace_request',
          level: 'error',
          title: '搜索回放请求失败',
          message: traceErrorMessage,
          elapsed_ms: null,
          fields: [],
        },
      ]);
      setStatusMessage(`三维路线已生成，但搜索回放请求失败: ${traceErrorMessage}`);
    } finally {
      if (routeRequestSeq.current === requestId) {
        setIsTraceLoading(false);
      }
    }
  }

  async function handleExecuteRoute() {
    if (!surfaceStartPick || !surfaceGoalPick) {
      setStatusMessage('先在场景里设置起点和终点');
      return;
    }
    if (!routeReady) {
      setStatusMessage('先生成三维路线，再决定是否执行');
      return;
    }
    if (!routeAccepted) {
      setStatusMessage('当前显示的是参考路线，车体约束未通过，不能直接执行');
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
        setRouteAccepted(true);
      } else {
        setSurfaceProjectedStart(response.preview.projected_start ?? null);
        setSurfaceProjectedGoal(response.preview.projected_goal ?? null);
        setSurfacePath(previewDisplayPath(response.preview));
        setSurfaceSegments(previewDisplaySegments(response.preview));
        setRouteAccepted(false);
      }
      setPlanningTiming(response.preview.planning_timing_ms ?? null);
      setPlanningLogs(response.preview.planning_logs ?? []);
      if (!response.accepted) {
        const reason = formatFailureSummary(response.preview.failure_reason, response.preview.failure_code) || '目标被拒绝';
        setStatusMessage(`路线未被接受: ${reason}`);
        return;
      }
      setStatusMessage('路线已下发给运行时，机器人需已经在起点附近');
    } catch (error) {
      setStatusMessage(`路线执行失败: ${formatUnexpectedError(error, '路线下发请求失败')}`);
    } finally {
      setIsExecuting(false);
    }
  }

  async function handleLoadLocalPlannerTrace() {
    if (!selectedLocalPlannerScenario) {
      setStatusMessage('先选择一个局部规划案例');
      return;
    }

    setIsLocalPlannerTraceLoading(true);
    try {
      const trace = await traceLocalPlannerScenario({
        scenario_name: selectedLocalPlannerScenario,
      });
      setTraceFrames(trace.frames);
      setTraceIndex(Math.max(trace.frames.length - 1, 0));
      setLocalPlannerTraceSummary({
        snapshotLabel: trace.snapshotLabel,
        finalStatus: trace.summary.finalStatus,
        finalReason: trace.summary.finalReason,
        candidateCount: trace.summary.candidateCount,
      });
      setPlanningLogs([
        {
          stage: 'local_planner_trace_cli',
          level: trace.success ? 'info' : 'error',
          title: '局部规划案例回放',
          message: `${trace.snapshotLabel} 已导入局部规划候选轨迹`,
          elapsed_ms: null,
          fields: [
            { label: '最终状态', value: trace.summary.finalStatus },
            { label: '候选轨迹数', value: String(trace.summary.candidateCount) },
            { label: '最终原因', value: trace.summary.finalReason || UI_LABELS.emptyValue },
          ],
        },
      ]);
      setStatusMessage(
        `已载入局部规划案例 ${trace.snapshotLabel}，最终状态 ${trace.summary.finalStatus}`,
      );
    } catch (error) {
      setStatusMessage(`局部规划案例加载失败: ${formatUnexpectedError(error, '局部规划 trace 请求失败')}`);
    } finally {
      setIsLocalPlannerTraceLoading(false);
    }
  }

  return (
    <div className="app-shell">
      <header className="topbar">
        <div>
          <p className="eyebrow">{UI_LABELS.appEyebrow}</p>
          <h1 className="title">{viewerTitle}</h1>
          <p className="subtitle">{viewerSubtitle}</p>
        </div>
        <div className="topbar-actions">
          {activeLayoutPreset && (
            <div className="status-pill">
              <span className="status-dot" />
              {`${UI_LABELS.fieldLayoutPreset}: ${activeLayoutPreset.label}`}
            </div>
          )}
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
            {layoutPresets.length > 0 && (
              <div className="command-group" aria-label={UI_LABELS.panelLayout}>
                {layoutPresets.map((preset) => (
                  <button
                    key={preset.id}
                    type="button"
                    className={clsx('mode-button', layoutPresetId === preset.id && 'mode-button-active')}
                    onClick={() => applyLayoutPreset(preset.id)}
                    aria-label={preset.label}
                    title={preset.description}
                    disabled={loadingScene}
                  >
                    {preset.label}
                  </button>
                ))}
              </div>
            )}
            <div className="command-note">
              {activeLayoutPreset?.description ?? UI_LABELS.routeModeHint}
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
              hoverPose={surfaceHoverPose}
              manualPath={surfacePath}
              manualPathRejected={routeRejected}
              blockedGridIds={blockedGridIds}
              pickMode={pickMode}
              onHoverWorldChange={setSurfaceHoverPose}
              onPickWorld={handleSurfaceScenePick}
            />

            <div className="canvas-overlay">
              <div className="canvas-overlay-card canvas-overlay-card-strong">
                <strong>{UI_LABELS.routeTitle}</strong>
                <span>{traceStatusText}</span>
              </div>
              <div className="canvas-overlay-card">
                <strong>{UI_LABELS.panelTiming}</strong>
                <span>{timingHeadline}</span>
              </div>
              <div className="canvas-overlay-card">
                <strong>{UI_LABELS.fieldActivePhase}</strong>
                <span>{activePhaseZone?.label ?? UI_LABELS.emptyValue}</span>
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
                <div className="hud-card">
                  <span className="hud-section-title">{UI_LABELS.fieldRobotPose}</span>
                  <strong>{formatPose(liveRobotPose)}</strong>
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

              <div className="hud-card hud-card-wide">
                <span className="hud-section-title">{UI_LABELS.panelLegend}</span>
                <div className="canvas-legend-list">
                  <div className="canvas-legend-item">
                    <span className="layer-mark layer-mark-openSet" aria-hidden="true" />
                    <div className="canvas-legend-copy">
                      <strong>{UI_LABELS.legendOpenSet}</strong>
                      <span>{UI_LABELS.legendOpenSetHint}</span>
                    </div>
                  </div>
                  <div className="canvas-legend-item">
                    <span className="layer-mark layer-mark-expanded" aria-hidden="true" />
                    <div className="canvas-legend-copy">
                      <strong>{UI_LABELS.legendExpanded}</strong>
                      <span>{UI_LABELS.legendExpandedHint}</span>
                    </div>
                  </div>
                  <div className="canvas-legend-item">
                    <span className="layer-mark layer-mark-phaseZones" aria-hidden="true" />
                    <div className="canvas-legend-copy">
                      <strong>{UI_LABELS.legendPhaseZones}</strong>
                      <span>{UI_LABELS.legendPhaseZonesHint}</span>
                    </div>
                  </div>
                </div>
                <div className="canvas-legend-note">{UI_LABELS.legendLayerHint}</div>
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
                <div className="hud-metric">
                  <span>{UI_LABELS.statCompletePlanning}</span>
                  <strong>{formatElapsedMs(surfaceCompletePlanningMs)}</strong>
                </div>
                <div className="button-row run-dock">
                  <button
                    className="primary-button"
                    type="button"
                    onClick={handleGenerateRoute}
                    disabled={!scene || loadingScene || !surfaceStartPick || !surfaceGoalPick || isGenerating}
                  >
                    {isGenerating ? UI_LABELS.statusGenerating : UI_LABELS.btnGenerateRoute}
                  </button>
                  <button
                    className="secondary-button"
                    type="button"
                    onClick={handleExecuteRoute}
                    disabled={!routeAccepted || isExecuting}
                  >
                    {isExecuting ? UI_LABELS.statusDispatching : UI_LABELS.btnExecuteRoute}
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
          <div className="summary-strip" data-testid="summary-strip">
            <RouteStats
              title={UI_LABELS.fieldLayoutPreset}
              value={activeLayoutPreset?.label ?? UI_LABELS.emptyValue}
            />
            <RouteStats
              title={UI_LABELS.fieldActivePhase}
              value={activePhaseZone?.label ?? UI_LABELS.emptyValue}
            />
            <RouteStats
              title={UI_LABELS.statEvents}
              value={String(activeEventList.length)}
            />
            <RouteStats
              title={UI_LABELS.statKeepouts}
              value={String(activeKeepoutCount)}
            />
            <RouteStats title={UI_LABELS.statCompletePlanning} value={formatElapsedMs(surfaceCompletePlanningMs)} />
            <RouteStats title={UI_LABELS.statSurfaceProjection} value={formatElapsedMs(surfaceProjectionMs)} />
            <RouteStats title={UI_LABELS.statSurfacePlanning} value={formatElapsedMs(surfacePlanningMs)} />
            <RouteStats title={UI_LABELS.statPathExpand} value={formatElapsedMs(surfacePathExpandMs)} />
            <RouteStats title={UI_LABELS.statSegmentBuild} value={formatElapsedMs(surfaceSegmentBuildMs)} />
            <RouteStats title={UI_LABELS.statPreviewChain} value={formatElapsedMs(previewChainMs)} />
            <RouteStats title={UI_LABELS.statTracePlanning} value={formatElapsedMs(tracePlanningMs)} />
            <RouteStats title={UI_LABELS.statTraceChain} value={formatElapsedMs(traceChainMs)} />
            <RouteStats
              title={UI_LABELS.statCost}
              value={traceSummary?.totalCost == null ? UI_LABELS.emptyValue : formatMetricValue(traceSummary.totalCost)}
            />
            <RouteStats
              title={UI_LABELS.statFrame}
              value={traceReady ? traceProgressDetail : isTraceLoading ? UI_LABELS.statusGenerating : UI_LABELS.emptyValue}
            />
            <RouteStats title={UI_LABELS.statPathPoints} value={String(surfacePath.length)} />
            <RouteStats title={UI_LABELS.statSegments} value={String(surfaceSegments.length)} />
          </div>

          <div className="inspector-grid" data-testid="inspector-grid">
            <div className="inspector-column">
              <section className="panel-section">
                <h2>{UI_LABELS.panelTrace}</h2>
                <div className="trace-card">
                  <div className="trace-title">{traceTitle}</div>
                  <div className="trace-meta">
                    <span>{currentFrame ? formatFramePhase(currentFrame.phase) : isTraceLoading ? UI_LABELS.statusBackgroundReplay : UI_LABELS.statusPending}</span>
                    <span>{traceReady ? `帧 ${traceProgressDetail}` : isTraceLoading ? UI_LABELS.statusReplayPending : UI_LABELS.statusReplayEmpty}</span>
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
                    {frameMetrics.length === 0 && (
                      <div className="empty-note">
                        {isTraceLoading ? '路线已经生成，搜索回放正在后台补齐。' : UI_LABELS.hintTraceEmpty}
                      </div>
                    )}
                    {frameMetrics.map((metric) => (
                      <div key={metric.key} className="metric-row">
                        <span>{metric.label}</span>
                        <strong>{formatMetricValue(metric.value)}</strong>
                      </div>
                    ))}
                  </div>
                </div>
              </section>

              <section className="panel-section">
                <h2>{UI_LABELS.panelPlanningLogs}</h2>
                <div className="trace-card">
                  <div className="trace-title">{UI_LABELS.panelPlanningLogs}</div>
                  <div className="trace-meta">
                    <span>{UI_LABELS.hintPlanningLogs}</span>
                    <span>{totalElapsedMs == null ? UI_LABELS.statusPending : `${UI_LABELS.statTotalElapsed} ${formatElapsedMs(totalElapsedMs)}`}</span>
                  </div>
                  <div className="planning-log-list">
                    {planningLogs.length === 0 && <div className="empty-note">{UI_LABELS.hintPlanningLogsEmpty}</div>}
                    {planningLogs.map((entry, index) => (
                      <div
                        key={`${entry.stage}-${index}`}
                        className={clsx('planning-log-item', `planning-log-item-${entry.level}`)}
                      >
                        <div className="planning-log-header">
                          <div className="planning-log-heading">
                            <div className="planning-log-title">{entry.title}</div>
                            <div className="planning-log-message">{entry.message}</div>
                          </div>
                          <div className="planning-log-meta">
                            <span className={clsx('planning-log-badge', `planning-log-badge-${entry.level}`)}>
                              {formatPlanningLogLevel(entry.level)}
                            </span>
                            <span className="planning-log-elapsed">
                              {entry.elapsed_ms == null ? UI_LABELS.statusPending : formatElapsedMs(entry.elapsed_ms)}
                            </span>
                          </div>
                        </div>
                        {entry.fields.length > 0 && (
                          <div className="metrics-list planning-log-fields">
                            {entry.fields.map((field) => (
                              <div key={`${entry.stage}-${field.label}`} className="metric-row">
                                <span>{field.label}</span>
                                <strong className="planning-log-field-value">
                                  {formatPlanningLogFieldValue(field.label, field.value)}
                                </strong>
                              </div>
                            ))}
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                </div>
              </section>
            </div>

            <div className="inspector-column">
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
                      <strong>{formatNodeLabel(traceSummary?.projectedStartNodeId)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>{UI_LABELS.fieldGoalNode}</span>
                      <strong>{formatNodeLabel(traceSummary?.projectedGoalNodeId)}</strong>
                    </div>
                  </div>
                </div>
              </section>

              <section className="panel-section">
                <h2>{UI_LABELS.panelPlatformStatus}</h2>
                <div className="trace-card">
                  <div className="trace-title">{UI_LABELS.panelPlatformStatus}</div>
                  <div className="trace-meta">
                    <span>{UI_LABELS.hintPlatformStatus}</span>
                    <span>{liveEvent?.timestamp ? new Date(liveEvent.timestamp * 1000).toLocaleTimeString() : UI_LABELS.emptyValue}</span>
                  </div>
                  <div className="metrics-list">
                    <div className="metric-row">
                      <span>{UI_LABELS.fieldRobotPose}</span>
                      <strong>{formatPose(liveRobotPose)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>行为树阶段</span>
                      <strong>{activePhaseZone?.label ?? liveBtSnapshot?.activeSubtreeId ?? UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>布局预设</span>
                      <strong>{activeLayoutPreset?.label ?? UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>运动模式</span>
                      <strong>
                        {liveMotionMode
                          ? `${liveMotionMode.activeMode} / ${liveMotionMode.stopRequired ? '需停' : '可行'}`
                          : UI_LABELS.emptyValue}
                      </strong>
                    </div>
                    <div className="metric-row">
                      <span>跟踪状态</span>
                      <strong>{liveTracking?.status ?? UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>局部规划状态</span>
                      <strong>{livePlanner?.status ?? UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>恢复动作</span>
                      <strong>
                        {liveRecovery
                          ? `${liveRecovery.recoveryName} / ${liveRecovery.status}`
                          : UI_LABELS.emptyValue}
                      </strong>
                    </div>
                    <div className="metric-row">
                      <span>语义阻塞</span>
                      <strong>
                        {liveSemantic
                          ? `${liveSemantic.blockedCells} blocked / ${liveSemantic.slowCells} slow`
                          : UI_LABELS.emptyValue}
                      </strong>
                    </div>
                    <div className="metric-row">
                      <span>定位健康</span>
                      <strong>
                        {liveLocalizationHealth
                          ? `${formatLevel(liveLocalizationHealth.level)} / ${liveLocalizationHealth.localizationState}`
                          : UI_LABELS.emptyValue}
                      </strong>
                    </div>
                    <div className="metric-row">
                      <span>定位后端</span>
                      <strong>
                        {liveLocalizationBackend
                          ? `${liveLocalizationBackend.optimizerState} / 图健康 ${formatMetricValue(liveLocalizationBackend.graphHealth)}`
                          : UI_LABELS.emptyValue}
                      </strong>
                    </div>
                    <div className="metric-row">
                      <span>操作员状态</span>
                      <strong>
                        {liveOperatorStatus
                          ? `${formatLevel(liveOperatorStatus.overallLevel)} / ${liveOperatorStatus.overallReason || UI_LABELS.emptyValue}`
                          : UI_LABELS.emptyValue}
                      </strong>
                    </div>
                    <div className="metric-row">
                      <span>机构状态</span>
                      <strong>
                        {liveMechanismState
                          ? `tip ${liveMechanismState.tipState} / RTT ${formatMetricValue(liveMechanismState.avgRttMs)}`
                          : UI_LABELS.emptyValue}
                      </strong>
                    </div>
                    <div className="metric-row">
                      <span>活动边</span>
                      <strong>{liveEvent?.activeEdge || UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>语义门控</span>
                      <strong>{liveEvent?.gateStatus || UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>局部规划原因</span>
                      <strong>{livePlanner?.reason ?? UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>恢复原因</span>
                      <strong>{liveRecovery?.reason ?? UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>语义来源</span>
                      <strong>{formatStringList(liveSemantic?.activeSources)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>运行路径 UID</span>
                      <strong>{formatStringList(liveBtSnapshot?.runningPathUids.map((uid) => String(uid)))}</strong>
                    </div>
                    <div className="metric-row">
                      <span>活跃事件数</span>
                      <strong>{String(activeEventList.length)}</strong>
                    </div>
                  </div>
                </div>
              </section>

              <section className="panel-section">
                <h2>{UI_LABELS.panelEvents}</h2>
                <div className="trace-card">
                  <div className="trace-title">{UI_LABELS.panelEvents}</div>
                  <div className="trace-meta">
                    <span>聚合 `/r2/diag/events` 与 `/r2/bt/events` 的最近状态。</span>
                  </div>
                  <div className="metrics-list metrics-list-scroll">
                    {activeEventList.length === 0 && btEventList.length === 0 && (
                      <div className="empty-note">{UI_LABELS.hintEventsEmpty}</div>
                    )}
                    {activeEventList.map((event) => (
                      <div key={event.code} className="metric-row metric-row-stack">
                        <span>{`${event.title} / ${formatSeverity(event.severity)}`}</span>
                        <strong>{event.recommendation || event.detail || UI_LABELS.emptyValue}</strong>
                      </div>
                    ))}
                    {btEventList.map((event) => (
                      <div key={`${event.uid}-${event.nodeName}`} className="metric-row metric-row-stack">
                        <span>{`BT ${event.nodeName}`}</span>
                        <strong>{event.fullPath || UI_LABELS.emptyValue}</strong>
                      </div>
                    ))}
                  </div>
                </div>
              </section>

              <section className="panel-section">
                <h2>{UI_LABELS.panelLocalPlannerTrace}</h2>
                <div className="trace-card">
                  <div className="trace-title">局部规划离线案例</div>
                  <div className="trace-meta">
                    <span>{UI_LABELS.hintLocalPlannerTrace}</span>
                  </div>
                  <div className="button-row run-dock">
                    <select
                      className="secondary-button"
                      value={selectedLocalPlannerScenario}
                      onChange={(event) => setSelectedLocalPlannerScenario(event.target.value)}
                      disabled={localPlannerScenarios.length === 0 || isLocalPlannerTraceLoading}
                    >
                      {localPlannerScenarios.length === 0 && (
                        <option value="">暂无案例</option>
                      )}
                      {localPlannerScenarios.map((scenario) => (
                        <option key={scenario.name} value={scenario.name}>
                          {scenario.label}
                        </option>
                      ))}
                    </select>
                    <button
                      className="secondary-button"
                      type="button"
                      onClick={handleLoadLocalPlannerTrace}
                      disabled={!selectedLocalPlannerScenario || isLocalPlannerTraceLoading}
                    >
                      {isLocalPlannerTraceLoading
                        ? UI_LABELS.statusGenerating
                        : UI_LABELS.btnLoadLocalPlannerTrace}
                    </button>
                  </div>
                  <div className="metrics-list">
                    <div className="metric-row">
                      <span>案例标签</span>
                      <strong>{localPlannerTraceSummary?.snapshotLabel ?? UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>最终状态</span>
                      <strong>{localPlannerTraceSummary?.finalStatus ?? UI_LABELS.emptyValue}</strong>
                    </div>
                    <div className="metric-row">
                      <span>候选轨迹数</span>
                      <strong>
                        {localPlannerTraceSummary == null
                          ? UI_LABELS.emptyValue
                          : String(localPlannerTraceSummary.candidateCount)}
                      </strong>
                    </div>
                    <div className="metric-row">
                      <span>最终原因</span>
                      <strong>{localPlannerTraceSummary?.finalReason ?? UI_LABELS.emptyValue}</strong>
                    </div>
                  </div>
                </div>
              </section>

              <section className="panel-section">
                <h2>{UI_LABELS.panelTiming}</h2>
                <div className="trace-card">
                  <div className="trace-title">完整规划时间拆解</div>
                  <div className="empty-note">{UI_LABELS.hintTimingFormula}</div>
                  <div className="empty-note">{UI_LABELS.hintTimingDiagnostic}</div>
                  <div className="metrics-list">
                    <div className="metric-row">
                      <span>{UI_LABELS.statCompletePlanning}</span>
                      <strong>{formatElapsedMs(surfaceCompletePlanningMs)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>{UI_LABELS.statSurfaceProjection}</span>
                      <strong>{formatElapsedMs(surfaceProjectionMs)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>{UI_LABELS.statSurfacePlanning}</span>
                      <strong>{formatElapsedMs(surfacePlanningMs)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>{UI_LABELS.statPathExpand}</span>
                      <strong>{formatElapsedMs(surfacePathExpandMs)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>{UI_LABELS.statSegmentBuild}</span>
                      <strong>{formatElapsedMs(surfaceSegmentBuildMs)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>{UI_LABELS.statPreviewChain}</span>
                      <strong>{formatElapsedMs(previewChainMs)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>{UI_LABELS.statTracePlanning}</span>
                      <strong>{formatElapsedMs(tracePlanningMs)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>{UI_LABELS.statTraceChain}</span>
                      <strong>{formatElapsedMs(traceChainMs)}</strong>
                    </div>
                    <div className="metric-row">
                      <span>{UI_LABELS.statTotalElapsed}</span>
                      <strong>{formatElapsedMs(totalElapsedMs)}</strong>
                    </div>
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
                  <div className="metrics-list metrics-list-scroll">
                    {surfaceSegments.length === 0 && <div className="empty-note">{UI_LABELS.hintTraceEmpty}</div>}
                    {surfaceSegments.map((segment) => (
                      <div key={segment.segment_id} className="metric-row">
                        <span>{formatNodeTransitionLabel(segment.from_node_id, segment.to_node_id)}</span>
                        <strong>
                          {`${MOTION_TYPE_LABELS[segment.motion_type] ?? segment.motion_type}，${segment.point_count} 点`}
                        </strong>
                      </div>
                    ))}
                  </div>
                </div>
              </section>
            </div>
          </div>
        </section>
      </main>
    </div>
  );
}
