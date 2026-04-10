import { formatFailureSummary, UI_LABELS } from '../../labels';
import type {
  PlanningLogEntry,
  Pose3,
  RouteTraceSummary,
  SurfaceRoutePlanningTiming,
  SurfaceRoutePreviewResponse,
  SurfaceRouteSegment,
} from '../../types';

export function formatPose(pose: Pose3 | null): string {
  if (!pose) {
    return UI_LABELS.emptyValue;
  }
  return `${pose.x.toFixed(2)}，${pose.y.toFixed(2)}，${pose.z.toFixed(2)}`;
}

export function formatMetricValue(value: unknown): string {
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

export function formatStringList(values: string[] | undefined): string {
  if (!values || values.length === 0) {
    return UI_LABELS.emptyValue;
  }
  return values.join('，');
}

export function formatElapsedMs(value: number | null | undefined): string {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    return UI_LABELS.emptyValue;
  }
  return `${value.toFixed(2)} 毫秒`;
}

export function formatLevel(level: number | null | undefined): string {
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

export function formatSeverity(severity: number | null | undefined): string {
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

export function resolveTimingMs(...values: Array<number | null | undefined>): number | null {
  for (const value of values) {
    if (typeof value === 'number' && Number.isFinite(value)) {
      return value;
    }
  }
  return null;
}

export function formatPlanningLogLevel(level: PlanningLogEntry['level']): string {
  if (level === 'error') {
    return '失败';
  }
  if (level === 'warn') {
    return '告警';
  }
  return '完成';
}

export function buildPreviewSummary(
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

export function previewDisplayPath(response: SurfaceRoutePreviewResponse): Pose3[] {
  if (response.success) {
    return response.path_points;
  }
  return response.fallback_path_points ?? [];
}

export function previewDisplaySegments(response: SurfaceRoutePreviewResponse): SurfaceRouteSegment[] {
  if (response.success) {
    return response.segments;
  }
  return response.fallback_segments ?? [];
}

export function previewHasReferenceRoute(response: SurfaceRoutePreviewResponse): boolean {
  return !response.success && previewDisplayPath(response).length > 0;
}

export function formatPreviewResultLabel(response: SurfaceRoutePreviewResponse): string {
  if (response.success) {
    return `三维路线已生成，共 ${response.segments.length} 段`;
  }

  const failureSummary = formatFailureSummary(response.failure_reason, response.failure_code);
  if (previewHasReferenceRoute(response)) {
    const backendLabel = response.fallback_planner_backend?.trim() || 'legacy';
    return `三维路线未通过车体约束: ${failureSummary}，已显示 ${backendLabel} 参考路线`;
  }
  return `三维路线生成失败: ${failureSummary}`;
}

export function mergePlanningTiming(
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
