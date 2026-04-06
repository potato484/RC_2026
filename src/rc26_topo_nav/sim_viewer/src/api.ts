import type {
  LocalPlannerScenario,
  LocalPlannerTraceResponse,
  Pose3,
  SceneManifest,
  SurfaceRouteExecuteResponse,
  SurfaceRoutePreviewResponse,
  SurfaceRouteTraceFromNodesResponse,
  SurfaceRouteTraceResponse,
  Team,
} from './types';

export interface SurfaceRoutePayload {
  team: Team;
  start_pick_world: Pose3;
  goal_pick_world: Pose3;
  projection_radius_m?: number;
}

export interface SurfaceRouteTraceFromNodesPayload {
  team: Team;
  start_node_id: string;
  goal_node_id: string;
  surface_graph_file?: string;
  requested_start?: Pose3;
  requested_goal?: Pose3;
}

export interface LocalPlannerTracePayload {
  scenario_name?: string;
  snapshot_file?: string;
}

const apiBase = (import.meta.env.VITE_API_BASE_URL ?? '').trim().replace(/\/$/, '');

function apiUrl(path: string): string {
  return `${apiBase}${path}`;
}

async function fetchJson<T>(path: string, init?: RequestInit, timeoutMs = 30000): Promise<T> {
  const controller = new AbortController();
  const timer = window.setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetch(apiUrl(path), {
      ...init,
      signal: controller.signal,
    });
    if (!response.ok) {
      throw new Error(`接口请求失败，状态码 ${response.status}`);
    }
    return (await response.json()) as T;
  } catch (error) {
    if (error instanceof DOMException && error.name === 'AbortError') {
      throw new Error(`请求超时（>${Math.round(timeoutMs / 1000)} 秒）`);
    }
    throw error;
  } finally {
    window.clearTimeout(timer);
  }
}

export async function fetchSceneManifest(team: Team): Promise<SceneManifest> {
  return fetchJson<SceneManifest>(`/api/scene-manifest?team=${team}&full_geometry=true`, undefined, 15000);
}

export async function startLiveBridge(namespace = ''): Promise<{ status: string }> {
  return fetchJson<{ status: string }>(
    '/api/live/start',
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ namespace }),
    },
    15000
  );
}

export async function fetchLocalPlannerScenarios(): Promise<LocalPlannerScenario[]> {
  const response = await fetchJson<{ scenarios: LocalPlannerScenario[] }>('/api/local-planner/scenarios', undefined, 15000);
  return response.scenarios;
}

export async function traceLocalPlannerScenario(
  payload: LocalPlannerTracePayload,
): Promise<LocalPlannerTraceResponse> {
  return fetchJson<LocalPlannerTraceResponse>(
    '/api/local-planner/trace',
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
    },
    20000
  );
}

export async function previewSurfaceRoute(payload: SurfaceRoutePayload): Promise<SurfaceRoutePreviewResponse> {
  return fetchJson<SurfaceRoutePreviewResponse>(
    '/api/surface-route/preview',
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
    },
    15000
  );
}

export async function traceSurfaceRoute(payload: SurfaceRoutePayload): Promise<SurfaceRouteTraceResponse> {
  return fetchJson<SurfaceRouteTraceResponse>(
    '/api/surface-route/trace',
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
    },
    30000
  );
}

export async function traceSurfaceRouteFromNodes(
  payload: SurfaceRouteTraceFromNodesPayload,
): Promise<SurfaceRouteTraceFromNodesResponse> {
  return fetchJson<SurfaceRouteTraceFromNodesResponse>(
    '/api/surface-route/trace-from-nodes',
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
    },
    30000
  );
}

export async function executeSurfaceRoute(payload: SurfaceRoutePayload): Promise<SurfaceRouteExecuteResponse> {
  return fetchJson<SurfaceRouteExecuteResponse>(
    '/api/surface-route/execute',
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
    },
    15000
  );
}
