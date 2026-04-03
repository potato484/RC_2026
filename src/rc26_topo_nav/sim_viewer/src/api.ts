import type {
  Pose3,
  SceneManifest,
  SurfaceRouteExecuteResponse,
  SurfaceRoutePreviewResponse,
  SurfaceRouteTraceResponse,
  Team,
} from './types';

export interface SurfaceRoutePayload {
  team: Team;
  start_pick_world: Pose3;
  goal_pick_world: Pose3;
  projection_radius_m?: number;
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
      throw new Error(`${path} failed: ${response.status}`);
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
