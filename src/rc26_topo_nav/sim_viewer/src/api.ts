import type { GoalKind, LiveEvent, PlannerFrame, RunFrameMessage, RunMetaMessage, RunSummary, SceneManifest, Team } from './types';

export interface CreateRunPayload {
  algorithm: 'astar' | 'rrt' | 'dwa';
  mode: 'offline-sim';
  team: Team;
  start_node: string;
  goal_node?: string;
  goal_task?: string;
  goal_route?: string;
  strict_runtime: boolean;
  animation_speed: number;
  blocked_nodes: string[];
  blocked_edges: string[];
}

export interface CreateRunResponse {
  runId: string;
  frameCount: number;
  summary: RunSummary;
  state: string;
}

const apiBase = '';

function websocketBase(path: string): string {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const isDev = window.location.port === '5176';
  if (isDev) {
    return `${protocol}//${window.location.hostname}:5176/ws${path}`;
  }
  return `${protocol}//${window.location.host}${path}`;
}

export async function fetchSceneManifest(team: Team): Promise<SceneManifest> {
  const response = await fetch(`${apiBase}/api/scene-manifest?team=${team}&full_geometry=true`);
  if (!response.ok) {
    throw new Error(`Failed to load scene manifest: ${response.status}`);
  }
  return response.json();
}

export async function createRun(payload: CreateRunPayload): Promise<CreateRunResponse> {
  const response = await fetch(`${apiBase}/api/runs`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(payload),
  });
  if (!response.ok) {
    throw new Error(`Failed to create run: ${response.status}`);
  }
  return response.json();
}

export async function controlRun(
  runId: string,
  action: 'play' | 'pause' | 'step' | 'reset' | 'seek',
  cursor?: number,
  speed?: number,
): Promise<void> {
  const response = await fetch(`${apiBase}/api/runs/${runId}/control`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ action, cursor, speed }),
  });
  if (!response.ok) {
    throw new Error(`Failed to control run: ${response.status}`);
  }
}

export async function startLive(namespace = ''): Promise<void> {
  const response = await fetch(`${apiBase}/api/live/start`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ namespace }),
  });
  if (!response.ok) {
    throw new Error(`Failed to start live mode: ${response.status}`);
  }
}

export function openRunSocket(
  runId: string,
  handlers: {
    onMeta: (message: RunMetaMessage) => void;
    onFrame: (message: RunFrameMessage) => void;
    onError: (message: string) => void;
  },
): WebSocket {
  const socket = new WebSocket(websocketBase(`/api/runs/${runId}/events`));
  socket.addEventListener('message', (event) => {
    const payload = JSON.parse(event.data) as RunMetaMessage | RunFrameMessage;
    if (payload.type === 'meta') {
      handlers.onMeta(payload);
      return;
    }
    handlers.onFrame(payload);
  });
  socket.addEventListener('error', () => handlers.onError('Run socket error'));
  return socket;
}

export function openLiveSocket(
  handlers: {
    onEvent: (message: LiveEvent) => void;
    onError: (message: string) => void;
  },
): WebSocket {
  const socket = new WebSocket(websocketBase('/api/live/events'));
  socket.addEventListener('message', (event) => {
    handlers.onEvent(JSON.parse(event.data) as LiveEvent);
  });
  socket.addEventListener('error', () => handlers.onError('Live socket error'));
  return socket;
}

export function mapGoalPayload(goalKind: GoalKind, goalValue: string): Pick<CreateRunPayload, 'goal_node' | 'goal_task' | 'goal_route'> {
  if (goalKind === 'task') {
    return { goal_task: goalValue };
  }
  if (goalKind === 'route') {
    return { goal_route: goalValue };
  }
  return { goal_node: goalValue };
}

export function finalGoalPoint(frame: PlannerFrame | null): { x: number; y: number; z: number; yaw: number } | null {
  const points = frame?.bestPath.points ?? [];
  return points.length > 0 ? points[points.length - 1] : null;
}
