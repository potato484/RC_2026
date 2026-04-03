import { create } from 'zustand';

import { UI_LABELS } from './labels';
import type {
  Algorithm,
  GoalKind,
  LiveEvent,
  PlannerFrame,
  RunMetaMessage,
  RunMode,
  RunSummary,
  SceneManifest,
  Team,
  ViewMode,
} from './types';

export interface LayerState {
  scene: boolean;
  graph: boolean;
  keyNodes: boolean;
  openSet: boolean;
  expanded: boolean;
  tree: boolean;
  candidates: boolean;
  shadows: boolean;
  blocked: boolean;
}

interface SimState {
  team: Team;
  algorithm: Algorithm;
  mode: RunMode;
  goalKind: GoalKind;
  strictRuntime: boolean;
  animationSpeed: number;
  scene: SceneManifest | null;
  loadingScene: boolean;
  startNode: string;
  goalValue: string;
  blockedNodeText: string;
  viewMode: ViewMode;
  layers: LayerState;
  runId: string | null;
  runState: string;
  frameCount: number;
  cursor: number;
  currentFrame: PlannerFrame | null;
  runSummary: RunSummary | null;
  liveEvent: LiveEvent | null;
  statusMessage: string;
  setTeam: (team: Team) => void;
  setAlgorithm: (algorithm: Algorithm) => void;
  setMode: (mode: RunMode) => void;
  setGoalKind: (goalKind: GoalKind) => void;
  setStrictRuntime: (strictRuntime: boolean) => void;
  setAnimationSpeed: (speed: number) => void;
  setSceneLoading: (loading: boolean) => void;
  setScene: (scene: SceneManifest) => void;
  setStartNode: (startNode: string) => void;
  setGoalValue: (goalValue: string) => void;
  setBlockedNodeText: (value: string) => void;
  setViewMode: (viewMode: ViewMode) => void;
  toggleLayer: (layer: keyof LayerState) => void;
  setRunMeta: (
    runId: string,
    meta: Pick<RunMetaMessage, 'state' | 'cursor' | 'frameCount' | 'summary'>,
  ) => void;
  setRunFrame: (payload: { state: string; cursor: number; frameCount?: number; summary?: RunSummary; frame?: PlannerFrame | null }) => void;
  setLiveEvent: (event: LiveEvent) => void;
  setStatusMessage: (message: string) => void;
  resetRun: () => void;
}

const defaultLayers: LayerState = {
  scene: true,
  graph: false,
  keyNodes: false,
  openSet: false,
  expanded: false,
  tree: false,
  candidates: false,
  shadows: true,
  blocked: true,
};

export const useSimStore = create<SimState>((set) => ({
  team: 'blue',
  algorithm: 'astar',
  mode: 'offline-sim',
  goalKind: 'node',
  strictRuntime: true,
  animationSpeed: 1,
  scene: null,
  loadingScene: false,
  startNode: '',
  goalValue: '',
  blockedNodeText: '',
  viewMode: 'orbit',
  layers: defaultLayers,
  runId: null,
  runState: 'paused',
  frameCount: 0,
  cursor: 0,
  currentFrame: null,
  runSummary: null,
  liveEvent: null,
  statusMessage: UI_LABELS.statusWaiting,
  setTeam: (team) => set({ team }),
  setAlgorithm: (algorithm) => set({ algorithm }),
  setMode: (mode) => set({ mode }),
  setGoalKind: (goalKind) => set({ goalKind }),
  setStrictRuntime: (strictRuntime) => set({ strictRuntime }),
  setAnimationSpeed: (animationSpeed) => set({ animationSpeed }),
  setSceneLoading: (loadingScene) => set({ loadingScene }),
  setScene: (scene) =>
    set((state) => {
      const taskFallback = scene.tasks[0]?.task_tag ?? scene.defaults.goalNode;
      const routeFallback = scene.routes[0]?.route_tag ?? scene.defaults.goalNode;
      const goalValue =
        state.goalKind === 'task'
          ? taskFallback
          : state.goalKind === 'route'
            ? routeFallback
            : scene.defaults.goalNode;
      return {
        scene,
        loadingScene: false,
        startNode: state.startNode || scene.defaults.startNode,
        goalValue: state.goalValue || goalValue,
        statusMessage: UI_LABELS.statusLoaded,
      };
    }),
  setStartNode: (startNode) => set({ startNode }),
  setGoalValue: (goalValue) => set({ goalValue }),
  setBlockedNodeText: (blockedNodeText) => set({ blockedNodeText }),
  setViewMode: (viewMode) => set({ viewMode }),
  toggleLayer: (layer) =>
    set((state) => ({
      layers: {
        ...state.layers,
        [layer]: !state.layers[layer],
      },
    })),
  setRunMeta: (runId, meta) =>
    set({
      runId,
      runState: meta.state,
      frameCount: meta.frameCount,
      cursor: meta.cursor,
      runSummary: meta.summary,
      statusMessage: `手动离线运行已创建: ${runId.slice(0, 8)}，可播放回放`,
    }),
  setRunFrame: ({ state, cursor, frameCount, summary, frame }) =>
    set((store) => ({
      runState: state,
      cursor,
      frameCount: frameCount ?? store.frameCount,
      runSummary: summary ?? store.runSummary,
      currentFrame: frame ?? store.currentFrame,
    })),
  setLiveEvent: (liveEvent) =>
    set({
      liveEvent,
      statusMessage:
        liveEvent.type === 'live_error'
          ? liveEvent.message ?? '实时桥接失败'
          : `实时模式更新 @ ${new Date((liveEvent.timestamp ?? Date.now() / 1000) * 1000).toLocaleTimeString()}`,
    }),
  setStatusMessage: (statusMessage) => set({ statusMessage }),
  resetRun: () =>
    set({
      runId: null,
      runState: 'paused',
      frameCount: 0,
      cursor: 0,
      currentFrame: null,
      runSummary: null,
    }),
}));
