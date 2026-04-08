import { create } from 'zustand';

import { UI_LABELS } from './labels';
import type { DisplayLayerKey, SceneManifest, Team, ViewMode } from './types';

// Type alias keeps the store contract readable without duplicating manifest ids.
type LayerPreset = {
  id: string;
  visible_displays: DisplayLayerKey[];
};

export interface LayerState {
  scene: boolean;
  route: boolean;
  corridor: boolean;
  lookahead: boolean;
  robotPose: boolean;
  phaseZones: boolean;
  blocked: boolean;
  graph: boolean;
  keyNodes: boolean;
  openSet: boolean;
  expanded: boolean;
  tree: boolean;
  candidates: boolean;
  shadows: boolean;
}

interface SimState {
  team: Team;
  scene: SceneManifest | null;
  loadingScene: boolean;
  viewMode: ViewMode;
  layoutPresetId: string | null;
  layers: LayerState;
  statusMessage: string;
  setTeam: (team: Team) => void;
  setSceneLoading: (loading: boolean) => void;
  setScene: (scene: SceneManifest) => void;
  setViewMode: (viewMode: ViewMode) => void;
  applyLayoutPreset: (presetId: string) => void;
  toggleLayer: (layer: keyof LayerState) => void;
  setLayerVisible: (layer: keyof LayerState, visible: boolean) => void;
  setStatusMessage: (message: string) => void;
}

const LAYER_KEYS: Array<keyof LayerState> = [
  'scene',
  'route',
  'corridor',
  'lookahead',
  'robotPose',
  'phaseZones',
  'blocked',
  'graph',
  'keyNodes',
  'openSet',
  'expanded',
  'tree',
  'candidates',
  'shadows',
];

const defaultLayers: LayerState = {
  scene: true,
  route: true,
  corridor: true,
  lookahead: true,
  robotPose: true,
  phaseZones: true,
  blocked: true,
  graph: false,
  keyNodes: false,
  openSet: false,
  expanded: false,
  tree: false,
  candidates: false,
  shadows: true,
};

function resolvePreferredPreset(scene: SceneManifest, currentPresetId: string | null): string | null {
  const presets = scene.layoutPresets ?? [];
  if (presets.length === 0) {
    return null;
  }
  if (currentPresetId && presets.some((preset) => preset.id === currentPresetId)) {
    return currentPresetId;
  }
  if (presets.some((preset) => preset.id === 'operator')) {
    return 'operator';
  }
  return presets[0]?.id ?? null;
}

function buildLayerStateFromPreset(
  presets: LayerPreset[] | undefined,
  presetId: string | null,
  fallback: LayerState,
): LayerState {
  const preset = presets?.find((item) => item.id === presetId);
  if (!preset) {
    return { ...fallback };
  }

  const visible = new Set<string>(preset.visible_displays);
  const nextState = {} as LayerState;
  LAYER_KEYS.forEach((key) => {
    nextState[key] = visible.has(key);
  });
  return nextState;
}

export const useSimStore = create<SimState>((set) => ({
  team: 'blue',
  scene: null,
  loadingScene: false,
  viewMode: 'orbit',
  layoutPresetId: 'operator',
  layers: defaultLayers,
  statusMessage: UI_LABELS.statusWaiting,
  setTeam: (team) => set({ team }),
  setSceneLoading: (loadingScene) => set({ loadingScene }),
  setScene: (scene) =>
    set((state) => {
      const layoutPresetId = resolvePreferredPreset(scene, state.layoutPresetId);
      return {
        scene,
        loadingScene: false,
        layoutPresetId,
        layers: buildLayerStateFromPreset(scene.layoutPresets, layoutPresetId, state.layers),
        statusMessage: UI_LABELS.statusLoaded,
      };
    }),
  setViewMode: (viewMode) => set({ viewMode }),
  applyLayoutPreset: (presetId) =>
    set((state) => ({
      layoutPresetId: presetId,
      layers: buildLayerStateFromPreset(state.scene?.layoutPresets, presetId, state.layers),
    })),
  toggleLayer: (layer) =>
    set((state) => ({
      layers: {
        ...state.layers,
        [layer]: !state.layers[layer],
      },
    })),
  setLayerVisible: (layer, visible) =>
    set((state) => {
      if (state.layers[layer] === visible) {
        return state;
      }
      return {
        layers: {
          ...state.layers,
          [layer]: visible,
        },
      };
    }),
  setStatusMessage: (statusMessage) => set({ statusMessage }),
}));
