import { create } from 'zustand';

import { UI_LABELS } from './labels';
import type { SceneManifest, Team, ViewMode } from './types';

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
  scene: SceneManifest | null;
  loadingScene: boolean;
  viewMode: ViewMode;
  layers: LayerState;
  statusMessage: string;
  setTeam: (team: Team) => void;
  setSceneLoading: (loading: boolean) => void;
  setScene: (scene: SceneManifest) => void;
  setViewMode: (viewMode: ViewMode) => void;
  toggleLayer: (layer: keyof LayerState) => void;
  setStatusMessage: (message: string) => void;
}

const defaultLayers: LayerState = {
  scene: true,
  graph: false,
  keyNodes: false,
  openSet: true,
  expanded: true,
  tree: false,
  candidates: false,
  shadows: true,
  blocked: false,
};

export const useSimStore = create<SimState>((set) => ({
  team: 'blue',
  scene: null,
  loadingScene: false,
  viewMode: 'orbit',
  layers: defaultLayers,
  statusMessage: UI_LABELS.statusWaiting,
  setTeam: (team) => set({ team }),
  setSceneLoading: (loadingScene) => set({ loadingScene }),
  setScene: (scene) =>
    set({
      scene,
      loadingScene: false,
      statusMessage: UI_LABELS.statusLoaded,
    }),
  setViewMode: (viewMode) => set({ viewMode }),
  toggleLayer: (layer) =>
    set((state) => ({
      layers: {
        ...state.layers,
        [layer]: !state.layers[layer],
      },
    })),
  setStatusMessage: (statusMessage) => set({ statusMessage }),
}));
