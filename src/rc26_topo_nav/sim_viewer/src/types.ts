export type Team = 'blue' | 'red';
export type Algorithm = 'astar' | 'rrt' | 'dwa';
export type RunMode = 'offline-sim' | 'live-ros';
export type GoalKind = 'node' | 'task' | 'route';
export type PickMode = 'idle' | 'start' | 'goal';
export type ViewMode = 'orbit' | 'follow' | 'first_person' | 'top_ortho' | 'side_ortho' | 'side_perspective';

export interface Pose3 {
  x: number;
  y: number;
  z: number;
  yaw: number;
  world_anchor_z?: number | null;
  world_z?: number;
}

export interface SceneFeature {
  id: string;
  name: string;
  material_symbol?: string;
  fill: string;
  opacity: number;
  render_class: string;
  avg_z: number;
  z_span: number;
  area_xy: number;
  points: Pose3[];
}

export interface GraphNode {
  id: string;
  type: string;
  block_id: number;
  base_cost: number;
  operation_tag: string;
  pose: Pose3;
}

export interface GraphEdge {
  id: string;
  from: string;
  to: string;
  motion_type: string;
  height_change: number;
  required_mode: string;
  base_cost: number;
  points: Pose3[];
}

export interface CameraPreset {
  id: ViewMode;
  kind: 'perspective' | 'orthographic';
  position: Pose3;
  target: Pose3;
}

export interface SceneManifest {
  meta: {
    team: Team;
    graph_file: string;
    world_file: string;
    kfs_config_file: string;
    full_geometry: boolean;
  };
  bounds: {
    min_x: number;
    max_x: number;
    min_y: number;
    max_y: number;
    min_z: number;
    max_z: number;
  };
  lights: {
    ambient: number[];
    background: number[];
    lights: Array<{
      name: string;
      type: string;
      pose: Pose3;
      direction: number[];
      diffuse: number[];
      cast_shadows: boolean;
    }>;
  };
  sceneFeatures: SceneFeature[];
  graphNodes: GraphNode[];
  graphEdges: GraphEdge[];
  tasks: Array<{ task_tag: string; candidate_nodes: string[] }>;
  routes: Array<{ route_tag: string; nodes: string[] }>;
  meilinSlots: Array<{ block_id: number; x: number; y: number; z: number }>;
  cameraPresets: CameraPreset[];
  defaults: {
    startNode: string;
    goalNode: string;
  };
}

export interface OpenSetEntry {
  nodeId: string;
  pose: Pose3;
  gCost: number;
  fCost: number;
}

export interface ExpandedNode {
  nodeId: string;
  pose: Pose3;
}

export interface CandidateTrajectory {
  velocity?: { vx: number; vy: number; wz: number };
  points: Pose3[];
  score?: number;
  collision?: boolean;
  selected?: boolean;
  clearance?: number;
}

export interface PlannerFrame {
  stepIndex: number;
  algorithm: Algorithm;
  phase: string;
  label: string;
  robotPose: Pose3 | null;
  openSet: OpenSetEntry[];
  expandedNodes: ExpandedNode[];
  bestPath: {
    nodeIds: string[];
    points: Pose3[];
  };
  treeSegments: Array<{ from: Pose3; to: Pose3 }>;
  candidateTrajectories: CandidateTrajectory[];
  selectedTrajectory: Pose3[];
  metrics: Record<string, string | number | boolean | null>;
}

export interface RunSummary {
  goalKind?: string;
  goalValue?: string;
  framesCount?: number;
  totalCost?: number;
  selectedCandidate?: string;
  candidateResults?: Array<Record<string, unknown>>;
  iterations?: number;
  tree_size?: number;
  goal_distance?: number | null;
  steps?: number;
}

export interface RunMetaMessage {
  type: 'meta';
  runId: string;
  state: string;
  cursor: number;
  frameCount: number;
  summary: RunSummary;
}

export interface RunFrameMessage {
  type: 'frame' | 'state';
  runId: string;
  state: string;
  cursor: number;
  frameCount?: number;
  summary?: RunSummary;
  frame?: PlannerFrame | null;
}

export interface LiveTrackingState {
  corridorId: string;
  edgeId: string;
  status: string;
  terminal: boolean;
  distanceToGoal: number;
  reason: string;
  cmd: {
    vx: number;
    vy: number;
    wz: number;
  };
}

export interface LiveEvent {
  type: 'live_state' | 'live_error';
  routePath?: Pose3[];
  corridorPath?: Pose3[];
  activeEdge?: string;
  gateStatus?: string;
  blockOverlay?: Array<{
    gridId: number;
    state: number;
    confidence: number;
    keepoutActive: boolean;
  }>;
  trackingState?: LiveTrackingState | null;
  message?: string;
  timestamp?: number;
}
