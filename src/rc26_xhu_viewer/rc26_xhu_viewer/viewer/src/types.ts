export type Team = 'blue' | 'red';
export type Algorithm = 'astar' | 'rrt' | 'dwa' | 'local_planner';
export type PickMode = 'idle' | 'start' | 'goal' | 'surface_start' | 'surface_goal';
export type ViewMode = 'orbit' | 'follow' | 'first_person' | 'top_ortho' | 'side_ortho' | 'side_perspective';
export type DisplayLayerKey =
  | 'scene'
  | 'route'
  | 'corridor'
  | 'lookahead'
  | 'robotPose'
  | 'phaseZones'
  | 'blocked'
  | 'graph'
  | 'keyNodes'
  | 'openSet'
  | 'expanded'
  | 'tree'
  | 'candidates'
  | 'shadows';
export type DisplayTone = 'scene' | 'search' | 'path' | 'risk' | 'appearance' | 'state';

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

export interface ViewerMeta {
  schema_version?: string;
  viewer_title?: string;
  viewer_subtitle?: string;
}

export interface SemanticZone {
  id: string;
  label: string;
  phase_key: string;
  color: string;
  source: string;
  viewer_only: boolean;
  polygon: Pose3[];
}

export interface DisplayCatalogEntry {
  id: DisplayLayerKey;
  label: string;
  short_label: string;
  group: 'primary' | 'advanced';
  tone: DisplayTone;
}

export interface LayoutPreset {
  id: string;
  label: string;
  description: string;
  visible_displays: DisplayLayerKey[];
}

export interface SceneManifest {
  meta: {
    team: Team;
    graph_file: string;
    surface_graph_file?: string;
    world_file: string;
    kfs_config_file: string;
    full_geometry: boolean;
  };
  viewerMeta?: ViewerMeta;
  alignment?: Record<string, unknown> | null;
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
  semanticZones?: SemanticZone[];
  displayCatalog?: DisplayCatalogEntry[];
  layoutPresets?: LayoutPreset[];
  defaults: {
    startNode: string;
    goalNode: string;
  };
}

export interface OpenSetEntryRef {
  nodeId: string;
  gCost: number;
  fCost: number;
  pose?: Pose3;
}

export interface ExpandedNodeRef {
  nodeId: string;
  pose?: Pose3;
}

export interface CandidateTrajectory {
  velocity?: { vx: number; vy: number; wz: number };
  points: Pose3[];
  score?: number;
  collision?: boolean;
  selected?: boolean;
  clearance?: number;
}

export interface PlannerTraceFrame {
  stepIndex: number;
  algorithm: Algorithm;
  phase: string;
  label: string;
  robotPose: Pose3 | null;
  openSet: OpenSetEntryRef[];
  expandedNodes: ExpandedNodeRef[];
  bestPath: {
    nodeIds: string[];
    points?: Pose3[];
  };
  treeSegments: Array<{ from: Pose3; to: Pose3 }>;
  candidateTrajectories: CandidateTrajectory[];
  selectedTrajectory: Pose3[];
  metrics: Record<string, string | number | boolean | null>;
}

export interface OpenSetEntry extends OpenSetEntryRef {
  pose: Pose3;
}

export interface ExpandedNode extends ExpandedNodeRef {
  pose: Pose3;
}

export interface PlannerFrame extends Omit<PlannerTraceFrame, 'openSet' | 'expandedNodes' | 'bestPath'> {
  openSet: OpenSetEntry[];
  expandedNodes: ExpandedNode[];
  bestPath: {
    nodeIds: string[];
    points: Pose3[];
  };
}

export interface SurfaceRouteSegment {
  segment_id: string;
  from_node_id: string;
  to_node_id: string;
  motion_type: string;
  required_mode: string;
  point_count: number;
}

export interface PlanningLogField {
  label: string;
  value: string;
}

export interface PlanningLogEntry {
  stage: string;
  level: 'info' | 'warn' | 'error';
  title: string;
  message: string;
  elapsed_ms: number | null;
  fields: PlanningLogField[];
}

export interface RouteTraceSummary {
  goalKind?: string;
  goalValue?: string;
  framesCount?: number;
  returnedFramesCount?: number;
  framesSampled?: boolean;
  totalCost?: number | null;
  selectedCandidate?: string | null;
  candidateResults?: Array<Record<string, unknown>>;
  projectedStartNodeId?: string;
  projectedGoalNodeId?: string;
  requestedStart?: Pose3;
  requestedGoal?: Pose3;
  surfaceProjectionMs?: number | null;
  surfacePlanningMs?: number | null;
  surfacePathExpandMs?: number | null;
  surfaceSegmentBuildMs?: number | null;
  surfaceCompletePlanningMs?: number | null;
  tracePlanningMs?: number | null;
  previewElapsedMs?: number | null;
  traceElapsedMs?: number | null;
  totalElapsedMs?: number | null;
}

export interface SurfaceRoutePlanningTiming {
  surfaceProjection?: number;
  surfacePlanning?: number;
  surfacePathExpand?: number;
  surfaceSegmentBuild?: number;
  surfaceCompletePlanning?: number;
  surfaceRouteCli?: number;
  tracePlanning?: number;
  plannerTraceCli?: number;
  surfaceRouteTraceTotal?: number;
}

export interface SurfaceRoutePreviewResponse {
  success: boolean;
  failure_code: string;
  failure_reason: string;
  fallback_available?: boolean;
  fallback_planner_backend?: string;
  projected_start_node_id?: string;
  projected_goal_node_id?: string;
  projected_start: Pose3;
  projected_goal: Pose3;
  path_points: Pose3[];
  segments: SurfaceRouteSegment[];
  fallback_path_points?: Pose3[];
  fallback_segments?: SurfaceRouteSegment[];
  team?: Team;
  surface_graph_file?: string;
  planning_logs?: PlanningLogEntry[];
  planning_timing_ms?: SurfaceRoutePlanningTiming;
}

export interface SurfaceRouteTraceResponse extends SurfaceRoutePreviewResponse {
  summary: RouteTraceSummary;
  node_poses?: Record<string, Pose3>;
  frames: PlannerTraceFrame[];
}

export interface SurfaceRouteTraceFromNodesResponse {
  success: boolean;
  failure_code: string;
  failure_reason: string;
  projected_start_node_id?: string;
  projected_goal_node_id?: string;
  team?: Team;
  surface_graph_file?: string;
  planning_logs?: PlanningLogEntry[];
  planning_timing_ms?: SurfaceRoutePlanningTiming;
  summary: RouteTraceSummary;
  node_poses?: Record<string, Pose3>;
  frames: PlannerTraceFrame[];
}

export interface SurfaceRouteExecuteResponse {
  accepted: boolean;
  preview: SurfaceRoutePreviewResponse;
}

export interface LocalPlannerScenario {
  name: string;
  label: string;
  snapshot_file: string;
}

export interface LocalPlannerTraceResponse {
  success: boolean;
  snapshotLabel: string;
  snapshot_file?: string;
  traceMode: 'local_planner';
  result: {
    status: string;
    reason: string;
    hasSolution: boolean;
    blockedByKeepout: boolean;
    blockedByTerrain: boolean;
    shouldRotateRecovery: boolean;
    cmd: {
      vx: number;
      vy: number;
      wz: number;
    };
    bestScore: number;
    clearanceMarginM: number;
  };
  summary: {
    candidateCount: number;
    linearLimit: number;
    angularLimit: number;
    preferredLinearSpeed: number;
    currentPathDistance: number;
    goalHeadingError: number;
    semanticRevision: number;
    finalStatus: string;
    finalReason: string;
  };
  frames: PlannerTraceFrame[];
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

export interface LiveLocalPlannerState {
  corridorId: string;
  edgeId: string;
  status: string;
  terminal: boolean;
  observeOnly: boolean;
  semanticRevision: number;
  cmd: {
    vx: number;
    vy: number;
    wz: number;
  };
  bestScore: number;
  clearanceMarginM: number;
  reason: string;
}

export interface LiveRecoveryState {
  corridorId: string;
  edgeId: string;
  recoveryName: string;
  status: string;
  terminal: boolean;
  elapsedSec: number;
  reason: string;
}

export interface LiveSemanticSummary {
  revision: number;
  terrainAvailable: boolean;
  keepoutAvailable: boolean;
  blockedCells: number;
  slowCells: number;
  maxObstacleProbability: number;
  maxDropProbability: number;
  activeSources: string[];
  activeReasons: string[];
}

export interface LiveControlAxis {
  x: number;
  y: number;
  z: number;
}

export interface LiveControlState {
  pose: Pose3;
  linear: LiveControlAxis;
  angular: LiveControlAxis;
}

export interface LiveMotionModeState {
  activeMode: string;
  reason: string;
  stopRequired: boolean;
  timedOut: boolean;
  maxLinearSpeed: number;
  maxAngularSpeed: number;
}

export interface LiveLocalizationHealth {
  level: number;
  reason: string;
  localizationState: string;
  controlDegraded: boolean;
  sigmaXy: number;
  sigmaYaw: number;
}

export interface LiveLocalizationBackendStatus {
  optimizerReady: boolean;
  optimizerState: string;
  graphHealth: number;
  loopCandidateCount: number;
  acceptedLoopCount: number;
  acceptedAnchorCount: number;
  imuSpike: boolean;
}

export interface LiveOperatorStatus {
  overallLevel: number;
  overallReason: string;
  localizationLevel: number;
  localizationReason: string;
  controllerLevel: number;
  navSafetyLevel: number;
  terrainLevel: number;
  keepoutLevel: number;
  mechanismLevel: number;
  activeEventCodes: string[];
  topicTimeoutCount: number;
}

export interface LiveVisualizationEventItem {
  code: string;
  severity: number;
  title: string;
  detail: string;
  sourceSignal: string;
  recommendation: string;
  active: boolean;
}

export interface LiveMechanismState {
  tipState: number;
  halOpen: boolean;
  lockedTipSlot: number;
  assembledCount: number;
  lastErrorCode: number;
  cmdElapsedMs: number;
  ackTimeoutCount: number;
  reconnectCount: number;
  parseErrorCount: number;
  avgRttMs: number;
  commHealthLevel: number;
}

export interface LiveBtSnapshot {
  tickSeq: number;
  treeStatus: number;
  tickDurationMs: number;
  activeSubtreeId: string;
  runningPathUids: number[];
}

export interface LiveBtEventItem {
  uid: number;
  nodeName: string;
  fullPath: string;
  status: number;
  prevStatus: number;
}

export interface LiveEvent {
  type: 'live_state' | 'live_error';
  routePath?: Pose3[];
  corridorPath?: Pose3[];
  localPlannerPreviewPath?: Pose3[];
  controlState?: LiveControlState | null;
  activeEdge?: string;
  gateStatus?: string;
  blockOverlay?: Array<{
    gridId: number;
    state: number;
    confidence: number;
    keepoutActive: boolean;
  }>;
  motionModeState?: LiveMotionModeState | null;
  trackingState?: LiveTrackingState | null;
  localPlannerState?: LiveLocalPlannerState | null;
  recoveryState?: LiveRecoveryState | null;
  semanticSummary?: LiveSemanticSummary | null;
  localizationHealth?: LiveLocalizationHealth | null;
  localizationBackendStatus?: LiveLocalizationBackendStatus | null;
  operatorStatus?: LiveOperatorStatus | null;
  visualizationEvents?: LiveVisualizationEventItem[];
  mechanismState?: LiveMechanismState | null;
  btSnapshot?: LiveBtSnapshot | null;
  btEvents?: LiveBtEventItem[];
  message?: string;
  timestamp?: number;
}
