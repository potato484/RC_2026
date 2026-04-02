import { startTransition, useEffect, useMemo, useRef } from 'react';
import clsx from 'clsx';

import { createRun, controlRun, fetchSceneManifest, mapGoalPayload, openLiveSocket, openRunSocket, startLive } from './api';
import { SceneCanvas } from './components/SceneCanvas';
import { useSimStore } from './store';
import type { GoalKind, GraphNode, RunFrameMessage, RunMetaMessage, Team } from './types';

function nodeById(nodes: GraphNode[], nodeId: string): GraphNode | undefined {
  return nodes.find((node) => node.id === nodeId);
}

function parseBlockedNodes(value: string): string[] {
  return value
    .split(',')
    .map((item) => item.trim())
    .filter(Boolean);
}

function GoalSelector({
  goalKind,
  goalValue,
  setGoalKind,
  setGoalValue,
}: {
  goalKind: GoalKind;
  goalValue: string;
  setGoalKind: (value: GoalKind) => void;
  setGoalValue: (value: string) => void;
}) {
  const scene = useSimStore((state) => state.scene);

  const goalOptions = useMemo(() => {
    if (!scene) {
      return [] as Array<{ value: string; label: string }>;
    }
    if (goalKind === 'task') {
      return scene.tasks.map((task) => ({ value: task.task_tag, label: task.task_tag }));
    }
    if (goalKind === 'route') {
      return scene.routes.map((route) => ({ value: route.route_tag, label: route.route_tag }));
    }
    return scene.graphNodes.map((node) => ({ value: node.id, label: `${node.id} · ${node.type}` }));
  }, [goalKind, scene]);

  useEffect(() => {
    if (!scene) {
      return;
    }
    if (goalKind === 'node' && !goalOptions.some((option) => option.value === goalValue)) {
      setGoalValue(scene.defaults.goalNode);
      return;
    }
    if (goalKind === 'task' && goalOptions.length > 0 && !goalOptions.some((option) => option.value === goalValue)) {
      setGoalValue(goalOptions[0].value);
      return;
    }
    if (goalKind === 'route' && goalOptions.length > 0 && !goalOptions.some((option) => option.value === goalValue)) {
      setGoalValue(goalOptions[0].value);
    }
  }, [goalKind, goalOptions, goalValue, scene, setGoalValue]);

  return (
    <>
      <label className="field-label" htmlFor="goal-kind">
        目标类型
      </label>
      <select id="goal-kind" className="field-input" value={goalKind} onChange={(event) => setGoalKind(event.target.value as GoalKind)}>
        <option value="node">导航点</option>
        <option value="task">任务</option>
        <option value="route">预设路线</option>
      </select>

      <label className="field-label" htmlFor="goal-value">
        目标值
      </label>
      <select id="goal-value" className="field-input" value={goalValue} onChange={(event) => setGoalValue(event.target.value)}>
        {goalOptions.map((option) => (
          <option key={option.value} value={option.value}>
            {option.label}
          </option>
        ))}
      </select>
    </>
  );
}

export default function App() {
  const scene = useSimStore((state) => state.scene);
  const loadingScene = useSimStore((state) => state.loadingScene);
  const team = useSimStore((state) => state.team);
  const algorithm = useSimStore((state) => state.algorithm);
  const mode = useSimStore((state) => state.mode);
  const goalKind = useSimStore((state) => state.goalKind);
  const strictRuntime = useSimStore((state) => state.strictRuntime);
  const animationSpeed = useSimStore((state) => state.animationSpeed);
  const startNode = useSimStore((state) => state.startNode);
  const goalValue = useSimStore((state) => state.goalValue);
  const blockedNodeText = useSimStore((state) => state.blockedNodeText);
  const viewMode = useSimStore((state) => state.viewMode);
  const layers = useSimStore((state) => state.layers);
  const runId = useSimStore((state) => state.runId);
  const runState = useSimStore((state) => state.runState);
  const cursor = useSimStore((state) => state.cursor);
  const frameCount = useSimStore((state) => state.frameCount);
  const currentFrame = useSimStore((state) => state.currentFrame);
  const runSummary = useSimStore((state) => state.runSummary);
  const liveEvent = useSimStore((state) => state.liveEvent);
  const statusMessage = useSimStore((state) => state.statusMessage);
  const setTeam = useSimStore((state) => state.setTeam);
  const setAlgorithm = useSimStore((state) => state.setAlgorithm);
  const setMode = useSimStore((state) => state.setMode);
  const setGoalKind = useSimStore((state) => state.setGoalKind);
  const setStrictRuntime = useSimStore((state) => state.setStrictRuntime);
  const setAnimationSpeed = useSimStore((state) => state.setAnimationSpeed);
  const setSceneLoading = useSimStore((state) => state.setSceneLoading);
  const setScene = useSimStore((state) => state.setScene);
  const setStartNode = useSimStore((state) => state.setStartNode);
  const setGoalValue = useSimStore((state) => state.setGoalValue);
  const setBlockedNodeText = useSimStore((state) => state.setBlockedNodeText);
  const setViewMode = useSimStore((state) => state.setViewMode);
  const toggleLayer = useSimStore((state) => state.toggleLayer);
  const setRunMeta = useSimStore((state) => state.setRunMeta);
  const setRunFrame = useSimStore((state) => state.setRunFrame);
  const setLiveEvent = useSimStore((state) => state.setLiveEvent);
  const setStatusMessage = useSimStore((state) => state.setStatusMessage);
  const resetRun = useSimStore((state) => state.resetRun);

  const runSocketRef = useRef<WebSocket | null>(null);
  const liveSocketRef = useRef<WebSocket | null>(null);

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
        setStatusMessage(`场景加载失败: ${String(error)}`);
      });
    return () => {
      active = false;
    };
  }, [setScene, setSceneLoading, setStatusMessage, team]);

  useEffect(() => {
    return () => {
      runSocketRef.current?.close();
      liveSocketRef.current?.close();
    };
  }, []);

  const startPose = scene ? nodeById(scene.graphNodes, startNode)?.pose ?? null : null;
  const goalPose =
    scene && goalKind === 'node'
      ? nodeById(scene.graphNodes, goalValue)?.pose ?? null
      : (currentFrame && currentFrame.bestPath.points.length > 0
          ? currentFrame.bestPath.points[currentFrame.bestPath.points.length - 1]
          : null);

  async function handleCreateRun() {
    if (!scene) {
      return;
    }
    runSocketRef.current?.close();
    resetRun();
    const blockedNodes = parseBlockedNodes(blockedNodeText);
    try {
      const response = await createRun({
        algorithm,
        mode: 'offline-sim',
        team,
        start_node: startNode,
        strict_runtime: strictRuntime,
        animation_speed: animationSpeed,
        blocked_nodes: blockedNodes,
        blocked_edges: [],
        ...mapGoalPayload(goalKind, goalValue),
      });
      const socket = openRunSocket(response.runId, {
        onMeta: (message: RunMetaMessage) => {
          setRunMeta(response.runId, message);
        },
        onFrame: (message: RunFrameMessage) => {
          setRunFrame({
            state: message.state,
            cursor: message.cursor,
            frameCount: message.frameCount,
            summary: message.summary,
            frame: message.frame,
          });
        },
        onError: (message: string) => setStatusMessage(message),
      });
      runSocketRef.current = socket;
    } catch (error) {
      setStatusMessage(`创建运行失败: ${String(error)}`);
    }
  }

  async function handleRunControl(action: 'play' | 'pause' | 'step' | 'reset') {
    if (!runId) {
      return;
    }
    try {
      await controlRun(runId, action, undefined, animationSpeed);
    } catch (error) {
      setStatusMessage(`运行控制失败: ${String(error)}`);
    }
  }

  async function handleLiveStart() {
    liveSocketRef.current?.close();
    try {
      await startLive();
      liveSocketRef.current = openLiveSocket({
        onEvent: (event) => setLiveEvent(event),
        onError: (message: string) => setStatusMessage(message),
      });
      setStatusMessage('实时 ROS 只读桥接已启动');
    } catch (error) {
      setStatusMessage(`实时桥接启动失败: ${String(error)}`);
    }
  }

  const layerKeys = Object.keys(layers) as Array<keyof typeof layers>;
  const viewModes: Array<{ id: typeof viewMode; label: string }> = [
    { id: 'orbit', label: 'Orbit' },
    { id: 'follow', label: 'Follow' },
    { id: 'first_person', label: 'First Person' },
    { id: 'top_ortho', label: 'Top Ortho' },
    { id: 'side_ortho', label: 'Side Ortho' },
  ];

  return (
    <div className="app-shell">
      <div className="background-glow background-glow-left" />
      <div className="background-glow background-glow-right" />

      <header className="topbar">
        <div>
          <p className="eyebrow">RC26 TOPO NAV</p>
          <h1 className="title">Path Planning 3D Simulator</h1>
          <p className="subtitle">完整 mesh 场地、真实路径颜色层、轨道相机与 A* / RRT / DWA 仿真共用一套 WebGL 观察面。</p>
        </div>
        <div className="status-pill">
          <span className="status-dot" />
          {statusMessage}
        </div>
      </header>

      <main className="workspace-grid">
        <aside className="panel panel-left">
          <section className="panel-section">
            <h2>运行配置</h2>
            <label className="field-label" htmlFor="team-select">
              阵营
            </label>
            <select id="team-select" className="field-input" value={team} onChange={(event) => setTeam(event.target.value as Team)}>
              <option value="blue">Blue</option>
              <option value="red">Red</option>
            </select>

            <label className="field-label" htmlFor="mode-select">
              模式
            </label>
            <select id="mode-select" className="field-input" value={mode} onChange={(event) => setMode(event.target.value as typeof mode)}>
              <option value="offline-sim">Offline Simulation</option>
              <option value="live-ros">Live ROS</option>
            </select>

            <label className="field-label" htmlFor="algorithm-select">
              算法
            </label>
            <select id="algorithm-select" className="field-input" value={algorithm} onChange={(event) => setAlgorithm(event.target.value as typeof algorithm)}>
              <option value="astar">A* / Runtime Trace</option>
              <option value="rrt">RRT</option>
              <option value="dwa">Holonomic DWA</option>
            </select>

            <label className="field-label" htmlFor="start-node">
              起点节点
            </label>
            <select id="start-node" className="field-input" value={startNode} onChange={(event) => setStartNode(event.target.value)}>
              {scene?.graphNodes.map((node) => (
                <option key={node.id} value={node.id}>
                  {node.id} · {node.type}
                </option>
              ))}
            </select>

            <GoalSelector goalKind={goalKind} goalValue={goalValue} setGoalKind={setGoalKind} setGoalValue={setGoalValue} />

            <label className="field-label" htmlFor="blocked-nodes">
              阻塞节点
            </label>
            <input
              id="blocked-nodes"
              className="field-input"
              value={blockedNodeText}
              onChange={(event) => setBlockedNodeText(event.target.value)}
              placeholder="例如 mf_b4,mf_b5"
            />

            <label className="toggle-row">
              <input type="checkbox" checked={strictRuntime} onChange={(event) => setStrictRuntime(event.target.checked)} />
              <span>A* 复用运行时真逻辑</span>
            </label>

            <label className="field-label" htmlFor="speed">
              播放倍率
            </label>
            <input
              id="speed"
              className="range-input"
              type="range"
              min={0.5}
              max={4}
              step={0.5}
              value={animationSpeed}
              onChange={(event) => setAnimationSpeed(Number(event.target.value))}
            />
            <div className="range-readout">{animationSpeed.toFixed(1)}x</div>

            <div className="button-row">
              <button className="primary-button" type="button" onClick={handleCreateRun} disabled={!scene || loadingScene || mode !== 'offline-sim'}>
                生成离线 run
              </button>
              <button className="secondary-button" type="button" onClick={handleLiveStart} disabled={mode !== 'live-ros'}>
                启动实时桥接
              </button>
            </div>
          </section>

          <section className="panel-section">
            <h2>视图模式</h2>
            <div className="button-grid">
              {viewModes.map((item) => (
                <button
                  key={item.id}
                  className={clsx('mode-button', viewMode === item.id && 'mode-button-active')}
                  type="button"
                  onClick={() => setViewMode(item.id)}
                >
                  {item.label}
                </button>
              ))}
            </div>
          </section>

          <section className="panel-section">
            <h2>图层开关</h2>
            <div className="layer-grid">
              {layerKeys.map((layer) => (
                <label key={layer} className="toggle-row">
                  <input type="checkbox" checked={layers[layer]} onChange={() => toggleLayer(layer)} />
                  <span>{layer}</span>
                </label>
              ))}
            </div>
          </section>
        </aside>

        <section className="panel panel-canvas">
          <div className="canvas-toolbar">
            <div className="legend">
              <span className="legend-chip legend-start">Start</span>
              <span className="legend-chip legend-goal">Goal</span>
              <span className="legend-chip legend-path">Path</span>
              <span className="legend-chip legend-open">Open Set</span>
              <span className="legend-chip legend-tree">Tree / Candidates</span>
            </div>
            <div className="button-row">
              <button className="secondary-button" type="button" onClick={() => handleRunControl(runState === 'playing' ? 'pause' : 'play')} disabled={!runId}>
                {runState === 'playing' ? '暂停' : '播放'}
              </button>
              <button className="secondary-button" type="button" onClick={() => handleRunControl('step')} disabled={!runId}>
                单步
              </button>
              <button className="secondary-button" type="button" onClick={() => handleRunControl('reset')} disabled={!runId}>
                重置
              </button>
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
            />
          </div>
        </section>

        <aside className="panel panel-right">
          <section className="panel-section">
            <h2>运行状态</h2>
            <div className="stats-grid">
              <div className="stat-card">
                <span className="stat-label">Run ID</span>
                <span className="stat-value">{runId ? runId.slice(0, 8) : 'N/A'}</span>
              </div>
              <div className="stat-card">
                <span className="stat-label">Cursor</span>
                <span className="stat-value">
                  {cursor} / {Math.max(frameCount - 1, 0)}
                </span>
              </div>
              <div className="stat-card">
                <span className="stat-label">State</span>
                <span className="stat-value">{runState}</span>
              </div>
              <div className="stat-card">
                <span className="stat-label">Scene Faces</span>
                <span className="stat-value">{scene?.sceneFeatures.length ?? 0}</span>
              </div>
            </div>
          </section>

          <section className="panel-section">
            <h2>当前帧</h2>
            <div className="trace-card">
              <div className="trace-title">{currentFrame?.label ?? '尚未开始播放'}</div>
              <div className="trace-meta">
                <span>{currentFrame?.algorithm ?? algorithm}</span>
                <span>{currentFrame?.phase ?? 'idle'}</span>
              </div>
              <div className="metrics-list">
                {Object.entries(currentFrame?.metrics ?? {}).map(([key, value]) => (
                  <div key={key} className="metric-row">
                    <span>{key}</span>
                    <strong>{String(value)}</strong>
                  </div>
                ))}
              </div>
            </div>
          </section>

          <section className="panel-section">
            <h2>摘要</h2>
            <div className="metrics-list">
              {Object.entries(runSummary ?? {}).map(([key, value]) => (
                <div key={key} className="metric-row">
                  <span>{key}</span>
                  <strong>{typeof value === 'object' ? JSON.stringify(value) : String(value)}</strong>
                </div>
              ))}
            </div>
          </section>

          <section className="panel-section">
            <h2>实时桥接</h2>
            <div className="metrics-list">
              <div className="metric-row">
                <span>Active Edge</span>
                <strong>{liveEvent?.activeEdge ?? 'N/A'}</strong>
              </div>
              <div className="metric-row">
                <span>Gate</span>
                <strong>{liveEvent?.gateStatus ?? 'N/A'}</strong>
              </div>
              <div className="metric-row">
                <span>Corridor</span>
                <strong>{liveEvent?.trackingState?.corridorId ?? 'N/A'}</strong>
              </div>
              <div className="metric-row">
                <span>Distance</span>
                <strong>{liveEvent?.trackingState?.distanceToGoal?.toFixed?.(3) ?? 'N/A'}</strong>
              </div>
            </div>
          </section>
        </aside>
      </main>
    </div>
  );
}
