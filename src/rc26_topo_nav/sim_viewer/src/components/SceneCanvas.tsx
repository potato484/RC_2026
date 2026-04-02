import { CameraControls, GizmoHelper, GizmoViewport, Grid, Line, OrthographicCamera, PerspectiveCamera } from '@react-three/drei';
import { Canvas, useThree } from '@react-three/fiber';
import { useDeferredValue, useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';

import type { LayerState } from '../store';
import type { CameraPreset, LiveEvent, PlannerFrame, Pose3, SceneFeature, SceneManifest, ViewMode } from '../types';

interface SceneCanvasProps {
  scene: SceneManifest | null;
  frame: PlannerFrame | null;
  liveEvent: LiveEvent | null;
  viewMode: ViewMode;
  layers: LayerState;
  startPose: Pose3 | null;
  goalPose: Pose3 | null;
}

interface SceneContentProps extends Omit<SceneCanvasProps, 'scene'> {
  scene: SceneManifest;
}

function featureGeometry(feature: SceneFeature): THREE.BufferGeometry {
  const positions: number[] = [];
  const points = feature.points;
  for (let index = 1; index < points.length - 1; index += 1) {
    const left = points[0];
    const mid = points[index];
    const right = points[index + 1];
    positions.push(left.x, left.y, left.z);
    positions.push(mid.x, mid.y, mid.z);
    positions.push(right.x, right.y, right.z);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.computeVertexNormals();
  return geometry;
}

function FeatureMesh({ feature }: { feature: SceneFeature }) {
  const geometry = useMemo(() => featureGeometry(feature), [feature]);

  return (
    <mesh geometry={geometry} castShadow receiveShadow>
      <meshStandardMaterial
        color={feature.fill}
        transparent={feature.opacity < 0.99}
        opacity={feature.opacity}
        roughness={0.64}
        metalness={feature.render_class === 'world-platform' ? 0.12 : 0.04}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

function TubePath({
  points,
  color,
  radius,
  opacity = 1,
}: {
  points: Pose3[];
  color: string;
  radius: number;
  opacity?: number;
}) {
  const geometry = useMemo(() => {
    if (points.length < 2) {
      return null;
    }
    const curve = new THREE.CatmullRomCurve3(points.map((point) => new THREE.Vector3(point.x, point.y, point.world_z ?? point.z)));
    return new THREE.TubeGeometry(curve, Math.max(points.length * 8, 16), radius, 10, false);
  }, [points, radius]);

  if (geometry === null) {
    return null;
  }

  return (
    <mesh geometry={geometry} castShadow receiveShadow>
      <meshStandardMaterial color={color} transparent={opacity < 1} opacity={opacity} roughness={0.28} metalness={0.18} />
    </mesh>
  );
}

function RobotMarker({ pose, color }: { pose: Pose3; color: string }) {
  const groupRef = useRef<THREE.Group>(null);

  useEffect(() => {
    if (!groupRef.current) {
      return;
    }
    groupRef.current.position.set(pose.x, pose.y, pose.world_z ?? pose.z);
    groupRef.current.rotation.set(Math.PI / 2, 0, pose.yaw);
  }, [pose]);

  return (
    <group ref={groupRef}>
      <mesh castShadow>
        <coneGeometry args={[0.14, 0.38, 18]} />
        <meshStandardMaterial color={color} roughness={0.3} metalness={0.2} />
      </mesh>
      <mesh position={[0, 0, -0.18]} castShadow receiveShadow>
        <cylinderGeometry args={[0.11, 0.11, 0.14, 18]} />
        <meshStandardMaterial color="#f5f0e8" roughness={0.45} />
      </mesh>
    </group>
  );
}

function CameraRig({
  scene,
  robotPose,
  viewMode,
}: {
  scene: SceneManifest;
  robotPose: Pose3 | null;
  viewMode: ViewMode;
}) {
  const perspectiveRef = useRef<THREE.PerspectiveCamera>(null);
  const orthographicRef = useRef<THREE.OrthographicCamera>(null);
  const controlsRef = useRef<any>(null);
  const set = useThree((state) => state.set);
  const presets = useMemo(() => {
    return Object.fromEntries(scene.cameraPresets.map((preset) => [preset.id, preset])) as Record<ViewMode, CameraPreset>;
  }, [scene.cameraPresets]);

  useEffect(() => {
    const controls = controlsRef.current;
    if (!controls) {
      return;
    }

    if (viewMode === 'follow' && robotPose) {
      const targetX = robotPose.x;
      const targetY = robotPose.y;
      const targetZ = (robotPose.world_z ?? robotPose.z) + 0.12;
      const lookX = targetX - Math.cos(robotPose.yaw) * 1.35;
      const lookY = targetY - Math.sin(robotPose.yaw) * 1.35;
      const lookZ = targetZ + 0.68;
      if (perspectiveRef.current) {
        set({ camera: perspectiveRef.current });
      }
      controls.setLookAt(lookX, lookY, lookZ, targetX, targetY, targetZ, true);
      return;
    }

    if (viewMode === 'first_person' && robotPose) {
      const eyeX = robotPose.x;
      const eyeY = robotPose.y;
      const eyeZ = (robotPose.world_z ?? robotPose.z) + 0.42;
      const targetX = eyeX + Math.cos(robotPose.yaw) * 1.8;
      const targetY = eyeY + Math.sin(robotPose.yaw) * 1.8;
      const targetZ = eyeZ + 0.02;
      if (perspectiveRef.current) {
        set({ camera: perspectiveRef.current });
      }
      controls.setLookAt(eyeX, eyeY, eyeZ, targetX, targetY, targetZ, true);
      return;
    }

    const preset = presets[viewMode];
    if (!preset) {
      return;
    }
    if (preset.kind === 'orthographic' && orthographicRef.current) {
      set({ camera: orthographicRef.current });
    }
    if (preset.kind === 'perspective' && perspectiveRef.current) {
      set({ camera: perspectiveRef.current });
    }
    controls.setLookAt(
      preset.position.x,
      preset.position.y,
      preset.position.z,
      preset.target.x,
      preset.target.y,
      preset.target.z,
      true,
    );
  }, [presets, robotPose, set, viewMode]);

  return (
    <>
      <PerspectiveCamera ref={perspectiveRef} makeDefault={viewMode !== 'top_ortho' && viewMode !== 'side_ortho'} fov={48} near={0.05} far={240} />
      <OrthographicCamera
        ref={orthographicRef}
        makeDefault={viewMode === 'top_ortho' || viewMode === 'side_ortho'}
        near={-50}
        far={240}
        zoom={74}
      />
      <CameraControls
        ref={controlsRef}
        makeDefault
        smoothTime={0.25}
        mouseButtons={{
          left: 1,
          middle: 8,
          right: 2,
          wheel: 8,
        }}
      />
      <GizmoHelper alignment="bottom-right" margin={[84, 84]}>
        <GizmoViewport axisColors={['#d62828', '#2a9d8f', '#355070']} labelColor="#111827" />
      </GizmoHelper>
    </>
  );
}

function SceneContent({ scene, frame, liveEvent, viewMode, layers, startPose, goalPose }: SceneContentProps) {
  const deferredFrame = useDeferredValue(frame);
  const backgroundColor = new THREE.Color(scene.lights.background[0] / 255, scene.lights.background[1] / 255, scene.lights.background[2] / 255);

  const liveRoute = liveEvent?.routePath ?? [];
  const liveCorridor = liveEvent?.corridorPath ?? [];
  const offlinePath = deferredFrame?.bestPath.points ?? [];
  const pathGoal = offlinePath.length > 0 ? offlinePath[offlinePath.length - 1] : goalPose;

  const blockedSlots = useMemo(() => {
    const blockedGridIds = new Set((liveEvent?.blockOverlay ?? []).filter((cell) => cell.state === 1).map((cell) => cell.gridId));
    return scene.meilinSlots.filter((slot) => blockedGridIds.has(slot.block_id));
  }, [liveEvent?.blockOverlay, scene.meilinSlots]);

  return (
    <>
      <color attach="background" args={[backgroundColor]} />
      <fog attach="fog" args={['#d1d5db', 12, 34]} />
      <ambientLight intensity={0.8} />
      {scene.lights.lights.map((light, index) => (
        <directionalLight
          // eslint-disable-next-line react/no-array-index-key
          key={`${light.name}-${index}`}
          position={[light.pose.x, light.pose.y, light.pose.z]}
          intensity={1.6}
          color={new THREE.Color(light.diffuse[0] / 255, light.diffuse[1] / 255, light.diffuse[2] / 255)}
          castShadow={layers.shadows && light.cast_shadows}
          shadow-mapSize-width={2048}
          shadow-mapSize-height={2048}
          shadow-camera-near={0.1}
          shadow-camera-far={60}
          shadow-camera-left={-12}
          shadow-camera-right={12}
          shadow-camera-top={12}
          shadow-camera-bottom={-12}
        />
      ))}

      <CameraRig scene={scene} robotPose={deferredFrame?.robotPose ?? null} viewMode={viewMode} />
      <Grid
        args={[28, 28]}
        cellColor="#8ea6b4"
        sectionColor="#294451"
        position={[0, 0, -0.01]}
        fadeDistance={26}
        infiniteGrid
      />

      {layers.scene && scene.sceneFeatures.map((feature) => <FeatureMesh key={feature.id} feature={feature} />)}

      {layers.graph &&
        scene.graphEdges.map((edge) => (
          <Line
            key={edge.id}
            points={edge.points.map((point) => [point.x, point.y, point.world_z ?? point.z])}
            color="#5f7484"
            transparent
            opacity={0.42}
            lineWidth={1.1}
          />
        ))}

      {layers.graph &&
        scene.graphNodes.map((node) => (
          <mesh key={node.id} position={[node.pose.x, node.pose.y, node.pose.world_z ?? node.pose.z]} castShadow receiveShadow>
            <sphereGeometry args={[0.08, 16, 16]} />
            <meshStandardMaterial color={node.type.includes('staging') ? '#8ecae6' : '#f4a261'} />
          </mesh>
        ))}

      {layers.blocked &&
        blockedSlots.map((slot) => (
          <mesh key={`blocked-${slot.block_id}`} position={[slot.x, slot.y, slot.z + 0.22]} castShadow receiveShadow>
            <boxGeometry args={[0.72, 0.72, 0.42]} />
            <meshStandardMaterial color="#d62828" transparent opacity={0.58} />
          </mesh>
        ))}

      {startPose && (
        <mesh position={[startPose.x, startPose.y, startPose.world_z ?? startPose.z]} castShadow receiveShadow>
          <sphereGeometry args={[0.16, 20, 20]} />
          <meshStandardMaterial color="#2a9d8f" />
        </mesh>
      )}

      {pathGoal && (
        <mesh position={[pathGoal.x, pathGoal.y, pathGoal.world_z ?? pathGoal.z]} castShadow receiveShadow>
          <sphereGeometry args={[0.16, 20, 20]} />
          <meshStandardMaterial color="#d62828" />
        </mesh>
      )}

      {layers.keyNodes &&
        offlinePath.map((point, index) => (
          <mesh key={`path-node-${index}`} position={[point.x, point.y, point.world_z ?? point.z]} castShadow receiveShadow>
            <sphereGeometry args={[0.065, 12, 12]} />
            <meshStandardMaterial color="#fcbf49" />
          </mesh>
        ))}

      <TubePath points={offlinePath} color="#355070" radius={0.055} opacity={0.92} />
      <TubePath points={liveRoute} color="#0ea5e9" radius={0.05} opacity={0.82} />
      <TubePath points={liveCorridor} color="#fb8500" radius={0.04} opacity={0.82} />

      {layers.tree &&
        deferredFrame?.treeSegments.map((segment, index) => (
          <Line
            // eslint-disable-next-line react/no-array-index-key
            key={`tree-${index}`}
            points={[
              [segment.from.x, segment.from.y, segment.from.world_z ?? segment.from.z],
              [segment.to.x, segment.to.y, segment.to.world_z ?? segment.to.z],
            ]}
            color="#4ea8de"
            transparent
            opacity={0.42}
            lineWidth={1.2}
          />
        ))}

      {layers.candidates &&
        deferredFrame?.candidateTrajectories.map((trajectory, index) => (
          <Line
            // eslint-disable-next-line react/no-array-index-key
            key={`candidate-${index}`}
            points={trajectory.points.map((point) => [point.x, point.y, point.world_z ?? point.z])}
            color={trajectory.selected ? '#ff7b00' : trajectory.collision ? '#adb5bd' : '#94d2bd'}
            transparent
            opacity={trajectory.selected ? 0.95 : 0.34}
            lineWidth={trajectory.selected ? 2.2 : 1.1}
          />
        ))}

      {layers.openSet &&
        deferredFrame?.openSet.map((entry) => (
          <mesh key={`open-${entry.nodeId}`} position={[entry.pose.x, entry.pose.y, entry.pose.world_z ?? entry.pose.z]} castShadow receiveShadow>
            <sphereGeometry args={[0.072, 12, 12]} />
            <meshStandardMaterial color="#219ebc" transparent opacity={0.92} />
          </mesh>
        ))}

      {layers.expanded &&
        deferredFrame?.expandedNodes.map((entry) => (
          <mesh key={`expanded-${entry.nodeId}`} position={[entry.pose.x, entry.pose.y, entry.pose.world_z ?? entry.pose.z]} castShadow receiveShadow>
            <sphereGeometry args={[0.055, 10, 10]} />
            <meshStandardMaterial color="#ffb703" transparent opacity={0.72} />
          </mesh>
        ))}

      {deferredFrame?.robotPose && <RobotMarker pose={deferredFrame.robotPose} color="#111827" />}
    </>
  );
}

export function SceneCanvas(props: SceneCanvasProps) {
  const { scene } = props;

  if (!scene) {
    return <div className="canvas-placeholder">场景未加载，无法渲染 3D 视图。</div>;
  }

  return (
    <Canvas
      shadows
      gl={{ antialias: true }}
      dpr={[1, 2]}
      style={{ width: '100%', height: '100%' }}
    >
      <SceneContent {...props} scene={scene} />
    </Canvas>
  );
}
