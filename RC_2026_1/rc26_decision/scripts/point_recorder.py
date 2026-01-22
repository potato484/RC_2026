#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import yaml
import os

class PointRecorder(Node):
    def __init__(self):
        super().__init__('point_recorder')
        self.declare_parameter('team', 'red')
        self.declare_parameter('output_dir', '~/RC_2026/.rc26/waypoints')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.team = self.get_parameter('team').value
        self.output_path = os.path.expanduser(
            f"{self.get_parameter('output_dir').value}/waypoints_{self.team}.yaml"
        )
        self.data = self._load_or_init()

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return {
                'x': t.transform.translation.x,
                'y': t.transform.translation.y,
                'theta': self._quat_to_yaw(t.transform.rotation)
            }
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

    def cmd_record(self, name, strategy='default', safety='NORMAL'):
        pose = self.get_robot_pose()
        if pose:
            self.data['static_points'][name] = {
                'x': pose['x'],
                'y': pose['y'],
                'theta': pose['theta'],
                'nav_mode': 'Normal',      # 可扩展为命令行参数
                'speed_mps': 1.5,          # 可扩展为命令行参数
                'strategy_tag': f'TAG_{strategy.upper()}',
                'nav_safety_mode': safety,
                'speed_profile': 'FAST',
                'tolerance': {'xy': 0.10, 'yaw': 0.15},
                'timeout_sec': 10.0
            }
            print(f"✓ Recorded '{name}' at ({pose['x']:.3f}, {pose['y']:.3f}, {pose['theta']:.3f})")

    def cmd_record_anchor(self, block_id):
        pose = self.get_robot_pose()
        if pose:
            self.data['merlin_config']['anchors'][block_id] = {'x': pose['x'], 'y': pose['y']}
            print(f"✓ Recorded anchor '{block_id}'")

    def cmd_validate(self):
        errors = []
        anchors = self.data.get('merlin_config', {}).get('anchors', {})
        for key in ['block_1', 'block_2', 'block_4']:
            if key not in anchors:
                errors.append(f"Missing anchor: {key}")
        if errors:
            print("✗ Validation failed:")
            for e in errors: print(f"  - {e}")
        else:
            print("✓ Validation passed")

    def cmd_save(self):
        self.cmd_validate()
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        tmp = self.output_path + '.tmp'
        with open(tmp, 'w') as f:
            yaml.dump(self.data, f, default_flow_style=False)
        os.rename(tmp, self.output_path)  # 原子写
        print(f"✓ Saved to {self.output_path}")

    def run_repl(self):
        while rclpy.ok():
            try:
                line = input('> ').strip()
                parts = line.split()
                if not parts: continue
                cmd = parts[0]
                if cmd == 'record' and len(parts) >= 2:
                    self.cmd_record(parts[1], *parts[2:])
                elif cmd == 'record_anchor' and len(parts) >= 2:
                    self.cmd_record_anchor(parts[1])
                elif cmd == 'list':
                    print("Static points:", list(self.data.get('static_points', {}).keys()))
                    print("Anchors:", list(self.data.get('merlin_config', {}).get('anchors', {}).keys()))
                elif cmd == 'validate':
                    self.cmd_validate()
                elif cmd == 'save':
                    self.cmd_save()
                elif cmd in ('quit', 'exit'):
                    break
                else:
                    print("Commands: record <name>, record_anchor <block_id>, list, validate, save, quit")
            except (EOFError, KeyboardInterrupt):
                break

def main():
    rclpy.init()
    node = PointRecorder()
    node.run_repl()
    rclpy.shutdown()

if __name__ == '__main__':
    main()