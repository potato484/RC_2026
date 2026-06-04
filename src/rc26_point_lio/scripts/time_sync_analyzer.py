#!/usr/bin/env python3
"""
时间同步分析脚本
用于检测 DM_IMU 与 Mid-360 LiDAR 之间的时间偏移

使用方法:
    1. 启动 DM_IMU 和 Mid-360 驱动
    2. 运行: python3 time_sync_analyzer.py
    3. 等待采集完成，查看建议的 LiDAR/IMU 外部时间偏移值
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
import numpy as np
from collections import deque
import threading


class TimeSyncAnalyzer(Node):
    def __init__(self):
        super().__init__('time_sync_analyzer')

        # 配置参数
        self.declare_parameter('imu_topic', 'DM_IMU')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('sample_count', 100)
        self.declare_parameter('window_size', 50)

        imu_topic = self.get_parameter('imu_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        self.sample_count = self.get_parameter('sample_count').value
        self.window_size = self.get_parameter('window_size').value

        # 数据存储
        self.imu_timestamps = deque(maxlen=1000)
        self.lidar_timestamps = deque(maxlen=100)
        self.time_diffs = []
        self.lock = threading.Lock()

        # 统计
        self.imu_count = 0
        self.lidar_count = 0

        # QoS 配置
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # 订阅
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos)
        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self.lidar_callback, qos)

        # 定时输出
        self.timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info(f'时间同步分析器已启动')
        self.get_logger().info(f'  IMU 话题: {imu_topic}')
        self.get_logger().info(f'  LiDAR 话题: {lidar_topic}')
        self.get_logger().info(f'  采样数量: {self.sample_count}')
        self.get_logger().info('等待数据...')

    def imu_callback(self, msg):
        with self.lock:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.imu_timestamps.append(ts)
            self.imu_count += 1

    def lidar_callback(self, msg):
        with self.lock:
            lidar_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.lidar_timestamps.append(lidar_ts)
            self.lidar_count += 1

            # 找到最近的 IMU 时间戳
            if len(self.imu_timestamps) > 0:
                imu_arr = np.array(list(self.imu_timestamps))
                idx = np.argmin(np.abs(imu_arr - lidar_ts))
                nearest_imu_ts = imu_arr[idx]

                # 计算时间差: IMU - LiDAR
                diff = nearest_imu_ts - lidar_ts

                # 只记录合理范围内的差值 (±1秒)
                if abs(diff) < 1.0:
                    self.time_diffs.append(diff)

    def print_status(self):
        with self.lock:
            n = len(self.time_diffs)
            self.get_logger().info(f'已采集: IMU={self.imu_count}, LiDAR={self.lidar_count}, 有效样本={n}/{self.sample_count}')

            if n >= self.sample_count:
                self.analyze_and_report()
                rclpy.shutdown()

    def analyze_and_report(self):
        diffs = np.array(self.time_diffs)

        # 统计分析
        mean_diff = np.mean(diffs)
        median_diff = np.median(diffs)
        std_diff = np.std(diffs)
        min_diff = np.min(diffs)
        max_diff = np.max(diffs)

        # 使用中位数作为推荐值（更鲁棒）
        recommended = median_diff

        self.get_logger().info('=' * 60)
        self.get_logger().info('时间同步分析结果')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'样本数量: {len(diffs)}')
        self.get_logger().info(f'时间差 (IMU - LiDAR):')
        self.get_logger().info(f'  平均值: {mean_diff*1000:.3f} ms')
        self.get_logger().info(f'  中位数: {median_diff*1000:.3f} ms')
        self.get_logger().info(f'  标准差: {std_diff*1000:.3f} ms')
        self.get_logger().info(f'  范围: [{min_diff*1000:.3f}, {max_diff*1000:.3f}] ms')
        self.get_logger().info('-' * 60)

        # 判断同步质量
        if std_diff < 0.005:
            quality = '优秀'
        elif std_diff < 0.01:
            quality = '良好'
        elif std_diff < 0.02:
            quality = '一般'
        else:
            quality = '较差（建议检查时钟源）'

        self.get_logger().info(f'同步质量: {quality}')
        self.get_logger().info('-' * 60)
        self.get_logger().info('推荐外部时间偏移:')
        self.get_logger().info(f'  LiDAR/IMU offset (IMU - LiDAR): {recommended:.6f} s')
        self.get_logger().info('')
        self.get_logger().info('Point-LIO 当前不消费该脚本输出；如需补偿时间偏移，请在驱动或独立同步链路中处理。')
        self.get_logger().info('=' * 60)

        # 如果偏移较大，给出警告
        if abs(recommended) > 0.05:
            self.get_logger().warn(f'时间偏移较大 ({recommended*1000:.1f}ms)，请检查:')
            self.get_logger().warn('  1. DM_IMU 和 LiDAR 是否使用相同时钟源')
            self.get_logger().warn('  2. 网络/串口传输是否有额外延迟')


def main():
    rclpy.init()
    node = TimeSyncAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
