#!/usr/bin/env python3
"""
RC26 感知可视化节点 - 使用 ONNX Runtime
用法: ros2 run rc26_perception visualization_node.py --ros-args -p model_path:=/path/to/yolov8s.onnx
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

import cv2
import numpy as np
import onnxruntime as ort
import time

# RC26 32 类别名称
RC26_CLASSES = [
    'R_R1', 'B_R1',
    'T_03', 'T_04', 'T_05', 'T_06', 'T_07', 'T_08', 'T_09', 'T_10',
    'T_11', 'T_12', 'T_13', 'T_14', 'T_15', 'T_16', 'T_17',
    'F_18', 'F_19', 'F_20', 'F_21', 'F_22', 'F_23', 'F_24', 'F_25',
    'F_26', 'F_27', 'F_28', 'F_29', 'F_30', 'F_31', 'F_32'
]

# 颜色表 (BGR)
COLORS = [
    (255, 56, 56), (56, 255, 56), (56, 56, 255), (255, 178, 56),
    (178, 56, 255), (56, 255, 255), (255, 56, 178), (56, 178, 255)
]


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('rc26_visualization_node')

        # 声明参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('conf_thres', 0.45)
        self.declare_parameter('iou_thres', 0.45)
        self.declare_parameter('num_classes', 32)
        self.declare_parameter('window_name', 'RC26 Detection')
        self.declare_parameter('show_fps', True)
        self.declare_parameter('show_depth', True)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        
        # 深度可视化参数
        self.declare_parameter('show_depth_window', True)   # 显示深度图窗口
        self.declare_parameter('depth_colormap', 2)         # 深度色图: 0=灰度, 2=JET, 4=RAINBOW, 11=TURBO
        self.declare_parameter('depth_vis_min', 0.1)        # 深度可视化最小值 (米)
        self.declare_parameter('depth_vis_max', 5.0)        # 深度可视化最大值 (米)
        
        # 窗口参数
        self.declare_parameter('window_width', 960)         # 初始窗口宽度
        self.declare_parameter('window_height', 720)        # 初始窗口高度
        self.declare_parameter('show_info_panel', True)     # 显示信息面板

        # 获取参数
        self.model_path = self.get_parameter('model_path').value
        self.input_size = self.get_parameter('input_size').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.iou_thres = self.get_parameter('iou_thres').value
        self.num_classes = self.get_parameter('num_classes').value
        self.window_name = self.get_parameter('window_name').value
        self.show_fps = self.get_parameter('show_fps').value
        self.show_depth = self.get_parameter('show_depth').value
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        
        # 获取深度可视化参数
        self.show_depth_window = self.get_parameter('show_depth_window').value
        self.depth_colormap = self.get_parameter('depth_colormap').value
        self.depth_vis_min = self.get_parameter('depth_vis_min').value
        self.depth_vis_max = self.get_parameter('depth_vis_max').value
        
        # 获取窗口参数
        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value
        self.show_info_panel = self.get_parameter('show_info_panel').value

        # 初始化 ONNX Runtime
        self.session = None
        if self.model_path:
            try:
                self.get_logger().info(f'Loading ONNX model: {self.model_path}')
                providers = ['CPUExecutionProvider']
                self.session = ort.InferenceSession(self.model_path, providers=providers)
                self.input_name = self.session.get_inputs()[0].name
                self.get_logger().info('Model loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to load model: {e}')
        else:
            self.get_logger().warn('No model_path specified, visualization only')

        self.bridge = CvBridge()
        self.fps_smooth = 0.0
        self.inference_time_ms = 0.0
        self.frame_count = 0
        self.total_detections = 0
        self.class_counts = {}  # 各类别检测计数
        self.image_width = 0
        self.image_height = 0
        self.depth_available = False

        # 创建可缩放窗口 (WINDOW_NORMAL 允许用户调整大小)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        if self.show_depth_window:
            cv2.namedWindow('RC26 Depth', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow('RC26 Depth', self.window_width // 2, self.window_height // 2)

        # QoS for sensor data
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 同步订阅
        self.color_sub = message_filters.Subscriber(self, Image, self.color_topic, qos_profile=qos)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic, qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info('=' * 50)
        self.get_logger().info('RC26 Visualization Node started')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Model: {self.model_path if self.model_path else "(none - display only)"}')
        self.get_logger().info(f'  Input size: {self.input_size}x{self.input_size}')
        self.get_logger().info(f'  Confidence: {self.conf_thres}, IoU: {self.iou_thres}')
        self.get_logger().info(f'  Classes: {self.num_classes}')
        self.get_logger().info(f'  Color topic: {self.color_topic}')
        self.get_logger().info(f'  Depth topic: {self.depth_topic}')
        if self.show_depth_window:
            self.get_logger().info(f'  Depth window: ON (colormap={self.depth_colormap}, range={self.depth_vis_min:.1f}-{self.depth_vis_max:.1f}m)')
        self.get_logger().info('=' * 50)
        self.get_logger().info("Press 'q' or ESC to quit | Drag window edges to resize")

    def preprocess(self, frame):
        """预处理图像"""
        h, w = frame.shape[:2]
        # Letterbox resize
        scale = min(self.input_size / h, self.input_size / w)
        new_h, new_w = int(h * scale), int(w * scale)
        resized = cv2.resize(frame, (new_w, new_h))

        # Pad to input_size x input_size
        pad_h = self.input_size - new_h
        pad_w = self.input_size - new_w
        top, left = pad_h // 2, pad_w // 2
        bottom, right = pad_h - top, pad_w - left

        padded = cv2.copyMakeBorder(resized, top, bottom, left, right,
                                     cv2.BORDER_CONSTANT, value=(114, 114, 114))

        # Convert to float, normalize, HWC -> CHW, add batch dim
        blob = padded.astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)  # HWC -> CHW
        blob = np.expand_dims(blob, 0)  # Add batch dim
        blob = np.ascontiguousarray(blob)

        return blob, scale, (top, left)

    def postprocess(self, output, frame_shape, scale, pad):
        """后处理模型输出"""
        h, w = frame_shape[:2]
        pad_top, pad_left = pad

        # YOLOv8 输出: [1, 84, 8400] -> [8400, 84]
        output = output[0]  # Remove batch dim: [84, 8400]
        output = output.T   # Transpose: [8400, 84]

        boxes = []
        scores = []
        class_ids = []

        for row in output:
            cx, cy, bw, bh = row[:4]
            class_scores = row[4:4 + self.num_classes]
            max_score = np.max(class_scores)

            if max_score < self.conf_thres:
                continue

            class_id = np.argmax(class_scores)

            # Convert from input coords to original frame coords
            x1 = (cx - bw / 2 - pad_left) / scale
            y1 = (cy - bh / 2 - pad_top) / scale
            x2 = (cx + bw / 2 - pad_left) / scale
            y2 = (cy + bh / 2 - pad_top) / scale

            # Clip to frame bounds
            x1 = max(0, min(x1, w - 1))
            y1 = max(0, min(y1, h - 1))
            x2 = max(0, min(x2, w - 1))
            y2 = max(0, min(y2, h - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(float(max_score))
            class_ids.append(int(class_id))

        # NMS
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thres, self.iou_thres)
            indices = indices.flatten() if len(indices) > 0 else []
        else:
            indices = []

        detections = []
        for i in indices:
            x, y, bw, bh = boxes[i]
            detections.append({
                'box': (int(x), int(y), int(bw), int(bh)),
                'score': scores[i],
                'class_id': class_ids[i],
                'center': (int(x + bw / 2), int(y + bh / 2)),
                'distance': 0.0
            })

        return detections

    def fill_depth(self, detections, depth_img):
        """填充深度信息"""
        if depth_img is None:
            return
        h, w = depth_img.shape[:2]
        for det in detections:
            cx, cy = det['center']
            if 0 <= cx < w and 0 <= cy < h:
                # Sample 3x3 region around center
                x1 = max(0, cx - 1)
                x2 = min(w, cx + 2)
                y1 = max(0, cy - 1)
                y2 = min(h, cy + 2)
                region = depth_img[y1:y2, x1:x2]
                valid = region[region > 0]
                if len(valid) > 0:
                    det['distance'] = float(np.median(valid)) / 1000.0  # mm -> m

    def draw_detections(self, frame, detections):
        """绘制检测结果"""
        # 重置类别计数
        self.class_counts = {}
        
        for det in detections:
            x, y, bw, bh = det['box']
            class_id = det['class_id']
            score = det['score']
            distance = det['distance']

            color = COLORS[class_id % len(COLORS)]

            # Draw box with thicker lines for visibility
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)

            # Get class name
            if class_id < len(RC26_CLASSES):
                class_name = RC26_CLASSES[class_id]
            else:
                class_name = f'class_{class_id}'
            
            # 更新类别计数
            self.class_counts[class_name] = self.class_counts.get(class_name, 0) + 1

            # Build label text with box dimensions
            label_parts = [f'{class_name}: {score:.2f}']
            if distance > 0:
                label_parts.append(f'{distance:.2f}m')
            label_parts.append(f'{bw}x{bh}')
            label = ' | '.join(label_parts)

            # Draw label background
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            label_y = max(y - 5, th + 5)
            cv2.rectangle(frame, (x, label_y - th - 5), (x + tw + 5, label_y + 5), color, -1)
            cv2.putText(frame, label, (x + 2, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Draw center point with crosshair
            cx, cy = det['center']
            cv2.circle(frame, (cx, cy), 5, color, -1)
            cv2.line(frame, (cx - 8, cy), (cx + 8, cy), color, 1)
            cv2.line(frame, (cx, cy - 8), (cx, cy + 8), color, 1)
            
            # Draw pixel coordinates at center
            coord_text = f'({cx},{cy})'
            cv2.putText(frame, coord_text, (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

    def draw_info_panel(self, frame, detections, inference_ms, fps):
        """绘制信息面板"""
        if not self.show_info_panel:
            return
        
        h, w = frame.shape[:2]
        panel_width = 280
        panel_height = min(300, h - 20)
        
        # 半透明背景
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (10 + panel_width, 10 + panel_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # 绘制边框
        cv2.rectangle(frame, (10, 10), (10 + panel_width, 10 + panel_height), (100, 100, 100), 1)
        
        # 信息行
        y_offset = 35
        line_height = 22
        
        def draw_line(text, color=(255, 255, 255), bold=False):
            nonlocal y_offset
            thickness = 2 if bold else 1
            cv2.putText(frame, text, (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)
            y_offset += line_height
        
        # 标题
        draw_line('=== RC26 PERCEPTION ==', (0, 255, 255), bold=True)
        
        # 性能信息
        draw_line(f'FPS: {fps:.1f} | Infer: {inference_ms:.1f}ms', (0, 255, 0))
        
        # 图像信息
        draw_line(f'Resolution: {self.image_width}x{self.image_height}', (200, 200, 200))
        draw_line(f'Model input: {self.input_size}x{self.input_size}', (200, 200, 200))
        
        # 模型信息
        model_name = self.model_path.split('/')[-1] if self.model_path else '(none)'
        if len(model_name) > 30:
            model_name = model_name[:27] + '...'
        draw_line(f'Model: {model_name}', (200, 200, 200))
        
        # 阈值信息
        draw_line(f'Conf: {self.conf_thres} | IoU: {self.iou_thres}', (200, 200, 200))
        
        # 深度状态
        depth_status = 'OK' if self.depth_available else 'N/A'
        depth_color = (0, 255, 0) if self.depth_available else (0, 0, 255)
        draw_line(f'Depth: {depth_status}', depth_color)
        
        # 分隔线
        draw_line('-' * 35, (100, 100, 100))
        
        # 检测统计
        self.frame_count += 1
        self.total_detections += len(detections)
        avg_det = self.total_detections / self.frame_count if self.frame_count > 0 else 0
        draw_line(f'Detections: {len(detections)} (avg: {avg_det:.1f})', (255, 255, 0))
        
        # 各类别统计 (最多显示5个)
        if self.class_counts:
            sorted_classes = sorted(self.class_counts.items(), key=lambda x: -x[1])[:5]
            for class_name, count in sorted_classes:
                if y_offset + line_height > 10 + panel_height - 10:
                    break
                color_idx = RC26_CLASSES.index(class_name) if class_name in RC26_CLASSES else 0
                color = COLORS[color_idx % len(COLORS)]
                draw_line(f'  {class_name}: {count}', color)
        
        # 底部提示
        tip_y = 10 + panel_height - 15
        cv2.putText(frame, 'Press Q/ESC to quit', (20, tip_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

    def image_callback(self, color_msg, depth_msg):
        start_time = time.time()

        # Convert color image
        try:
            frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Color cv_bridge exception: {e}')
            return

        if frame is None or frame.size == 0:
            return
        
        # 记录图像尺寸
        self.image_height, self.image_width = frame.shape[:2]

        # Convert depth image
        depth_img = None
        self.depth_available = False
        if self.show_depth:
            try:
                depth_img = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                if depth_img is not None and depth_img.size > 0:
                    self.depth_available = True
            except Exception as e:
                self.get_logger().warn_once(f'Depth unavailable: {e}')

        # Run inference
        detections = []
        inference_start = time.time()
        if self.session is not None:
            blob, scale, pad = self.preprocess(frame)
            output = self.session.run(None, {self.input_name: blob})
            detections = self.postprocess(output[0], frame.shape, scale, pad)

            # Fill depth info
            if depth_img is not None:
                self.fill_depth(detections, depth_img)
        
        inference_end = time.time()
        self.inference_time_ms = (inference_end - inference_start) * 1000

        # Draw detections
        self.draw_detections(frame, detections)

        # Calculate FPS
        elapsed = time.time() - start_time
        fps = 1.0 / elapsed if elapsed > 0 else 0
        self.fps_smooth = 0.9 * self.fps_smooth + 0.1 * fps
        
        # Draw info panel (replaces old simple text)
        self.draw_info_panel(frame, detections, self.inference_time_ms, self.fps_smooth)
        
        # 在右下角显示时间戳
        timestamp = color_msg.header.stamp.sec + color_msg.header.stamp.nanosec * 1e-9
        ts_text = f't: {timestamp:.2f}'
        cv2.putText(frame, ts_text, (self.image_width - 120, self.image_height - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        # Show image
        cv2.imshow(self.window_name, frame)
        
        # 显示深度图窗口
        if self.show_depth_window and depth_img is not None:
            depth_vis = self.visualize_depth(depth_img)
            cv2.imshow('RC26 Depth', depth_vis)

        # Check for quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q') or key == 27:
            self.get_logger().info('Quit requested')
            raise SystemExit(0)
    
    def visualize_depth(self, depth_img):
        """将深度图可视化为彩色图像"""
        # 深度范围 (毫米)
        min_mm = self.depth_vis_min * 1000.0
        max_mm = self.depth_vis_max * 1000.0
        
        # 转换为浮点并裁剪到可视化范围
        depth_float = depth_img.astype(np.float32)
        depth_float = np.clip(depth_float, min_mm, max_mm)
        
        # 归一化到 0-255
        normalized = ((depth_float - min_mm) / (max_mm - min_mm) * 255.0).astype(np.uint8)
        
        # 应用颜色映射
        if self.depth_colormap == 0:
            # 灰度图
            colored = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        else:
            colored = cv2.applyColorMap(normalized, self.depth_colormap)
        
        # 将无效深度 (0) 标记为黑色
        invalid_mask = depth_img == 0
        colored[invalid_mask] = [0, 0, 0]
        
        # 添加深度刻度条
        self.draw_depth_colorbar(colored)
        
        return colored
    
    def draw_depth_colorbar(self, img):
        """绘制深度刻度条"""
        bar_width = 20
        bar_height = img.shape[0] - 40
        bar_x = img.shape[1] - bar_width - 10
        bar_y = 20
        
        # 绘制颜色条
        for i in range(bar_height):
            ratio = 1.0 - i / bar_height
            val = int(ratio * 255)
            
            if self.depth_colormap == 0:
                color = (val, val, val)
            else:
                val_mat = np.array([[val]], dtype=np.uint8)
                colored = cv2.applyColorMap(val_mat, self.depth_colormap)
                color = tuple(int(c) for c in colored[0, 0])
            
            cv2.line(img, (bar_x, bar_y + i), (bar_x + bar_width, bar_y + i), color, 1)
        
        # 绘制边框
        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), (255, 255, 255), 1)
        
        # 绘制刻度标签
        cv2.putText(img, f'{self.depth_vis_max:.1f}m', (bar_x - 35, bar_y + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(img, f'{(self.depth_vis_min + self.depth_vis_max) / 2:.1f}m', 
                    (bar_x - 35, bar_y + bar_height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(img, f'{self.depth_vis_min:.1f}m', (bar_x - 35, bar_y + bar_height - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
