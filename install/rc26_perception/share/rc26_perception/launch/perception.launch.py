"""
RC26 感知模块启动文件 - 生产模式
仅发布检测结果到话题，供决策/黑板等模块调用，不包含可视化

使用方法:
    # 基本启动 (需要外部相机驱动)
    ros2 launch rc26_perception perception.launch.py enable_driver:=false

    # 启动相机驱动 + 感知
    ros2 launch rc26_perception perception.launch.py model_path:=/path/to/model.bin

    # 自定义类别模式
    ros2 launch rc26_perception perception.launch.py \\
        model_path:=/path/to/model.bin \\
        use_custom_classes:=true \\
        num_classes:=5
"""
import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = get_package_share_directory('rc26_perception')
    config_file = os.path.join(pkg_dir, 'config', 'perception.yaml')
    # ========== D455 相机参数 ==========
    camera_width_arg = DeclareLaunchArgument(
        'camera_width', default_value='640',
        description='D455 image width')
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height', default_value='480',
        description='D455 image height')
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps', default_value='30',
        description='D455 framerate')
    
    enable_driver_arg = DeclareLaunchArgument(
        'enable_driver', default_value='true',
        description='Enable D455 driver node (set false if using external driver)')
    
    # ========== YOLO 推理参数 ==========
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='YOLO model file path (empty = pass-through mode)')
    
    input_size_arg = DeclareLaunchArgument(
        'input_size', default_value='640',
        description='YOLO input size')
    
    conf_thres_arg = DeclareLaunchArgument(
        'conf_thres', default_value='0.45',
        description='Confidence threshold')
    
    iou_thres_arg = DeclareLaunchArgument(
        'iou_thres', default_value='0.45',
        description='NMS IoU threshold')
    
    num_classes_arg = DeclareLaunchArgument(
        'num_classes', default_value='80',
        description='Number of classes (80 for COCO, 5 for custom)')
    
    use_custom_classes_arg = DeclareLaunchArgument(
        'use_custom_classes', default_value='false',
        description='Use custom block classes instead of COCO')
    
    # ========== 话题配置 ==========
    color_topic_arg = DeclareLaunchArgument(
        'color_topic', default_value='/camera/color/image_raw',
        description='Color image topic')
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/aligned_depth_to_color/image_raw',
        description='Depth image topic')
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value='/rc26/block_detections',
        description='Detection output topic')
    
    # ========== D455 驱动节点 ==========
    d455_driver_node = Node(
        package='rc26_perception',
        executable='d455_driver_node',
        name='d455_driver_node',
        output='screen',
        parameters=[
            config_file,  # 先加载 yaml 配置文件
            {
                'width': LaunchConfiguration('camera_width'),
                'height': LaunchConfiguration('camera_height'),
                'fps': LaunchConfiguration('camera_fps'),
                'color_topic': LaunchConfiguration('color_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'frame_id': 'd455_color_optical_frame',
            }
        ],
        condition=launch.conditions.IfCondition(LaunchConfiguration('enable_driver')),
    )
    
    # ========== 感知节点 (仅发布检测结果) ==========
    perception_node = Node(
        package='rc26_perception',
        executable='perception_node',
        name='rc26_perception_node',
        output='screen',
        parameters=[
            config_file,  # 先加载 yaml 配置文件
            {
                'model_path': LaunchConfiguration('model_path'),
                'input_size': LaunchConfiguration('input_size'),
                'conf_thres': LaunchConfiguration('conf_thres'),
                'iou_thres': LaunchConfiguration('iou_thres'),
                'num_classes': LaunchConfiguration('num_classes'),
                'use_custom_classes': LaunchConfiguration('use_custom_classes'),
                'color_topic': LaunchConfiguration('color_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                # D455 默认内参
                'camera_fx': 386.0,
                'camera_fy': 386.0,
                'camera_cx': 320.0,
                'camera_cy': 240.0,
                'camera_frame': 'd455_color_optical_frame',
                'base_frame': 'base_link',
            }
        ],
    )
    
    return LaunchDescription([
        # 启动日志
        LogInfo(msg='========================================'),
        LogInfo(msg='RC26 Perception - Production Mode'),
        LogInfo(msg='Output topic: /rc26/block_detections'),
        LogInfo(msg='========================================'),
        # 相机参数
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        enable_driver_arg,
        # YOLO 参数
        model_path_arg,
        input_size_arg,
        conf_thres_arg,
        iou_thres_arg,
        num_classes_arg,
        use_custom_classes_arg,
        color_topic_arg,
        depth_topic_arg,
        output_topic_arg,
        # 节点
        d455_driver_node,
        perception_node,
    ])
