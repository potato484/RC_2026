"""
RC26 感知模块启动文件 - 可视化调试模式
用于验证模型识别效果，带有可缩放的可视化界面

使用方法:

    # 使用 ONNX 模型验证识别
    ros2 launch rc26_perception visualization.launch.py \
        model_path:=/home/potato/RC_2026/RC_2026_1/rc26_perception/models/yolov8s.onnx

    # 使用自定义模型 + 调整阈值
    ros2 launch rc26_perception visualization.launch.py \
        model_path:=/home/potato/RC_2026/RC_2026_1/rc26_perception/models/best.onnx \
        conf_thres:=0.5 
   
    # 仅显示深度图
    ros2 launch rc26_perception visualization.launch.py \
        show_depth_window:=true

界面功能:
    - 窗口可自由拖拽缩放
    - 显示 FPS、推理时间、分辨率等信息
    - 显示各类别检测统计
    - 按 Q/ESC 退出
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = get_package_share_directory('rc26_perception')
    config_file = os.path.join(pkg_dir, 'config', 'perception.yaml')


    # ========== 相机驱动 ==========
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
        }.items()
    )

    # ========== 模型参数 ==========
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/potato/RC_2026/RC_2026_1/rc26_perception/models/best.onnx',
        description='ONNX model path (empty = display only, no inference)')
    
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
        'num_classes', default_value='32',
        description='Number of classes (32 for RC26)')
    
    # ========== 话题配置 ==========
    color_topic_arg = DeclareLaunchArgument(
        'color_topic', default_value='/camera/camera/color/image_raw',
        description='Color image topic')
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw',
        description='Depth image topic')
    
    # ========== 窗口配置 ==========
    window_name_arg = DeclareLaunchArgument(
        'window_name', default_value='RC26 Detection Visualization',
        description='Main window title')
    
    window_width_arg = DeclareLaunchArgument(
        'window_width', default_value='960',
        description='Initial window width')
    
    window_height_arg = DeclareLaunchArgument(
        'window_height', default_value='720',
        description='Initial window height')
    
    show_info_panel_arg = DeclareLaunchArgument(
        'show_info_panel', default_value='true',
        description='Show info panel overlay')
    
    # ========== 深度可视化配置 ==========
    show_depth_arg = DeclareLaunchArgument(
        'show_depth', default_value='true',
        description='Show depth distance in detection labels')
    
    show_depth_window_arg = DeclareLaunchArgument(
        'show_depth_window', default_value='true',
        description='Show separate depth visualization window')
    
    depth_colormap_arg = DeclareLaunchArgument(
        'depth_colormap', default_value='2',
        description='Depth colormap: 0=GRAY, 2=JET, 4=RAINBOW, 11=TURBO')
    
    depth_vis_min_arg = DeclareLaunchArgument(
        'depth_vis_min', default_value='0.1',
        description='Depth visualization min (meters)')
    
    depth_vis_max_arg = DeclareLaunchArgument(
        'depth_vis_max', default_value='5.0',
        description='Depth visualization max (meters)')
    
    # ========== 可视化节点 ==========
    visualization_node = Node(
        package='rc26_perception',
        executable='visualization_node.py',
        name='rc26_visualization_node',
        output='screen',
        emulate_tty=True,  # 保持彩色输出
        parameters=[
            config_file,  # 先加载 yaml 配置文件作为默认值
            {
                # 命令行参数覆盖 (仅当用户指定时生效)
                'model_path': LaunchConfiguration('model_path'),
                'input_size': LaunchConfiguration('input_size'),
                'conf_thres': LaunchConfiguration('conf_thres'),
                'iou_thres': LaunchConfiguration('iou_thres'),
                'num_classes': LaunchConfiguration('num_classes'),
                'color_topic': LaunchConfiguration('color_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'window_name': LaunchConfiguration('window_name'),
                'window_width': LaunchConfiguration('window_width'),
                'window_height': LaunchConfiguration('window_height'),
                'show_info_panel': LaunchConfiguration('show_info_panel'),
                'show_fps': True,
                'show_depth': LaunchConfiguration('show_depth'),
                'show_depth_window': LaunchConfiguration('show_depth_window'),
                'depth_colormap': LaunchConfiguration('depth_colormap'),
                'depth_vis_min': LaunchConfiguration('depth_vis_min'),
                'depth_vis_max': LaunchConfiguration('depth_vis_max'),
            }
        ],
    )
    
    return LaunchDescription([
        # 启动日志
        LogInfo(msg='========================================'),
        LogInfo(msg='RC26 Perception - Visualization Mode'),
        LogInfo(msg='Starting D455 camera driver...'),
        LogInfo(msg='Drag window edges to resize'),
        LogInfo(msg='Press Q/ESC to quit'),
        LogInfo(msg='========================================'),
        # 相机驱动
        realsense_launch,
        # 参数
        model_path_arg,
        input_size_arg,
        conf_thres_arg,
        iou_thres_arg,
        num_classes_arg,
        color_topic_arg,
        depth_topic_arg,
        window_name_arg,
        window_width_arg,
        window_height_arg,
        show_info_panel_arg,
        show_depth_arg,
        show_depth_window_arg,
        depth_colormap_arg,
        depth_vis_min_arg,
        depth_vis_max_arg,
        # 节点
        visualization_node,
    ])
