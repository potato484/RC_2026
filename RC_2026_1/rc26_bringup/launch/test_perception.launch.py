"""
rc26_perception 模块测试

功能: 验证 D455 相机 + YOLO 物块检测
前置: 需要 D455 相机连接或图像话题

测试指令:
    # Pass-through 模式 (无需模型)
    ros2 launch rc26_bringup test_perception.launch.py

    # 带 YOLO 模型
    ros2 launch rc26_bringup test_perception.launch.py \
        model_path:=/path/to/yolo.tflite

验证:
    ros2 topic echo /rc26/block_detections --once
    ros2 topic hz /rc26/block_detections
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    perception_dir = get_package_share_directory('rc26_perception')
    
    model_path = LaunchConfiguration('model_path')
    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='YOLO 模型路径 (空=pass-through 模式)')
    
    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='RGB 图像话题')
    
    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/aligned_depth_to_color/image_raw',
        description='深度图像话题')
    
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='相机参数话题')
    
    perception_config = PathJoinSubstitution([perception_dir, 'config', 'perception.yaml'])
    
    perception_node = Node(
        package='rc26_perception',
        executable='perception_node',
        name='perception',
        output='screen',
        parameters=[
            perception_config,
            {
                'model_path': model_path,
                'image_topic': image_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
            },
        ],
    )
    
    return LaunchDescription([
        declare_model_path,
        declare_image_topic,
        declare_depth_topic,
        declare_camera_info_topic,
        perception_node,
    ])
