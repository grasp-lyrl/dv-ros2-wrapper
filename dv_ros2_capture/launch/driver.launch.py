import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

driver_args = [
    DeclareLaunchArgument('visualization_enable', default_value='false'),
    DeclareLaunchArgument(
        'bag_prefix', default_value='/data/nf1_',
        description='Bag path prefix, timestamp appended; run.sh mounts /mnt/extreme-pro at /data'),
]


def generate_launch_description():
    """Generate launch description with multiple components."""
    config_dir = os.path.join(FindPackageShare('dv_ros2_capture').find('dv_ros2_capture'), 'config')
    settings_file = os.path.join(config_dir, 'settings.yaml')
    dynamic_file = os.path.join(config_dir, 'dynamic.yaml')

    ld = LaunchDescription(driver_args)

    ld.add_action(
        ComposableNodeContainer(
            name='dv_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='dv_ros2_capture',
                    plugin='dv_capture_node::CaptureNode',
                    name='capture_node',
                    parameters=[settings_file, dynamic_file],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    condition=IfCondition(LaunchConfiguration('visualization_enable')),
                    package='dv_ros2_visualization',
                    plugin='dv_visualization_node::VisualizationNode',
                    name='visualization_node',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    condition=IfCondition(LaunchConfiguration('visualization_enable')),
                    package='dv_ros2_visualization',
                    plugin='dv_visualization_node::ImuVisualizationNode',
                    name='imu_visualization_node',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                # Idle until `ros2 service call /start_recording std_srvs/srv/Trigger`.
                ComposableNode(
                    package='rosbag2_composable_recorder',
                    plugin='rosbag2_composable_recorder::ComposableRecorder',
                    name='recorder',
                    parameters=[{
                        'topics': [
                            '/neurofly1/zed_node/rgb/image_rect_color',
                            '/neurofly1/zed_node/rgb/camera_info',
                            '/neurofly1/zed_node/depth/depth_registered',
                            '/neurofly1/zed_node/depth/camera_info',
                            '/neurofly1/control_odom',
                            '/neurofly1/mavros/distance_sensor',
                            # Relative, so they follow the capture node's namespace.
                            'events',
                            'imu',
                        ],
                        'storage_id': 'mcap',
                        'record_all': False,
                        'disable_discovery': False,
                        'serialization_format': 'cdr',
                        'start_recording_immediately': False,
                        'bag_prefix': LaunchConfiguration('bag_prefix'),
                        'qos_profile_overrides_path': os.path.join(config_dir, 'recorder_qos.yaml'),
                        'max_cache_size': 1024 * 1024 * 1024,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )
    )

    return ld
