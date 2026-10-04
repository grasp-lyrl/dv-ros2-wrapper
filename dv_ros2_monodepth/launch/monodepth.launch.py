import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

monodepth_args = [
    DeclareLaunchArgument('model_path', description='AOTI .pt2 from export_f3_aoti.py --module depth'),
    DeclareLaunchArgument('events_topic', default_value='/neurofly1/events'),
    DeclareLaunchArgument('capture', default_value='true',
                          description='Run the event camera in this container as neurofly1; false for bag replays'),
]


def generate_launch_description():
    share = FindPackageShare('dv_ros2_monodepth').find('dv_ros2_monodepth')
    config_file = os.path.join(share, 'config', 'monodepth.yaml')
    capture_config = os.path.join(FindPackageShare('dv_ros2_capture').find('dv_ros2_capture'), 'config')
    ld = LaunchDescription(monodepth_args)

    ld.add_action(
        ComposableNodeContainer(
            name='monodepth_container',
            namespace='',
            package='rclcpp_components',
            # Multi-threaded, so the 1 kHz event callback doesn't block depth.
            executable='component_container_mt',
            composable_node_descriptions=[
                # As in system_launch.launch.py, but in this container so events reach monodepth intra-process.
                ComposableNode(
                    condition=IfCondition(LaunchConfiguration('capture')),
                    package='dv_ros2_capture',
                    plugin='dv_capture_node::CaptureNode',
                    name='capture_node',
                    namespace='neurofly1',
                    parameters=[os.path.join(capture_config, 'settings.yaml'),
                                os.path.join(capture_config, 'dynamic.yaml')],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='dv_ros2_monodepth',
                    plugin='dv_monodepth_node::MonoDepthNode',
                    name='monodepth_node',
                    parameters=[config_file, {
                        'input_topic': LaunchConfiguration('events_topic'),
                        'model_path': LaunchConfiguration('model_path'),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='dv_ros2_monodepth',
                    plugin='dv_monodepth_node::MetricDepthNode',
                    name='metric_depth_node',
                    parameters=[config_file],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )
    )

    return ld
