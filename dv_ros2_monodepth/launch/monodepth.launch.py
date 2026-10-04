import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

monodepth_args = [
    DeclareLaunchArgument('model_path', description='AOTI .pt2 from export_f3_aoti.py --module depth'),
    DeclareLaunchArgument('events_topic', default_value='events'),
]


def generate_launch_description():
    share = FindPackageShare('dv_ros2_monodepth').find('dv_ros2_monodepth')
    config_file = os.path.join(share, 'config', 'monodepth.yaml')
    ld = LaunchDescription(monodepth_args)

    ld.add_action(
        ComposableNodeContainer(
            name='monodepth_container',
            namespace='',
            package='rclcpp_components',
            # Multi-threaded, so the 1 kHz event callback doesn't block depth.
            executable='component_container_mt',
            composable_node_descriptions=[
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
