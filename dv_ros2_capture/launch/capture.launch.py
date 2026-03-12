from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='capture_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='dv_ros2_capture',
                    plugin='dv_capture_node::CaptureNode',
                    name='capture_node',
                ),
            ],
            output='screen',
        ),
    ])
