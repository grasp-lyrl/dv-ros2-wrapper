from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='visualization_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='dv_ros2_visualization',
                    plugin='dv_visualization_node::VisualizationNode',
                    name='visualization_node',
                ),
            ],
            output='screen',
        ),
    ])
