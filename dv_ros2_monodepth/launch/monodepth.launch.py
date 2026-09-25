import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

monodepth_args = [
    DeclareLaunchArgument('model_path', description='AOTI .pt2 from export_f3_aoti.py --module depth'),
    DeclareLaunchArgument('window_ms', default_value='50'),
    DeclareLaunchArgument('window_stride', default_value='2', description='Run on every Nth window'),
    DeclareLaunchArgument('max_events', default_value='0', description='0 keeps every event'),
    DeclareLaunchArgument('visualization_enable', default_value='true'),
    DeclareLaunchArgument('camera_height', default_value='1.0', description='Metres above the floor'),
    DeclareLaunchArgument('height_topic', default_value='', description='sensor_msgs/Range; empty uses camera_height'),
    DeclareLaunchArgument('max_depth', default_value='5.0', description='Metres; 0 keeps every depth'),
    DeclareLaunchArgument(
        'calibration_file', default_value='',
        description="Metric depth node's OpenCV calibration; empty uses dv_ros2_capture's calib_40deg.xml"),
]


def generate_launch_description():
    """Capture and monodepth in one container, so events never leave the process."""
    config_dir = os.path.join(FindPackageShare('dv_ros2_capture').find('dv_ros2_capture'), 'config')
    settings_file = os.path.join(config_dir, 'settings.yaml')
    dynamic_file = os.path.join(config_dir, 'dynamic.yaml')

    ld = LaunchDescription(monodepth_args)

    ld.add_action(
        ComposableNodeContainer(
            name='dv_container',
            namespace='',
            package='rclcpp_components',
            # Multi-threaded, so the 1 kHz event callback doesn't block depth.
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='dv_ros2_capture',
                    plugin='dv_capture_node::CaptureNode',
                    name='capture_node',
                    parameters=[settings_file, dynamic_file],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='dv_ros2_monodepth',
                    plugin='dv_monodepth_node::MonoDepthNode',
                    name='monodepth_node',
                    parameters=[{
                        'input_topic': 'events',
                        'model_path': LaunchConfiguration('model_path'),
                        'window_ms': LaunchConfiguration('window_ms'),
                        'window_stride': LaunchConfiguration('window_stride'),
                        'max_events': LaunchConfiguration('max_events'),
                        'sensor_width': 640,
                        'sensor_height': 480,
                        'publish_visualization': True,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='dv_ros2_monodepth',
                    plugin='dv_monodepth_node::MetricDepthNode',
                    name='metric_depth_node',
                    parameters=[{
                        'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'), value_type=str),
                        'camera_height': LaunchConfiguration('camera_height'),
                        'height_topic': ParameterValue(LaunchConfiguration('height_topic'), value_type=str),
                        'max_depth': LaunchConfiguration('max_depth'),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    condition=IfCondition(LaunchConfiguration('visualization_enable')),
                    package='dv_ros2_visualization',
                    plugin='dv_visualization_node::VisualizationNode',
                    name='visualization_node',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )
    )

    return ld
