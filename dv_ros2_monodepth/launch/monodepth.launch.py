import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

monodepth_args = [
    DeclareLaunchArgument('model_path', description='AOTI .pt2 from export_f3_aoti.py --module depth'),
    DeclareLaunchArgument('window_ms', default_value='50'),
    DeclareLaunchArgument('max_events', default_value='0', description='0 keeps every event'),
    DeclareLaunchArgument('visualization_enable', default_value='true'),
    DeclareLaunchArgument('undistort_events', default_value='true'),
    DeclareLaunchArgument(
        'calibration_file', default_value='',
        description='OpenCV FileStorage calibration; defaults to the capture package config'),
]


def generate_launch_description():
    """Capture and monodepth in one container, so events never leave the process."""
    config_dir = os.path.join(FindPackageShare('dv_ros2_capture').find('dv_ros2_capture'), 'config')
    settings_file = os.path.join(config_dir, 'settings.yaml')
    dynamic_file = os.path.join(config_dir, 'dynamic.yaml')
    default_calib = os.path.join(config_dir, 'calib_40deg.xml')

    ld = LaunchDescription(monodepth_args)

    ld.add_action(
        ComposableNodeContainer(
            name='dv_container',
            namespace='',
            package='rclcpp_components',
            # Multi-threaded: the 1 kHz event callback and the depth publisher should not
            # take turns on one executor thread.
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='dv_ros2_capture',
                    plugin='dv_capture_node::CaptureNode',
                    name='capture_node',
                    parameters=[settings_file, dynamic_file, {
                        # Events are undistorted before publishing, so every consumer
                        # downstream sees a rectified stream.
                        'undistortEvents': LaunchConfiguration('undistort_events'),
                        'opencvCalibrationFilePath': PythonExpression(
                            ["'", LaunchConfiguration('calibration_file'), "' or '", default_calib, "'"]),
                    }],
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
                        'max_events': LaunchConfiguration('max_events'),
                        'sensor_width': 640,
                        'sensor_height': 480,
                        'publish_visualization': True,
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
