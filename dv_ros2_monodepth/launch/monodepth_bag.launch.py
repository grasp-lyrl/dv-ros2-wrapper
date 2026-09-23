import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

bag_args = [
    DeclareLaunchArgument('model_path', description='AOTI .pt2 from export_f3_aoti.py'),
    DeclareLaunchArgument('bag', default_value='',
                          description='Bag to replay once; leave empty to play one yourself'),
    DeclareLaunchArgument('events_topic', default_value='/neurofly1/events'),
    DeclareLaunchArgument('undistort_events', default_value='true'),
    DeclareLaunchArgument(
        'calibration_file', default_value='',
        description='OpenCV calibration; defaults to the capture package calib_80deg.xml'),
    DeclareLaunchArgument('visualization_enable', default_value='true'),
    DeclareLaunchArgument('rviz', default_value='true'),
    DeclareLaunchArgument('sensor_width', default_value='640'),
    DeclareLaunchArgument('sensor_height', default_value='480'),
]


def generate_launch_description():
    """Monodepth over a recorded event stream -- no camera, so no capture node."""
    share = FindPackageShare('dv_ros2_monodepth').find('dv_ros2_monodepth')
    qos_override = os.path.join(share, 'config', 'bag_qos_override.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'monodepth.rviz')
    capture_config = os.path.join(
        FindPackageShare('dv_ros2_capture').find('dv_ros2_capture'), 'config')
    default_calib = os.path.join(capture_config, 'calib_80deg.xml')

    undistort = LaunchConfiguration('undistort_events')
    # With undistortion on, everything downstream reads the rectified stream instead.
    stream = PythonExpression([
        "'/events_undistorted' if '", undistort, "'.lower() == 'true' else '",
        LaunchConfiguration('events_topic'), "'"])

    ld = LaunchDescription(bag_args)
    ld.add_action(
        ComposableNodeContainer(
            name='dv_container', namespace='', package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # The bag path has no capture node, so this does the capture node's
                # undistortion, with the same calibration and lookup.
                ComposableNode(
                    condition=IfCondition(undistort),
                    package='dv_ros2_capture', plugin='dv_capture_node::EventUndistortNode',
                    name='event_undistort_node',
                    parameters=[{
                        'input_topic': LaunchConfiguration('events_topic'),
                        'output_topic': '/events_undistorted',
                        'calibration_file': PythonExpression(
                            ["'", LaunchConfiguration('calibration_file'), "' or '", default_calib, "'"]),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='dv_ros2_monodepth', plugin='dv_monodepth_node::MonoDepthNode',
                    name='monodepth_node',
                    parameters=[{
                        'input_topic': stream,
                        'model_path': LaunchConfiguration('model_path'),
                        'window_ms': 50,
                        'sensor_width': LaunchConfiguration('sensor_width'),
                        'sensor_height': LaunchConfiguration('sensor_height'),
                        'publish_visualization': True,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    condition=IfCondition(LaunchConfiguration('visualization_enable')),
                    package='dv_ros2_visualization', plugin='dv_visualization_node::VisualizationNode',
                    name='visualization_node',
                    remappings=[('events', stream)],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            output='screen'))

    # Held back a few seconds so the model has loaded and run its first (slow) pass before
    # events arrive. The override is what makes the replayed topics visible at all.
    ld.add_action(TimerAction(period=4.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
                 '--qos-profile-overrides-path', qos_override],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('bag'), "' != ''"])),
            output='screen'),
    ]))

    ld.add_action(Node(package='rviz2', executable='rviz2', name='rviz2',
                       arguments=['-d', rviz_cfg], output='log',
                       condition=IfCondition(LaunchConfiguration('rviz'))))
    return ld
