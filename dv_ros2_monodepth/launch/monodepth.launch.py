from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

monodepth_args = [
    DeclareLaunchArgument('model_path', description='AOTI .pt2 from export_f3_aoti.py --module depth'),
    DeclareLaunchArgument('events_topic', default_value='events'),
    DeclareLaunchArgument('window_ms', default_value='50'),
    DeclareLaunchArgument('window_stride', default_value='2', description='Run on every Nth window'),
    DeclareLaunchArgument('max_events', default_value='0', description='0 keeps every event'),
    DeclareLaunchArgument('visualization_enable', default_value='true'),
    DeclareLaunchArgument('camera_height', default_value='1.0', description='Metres above the floor'),
    DeclareLaunchArgument('height_topic', default_value='', description='sensor_msgs/Range; empty uses camera_height'),
    DeclareLaunchArgument('attitude_topic', default_value='',
                          description='nav_msgs/Odometry giving the floor fit gravity; empty assumes a level camera'),
    DeclareLaunchArgument('max_depth', default_value='5.0', description='Metres; 0 keeps every depth'),
    DeclareLaunchArgument(
        'calibration_file', default_value='',
        description="Metric depth node's OpenCV calibration; empty uses dv_ros2_capture's calib_40deg.xml"),
    DeclareLaunchArgument('undistort_depth', default_value='false',
                          description='Publish metric depth as an undistorted pinhole image'),
    DeclareLaunchArgument('min_events', default_value='0', description='Skip windows with fewer events; 0 runs all'),
    DeclareLaunchArgument('min_height', default_value='0.0', description='Metres; drop depth below it, 0 keeps all'),
    DeclareLaunchArgument('max_fit_change', default_value='0.0',
                          description='Reject fits straying this fraction from recent ones; 0 accepts all'),
    DeclareLaunchArgument('fit_hold_ms', default_value='1000',
                          description='Milliseconds the last floor fit is reused without a new one'),
    DeclareLaunchArgument('ransac_iterations', default_value='64'),
    DeclareLaunchArgument('ransac_tolerance', default_value='0.026',
                          description='Inlier band, as a fraction of the floor disparity spread'),
    DeclareLaunchArgument('min_inlier_share', default_value='0.25',
                          description='Share of floor candidates the fitted line must explain'),
    DeclareLaunchArgument('min_inliers', default_value='300', description='Fewest floor pixels a fit may rest on'),
]


def generate_launch_description():
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
                    parameters=[{
                        'input_topic': LaunchConfiguration('events_topic'),
                        'model_path': LaunchConfiguration('model_path'),
                        'window_ms': LaunchConfiguration('window_ms'),
                        'window_stride': LaunchConfiguration('window_stride'),
                        'max_events': LaunchConfiguration('max_events'),
                        'min_events': LaunchConfiguration('min_events'),
                        'sensor_width': 640,
                        'sensor_height': 480,
                        'publish_visualization': LaunchConfiguration('visualization_enable'),
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
                        'attitude_topic': ParameterValue(LaunchConfiguration('attitude_topic'), value_type=str),
                        'max_depth': LaunchConfiguration('max_depth'),
                        'undistort_depth': LaunchConfiguration('undistort_depth'),
                        'min_height': LaunchConfiguration('min_height'),
                        'max_fit_change': LaunchConfiguration('max_fit_change'),
                        'fit_hold_ms': LaunchConfiguration('fit_hold_ms'),
                        'ransac_iterations': LaunchConfiguration('ransac_iterations'),
                        'ransac_tolerance': LaunchConfiguration('ransac_tolerance'),
                        'min_inlier_share': LaunchConfiguration('min_inlier_share'),
                        'min_inliers': LaunchConfiguration('min_inliers'),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )
    )

    return ld
