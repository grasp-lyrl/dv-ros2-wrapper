import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

bag_args = [
    DeclareLaunchArgument('model_path', description='AOTI .pt2 from export_f3_aoti.py'),
    DeclareLaunchArgument('bag', default_value='',
                          description='Bag to replay once; leave empty to play one yourself'),
    DeclareLaunchArgument('events_topic', default_value='/neurofly1/events'),
    DeclareLaunchArgument('visualization_enable', default_value='true'),
    DeclareLaunchArgument('rviz', default_value='true'),
]


def generate_launch_description():
    """Monodepth over a recorded event stream -- no camera, so no capture node."""
    share = FindPackageShare('dv_ros2_monodepth').find('dv_ros2_monodepth')
    qos_override = os.path.join(share, 'config', 'bag_qos_override.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'monodepth.rviz')

    ld = LaunchDescription(bag_args)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'monodepth.launch.py')),
        launch_arguments={
            'model_path': LaunchConfiguration('model_path'),
            'events_topic': LaunchConfiguration('events_topic'),
            'capture': 'false',
        }.items()))

    ld.add_action(LoadComposableNodes(
        condition=IfCondition(LaunchConfiguration('visualization_enable')),
        target_container='monodepth_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dv_ros2_visualization', plugin='dv_visualization_node::VisualizationNode',
                name='visualization_node',
                remappings=[('events', LaunchConfiguration('events_topic'))],
                extra_arguments=[{'use_intra_process_comms': True}]),
        ]))

    # Delayed so the model is loaded and warm before events arrive.
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
