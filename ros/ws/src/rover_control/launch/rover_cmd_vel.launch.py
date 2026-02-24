from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('server_url', default_value='http://127.0.0.1:5000'),
        DeclareLaunchArgument('server_url_file', default_value='/app/ros/.server_url'),
        DeclareLaunchArgument('url_refresh_sec', default_value='1.0'),
        DeclareLaunchArgument('command_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('timer_hz', default_value='10.0'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.6'),
        DeclareLaunchArgument('min_speed_pct', default_value='20'),
        DeclareLaunchArgument('max_speed_pct', default_value='60'),
        DeclareLaunchArgument('forward_gear', default_value='1'),
        DeclareLaunchArgument('auto_engine_start', default_value='true'),
        DeclareLaunchArgument('auto_release_emergency_brake', default_value='true'),
        DeclareLaunchArgument('control_rearm_sec', default_value='1.0'),
        Node(
            package='rover_control',
            executable='cmd_vel_bridge',
            name='rover_cmd_vel_bridge',
            output='screen',
            parameters=[{
                'server_url': LaunchConfiguration('server_url'),
                'server_url_file': LaunchConfiguration('server_url_file'),
                'url_refresh_sec': LaunchConfiguration('url_refresh_sec'),
                'command_topic': LaunchConfiguration('command_topic'),
                'timer_hz': LaunchConfiguration('timer_hz'),
                'command_timeout_sec': LaunchConfiguration('command_timeout_sec'),
                'min_speed_pct': LaunchConfiguration('min_speed_pct'),
                'max_speed_pct': LaunchConfiguration('max_speed_pct'),
                'forward_gear': ParameterValue(LaunchConfiguration('forward_gear'), value_type=str),
                'auto_engine_start': LaunchConfiguration('auto_engine_start'),
                'auto_release_emergency_brake': LaunchConfiguration('auto_release_emergency_brake'),
                'control_rearm_sec': LaunchConfiguration('control_rearm_sec'),
            }],
        ),
    ])
