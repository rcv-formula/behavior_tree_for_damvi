from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bypass_critical = LaunchConfiguration('bypass_critical')
    obstacle_mode = LaunchConfiguration('obstacle_mode')
    dynamic_min_speed_mps = LaunchConfiguration('dynamic_min_speed_mps')

    health_monitor_node = Node(
        package='behavior_tree_cpp',
        executable='health_monitor',
        name='health_monitor',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    bt_main_node = Node(
        package='behavior_tree_cpp',
        executable='bt_main',
        name='bt_main',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[{
            # 테스트가 아니라 실차면 보통 false
            'bypass_critical': ParameterValue(bypass_critical, value_type=bool),
            # 0=auto, 1=no obstacle, 2=dynamic only, 3=dynamic+static
            'obstacle_mode': ParameterValue(obstacle_mode, value_type=int),
            # Only mode 3 uses this to avoid treating nearly-static objects as dynamic.
            'dynamic_min_speed_mps': ParameterValue(dynamic_min_speed_mps, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bypass_critical',
            default_value='false',
            description='Bypass CondCriticalOK guard in bt_main'
        ),
        DeclareLaunchArgument(
            'obstacle_mode',
            default_value='0',
            description='0=auto, 1=no obstacle, 2=dynamic only, 3=dynamic+static'
        ),
        DeclareLaunchArgument(
            'dynamic_min_speed_mps',
            default_value='0.0',
            description='Mode 3 only: minimum /dynamic_obstacle speed required for dynamic judgment'
        ),
        health_monitor_node,
        bt_main_node,
    ])
