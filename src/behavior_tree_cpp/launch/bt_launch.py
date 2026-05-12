from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bypass_critical = LaunchConfiguration('bypass_critical')

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
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bypass_critical',
            default_value='false',
            description='Bypass CondCriticalOK guard in bt_main'
        ),
        health_monitor_node,
        bt_main_node,
    ])
