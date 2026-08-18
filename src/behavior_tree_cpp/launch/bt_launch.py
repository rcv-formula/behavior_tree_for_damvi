from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    cartographer_ws = '/home/rcv/SLAM_main-SLAM_IMU_WHEEL_tun_upg'
    cartographer_launch_file = 'Damvi_carto_pure_wheel_launch.py'
    pbstream_filename = LaunchConfiguration('pbstream_filename')
    default_cartographer_launch = [
        'if command -v gnome-terminal >/dev/null 2>&1 && [ -n "$DISPLAY" ]; then '
        f'gnome-terminal --title "Cartographer Restart" -- bash -lc '
        f'"cd {cartographer_ws} && source install/setup.bash && '
        f'ros2 launch cartographer_ros {cartographer_launch_file} pbstream_filename:=',
        pbstream_filename,
        '"; '
        f'else cd {cartographer_ws} && source install/setup.bash && '
        f'ros2 launch cartographer_ros {cartographer_launch_file} pbstream_filename:=',
        pbstream_filename,
        '; fi'
    ]

    bypass_critical = LaunchConfiguration('bypass_critical')
    obstacle_mode = LaunchConfiguration('obstacle_mode')
    dynamic_min_speed_mps = LaunchConfiguration('dynamic_min_speed_mps')
    cartographer_restart_enabled = LaunchConfiguration('cartographer_restart_enabled')
    cartographer_restart_rf_topic = LaunchConfiguration('cartographer_restart_rf_topic')
    cartographer_restart_rf_channel = LaunchConfiguration('cartographer_restart_rf_channel')
    cartographer_restart_rf_off_max = LaunchConfiguration('cartographer_restart_rf_off_max')
    cartographer_restart_rf_min = LaunchConfiguration('cartographer_restart_rf_min')
    cartographer_restart_rf_max = LaunchConfiguration('cartographer_restart_rf_max')
    cartographer_restart_cooldown_sec = LaunchConfiguration('cartographer_restart_cooldown_sec')
    cartographer_restart_stop_delay_sec = LaunchConfiguration('cartographer_restart_stop_delay_sec')
    cartographer_stop_command = LaunchConfiguration('cartographer_stop_command')
    cartographer_launch_command = LaunchConfiguration('cartographer_launch_command')
    cartographer_launch_log_path = LaunchConfiguration('cartographer_launch_log_path')

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
            # RF-triggered cartographer restart. /rf is std_msgs/UInt16MultiArray.
            'cartographer_restart_enabled': ParameterValue(cartographer_restart_enabled, value_type=bool),
            'cartographer_restart_rf_topic': cartographer_restart_rf_topic,
            'cartographer_restart_rf_channel': ParameterValue(cartographer_restart_rf_channel, value_type=int),
            'cartographer_restart_rf_off_max': ParameterValue(cartographer_restart_rf_off_max, value_type=int),
            'cartographer_restart_rf_min': ParameterValue(cartographer_restart_rf_min, value_type=int),
            'cartographer_restart_rf_max': ParameterValue(cartographer_restart_rf_max, value_type=int),
            'cartographer_restart_cooldown_sec': ParameterValue(cartographer_restart_cooldown_sec, value_type=float),
            'cartographer_restart_stop_delay_sec': ParameterValue(cartographer_restart_stop_delay_sec, value_type=float),
            'cartographer_stop_command': cartographer_stop_command,
            'cartographer_launch_command': cartographer_launch_command,
            'cartographer_launch_log_path': cartographer_launch_log_path,
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
        DeclareLaunchArgument(
            'cartographer_restart_enabled',
            default_value='true',
            description='Enable RF-triggered cartographer launch restart'
        ),
        DeclareLaunchArgument(
            'cartographer_restart_rf_topic',
            default_value='/rf',
            description='RF topic to watch. Expected type: std_msgs/UInt16MultiArray'
        ),
        DeclareLaunchArgument(
            'cartographer_restart_rf_channel',
            default_value='9',
            description='Zero-based /rf channel index that triggers cartographer restart'
        ),
        DeclareLaunchArgument(
            'cartographer_restart_rf_off_max',
            default_value='1000',
            description='RF channel value treated as low/off for restart edge detection'
        ),
        DeclareLaunchArgument(
            'cartographer_restart_rf_min',
            default_value='2000',
            description='Minimum RF channel value treated as high/on for restart edge detection'
        ),
        DeclareLaunchArgument(
            'cartographer_restart_rf_max',
            default_value='65535',
            description='Maximum RF channel value for restart trigger'
        ),
        DeclareLaunchArgument(
            'cartographer_restart_cooldown_sec',
            default_value='1.0',
            description='Minimum seconds between cartographer restart requests'
        ),
        DeclareLaunchArgument(
            'cartographer_restart_stop_delay_sec',
            default_value='1.0',
            description='Delay after stopping cartographer before relaunching'
        ),
        DeclareLaunchArgument(
            'pbstream_filename',
            default_value='latest_l.pbstream',
            description='pbstream_filename argument passed to the cartographer launch file'
        ),
        DeclareLaunchArgument(
            'cartographer_stop_command',
            default_value=f"pkill -SIGINT -f 'ros2 launch cartographer_ros {cartographer_launch_file}' || true",
            description='Shell command used to stop the existing cartographer launch'
        ),
        DeclareLaunchArgument(
            'cartographer_launch_command',
            default_value=default_cartographer_launch,
            description='Shell command used to start cartographer launch'
        ),
        DeclareLaunchArgument(
            'cartographer_launch_log_path',
            default_value='/tmp/cartographer_restart.log',
            description='Log file for the relaunched cartographer process'
        ),
        health_monitor_node,
        bt_main_node,
    ])
