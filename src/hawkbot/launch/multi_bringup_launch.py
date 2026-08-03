"""
Multi-robot bringup launch file.

Launches multiple HB-01S robots simultaneously, each in its own ROS2 namespace
with isolated UDP ports to avoid bind() conflicts on the PC side.

Usage:
  # Two robots:
  ros2 launch hawkbot multi_bringup_launch.py ip1:=192.168.88.78 ip2:=192.168.88.81

  # Three robots:
  ros2 launch hawkbot multi_bringup_launch.py ip1:=192.168.88.78 ip2:=192.168.88.81 ip3:=192.168.88.132

  # Four robots:
  ros2 launch hawkbot multi_bringup_launch.py ip1:=192.168.88.78 ip2:=192.168.88.81 ip3:=192.168.88.132 ip4:=192.168.88.169

Each robot gets:
  - Namespace: /robot1, /robot2, /robot3, /robot4
  - UDP port offset: 0, 100, 200, 300 (to avoid port conflicts)
  - Its own TF tree, state publisher, IMU filter, lidar, localization
"""

import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import LaunchConfigurationNotEquals

import subprocess
import signal


# Port offsets between robot instances — large enough to avoid any overlap
PORT_OFFSETS = [10, 100, 200, 300]


def cleanup_stale_processes(context, *args, **kwargs):
    """Kill any leftover hawkbot_node / lidar_trans / udp_client processes
    from a previous launch session that didn't shut down cleanly."""
    patterns = [
        'hawkbot_node', 'lidar_trans', 'udp_client_process',
        'video_client_process', 'ydlidar_ros2_driver_node', 'lidar_demux'
    ]
    killed = []
    for pattern in patterns:
        try:
            result = subprocess.run(
                ['pgrep', '-f', pattern],
                capture_output=True, text=True, timeout=5
            )
            if result.stdout.strip():
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    pid = pid.strip()
                    if pid:
                        try:
                            os.kill(int(pid), signal.SIGKILL)
                            killed.append(f"{pattern}(pid={pid})")
                        except (ProcessLookupError, ValueError):
                            pass
        except subprocess.TimeoutExpired:
            pass
    if killed:
        return [LogInfo(msg=f'Cleaned up stale processes: {", ".join(killed)}')]
    return [LogInfo(msg='No stale processes found — clean start.')]


def generate_launch_description():
    cam_quality = LaunchConfiguration('cam_quality')
    robot_param = LaunchConfiguration('robot_param')

    declared_args = [
        DeclareLaunchArgument('cam_quality', default_value='7'),
        DeclareLaunchArgument('robot_param', default_value=''),
    ]

    # Declare IP arguments for up to 4 robots (empty = not used)
    for i in range(1, 5):
        declared_args.append(
            DeclareLaunchArgument(
                f'ip{i}',
                default_value='',
                description=f'IP address of robot {i} (leave empty to skip)',
            )
        )

    robot_groups = []

    # Clean up zombie processes from previous launches
    cleanup_action = OpaqueFunction(function=cleanup_stale_processes)

    # Start the Lidar UDP Demux Proxy node to route robot lidar data to correct ports
    demux_node = Node(
        package='hawkbot',
        executable='lidar_demux',
        name='lidar_demux',
        output='screen',
        arguments=[
            '--ip1', LaunchConfiguration('ip1'),
            '--ip2', LaunchConfiguration('ip2'),
            '--ip3', LaunchConfiguration('ip3'),
            '--ip4', LaunchConfiguration('ip4'),
        ]
    )

    bringup_launch_file = os.path.join(
        get_package_share_directory('hawkbot'), 'launch', 'bringup_launch.py'
    )

    for i in range(1, 5):
        robot_ip = LaunchConfiguration(f'ip{i}')
        robot_ns = f'robot{i}'
        port_offset = str(PORT_OFFSETS[i - 1])
        enable_lidar_val = 'True'

        # Include the single-robot bringup with namespace and port offset
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_file),
            launch_arguments={
                'ip': robot_ip,
                'cam_quality': cam_quality,
                'robot_param': robot_param,
                'robot_ns': robot_ns,
                'port_offset': port_offset,
                'cleanup': 'False',
                'enable_lidar': enable_lidar_val,
                'launch_demux': 'False',
            }.items(),
            condition=LaunchConfigurationNotEquals(f'ip{i}', ''),
        )

        # Stagger robot launches by 3 seconds each to reduce startup conflicts
        delayed_launch = TimerAction(
            period=float((i - 1) * 3),
            actions=[
                LogInfo(msg=f'Launching robot{i} (port_offset={port_offset})...'),
                robot_launch,
            ],
        )
        robot_groups.append(delayed_launch)

    return LaunchDescription(declared_args + [cleanup_action, demux_node] + robot_groups)
