"""
Multi-robot navigation launch using Cartographer Pure Localization.

Replaces AMCL and map_server with Cartographer localization nodes,
reusing the user's working cartography setup for precise scan matching.

Prerequisites:
  1. A Cartographer map must be serialized as a .pbstream file:
     /home/hawkbot/ROS2_WS/src/hawkbot/map/hawkbot.pbstream
     (Run your normal cartography setup, then save it using:
      ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
        "{filename: '/home/hawkbot/ROS2_WS/src/hawkbot/map/hawkbot.pbstream', include_unfinished_submaps: true}")
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hawkbot_share = get_package_share_directory('hawkbot')
    pbstream_path = os.path.join(hawkbot_share, 'map', 'hawkbot.pbstream')
    if not os.path.exists(pbstream_path):
        pbstream_path = os.path.expanduser('~/ROS2_WS/src/hawkbot/map/hawkbot.pbstream')

    static_map_yaml = os.path.join(hawkbot_share, 'map', 'hawkbot_static.yaml')
    if not os.path.exists(static_map_yaml):
        static_map_yaml = os.path.expanduser('~/ROS2_WS/src/hawkbot/map/hawkbot_static.yaml')

    # Locate hawkbot_cartographer configurations
    cartographer_config_dir = os.path.join(
        get_package_share_directory('hawkbot_cartographer'), 'config'
    )
    if not os.path.exists(cartographer_config_dir):
        cartographer_config_dir = os.path.expanduser('~/ROS2_WS/src/hawkbot_cartographer/config')

    ld = LaunchDescription()
    
    # Global Static Map Server for the scan match monitors
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': static_map_yaml, 'frame_id': 'map'}],
        remappings=[('map', 'static_map')]
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    
    # Launch Cartographer localization for robot1 and robot2
    for i in [1, 2]:
        ns = f"robot{i}"
        
        # Select the robot-specific lua config
        lua_config_file = f"lidar_robot{i}_localization.lua"
        
        # 1. Cartographer Node (pure localization mode)
        cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            namespace=ns,
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', lua_config_file,
                '-load_state_filename', pbstream_path
            ],
            remappings=[
                ('imu', 'imu/data_corrected'),
                ('odom', 'odom_corrected')
            ]
        )
        
        # 2. Cartographer Occupancy Grid Node (publishes the map topic)
        cartographer_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            namespace=ns,
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        )
        
        # 3. Cartographer Initial Pose Bridge (enables 2D Pose Estimate button in RViz)
        initial_pose_bridge_node = Node(
            package='hawkbot',
            executable='cartographer_initial_pose_bridge',
            name='cartographer_initial_pose_bridge',
            output='screen',
            namespace=ns,
            parameters=[{
                'robot_ns': ns,
                'configuration_directory': cartographer_config_dir,
                'configuration_basename': lua_config_file,
            }]
        )
        
        # 4. Scan Match Monitor (compares laser scan to occupancy grid map)
        scan_match_monitor_node = Node(
            package='hawkbot',
            executable='scan_match_monitor',
            name='scan_match_monitor',
            output='screen',
            namespace=ns,
            parameters=[{'robot_ns': ns}]
        )
        
        # 5. Visual Odom Monitor (camera optical flow vs TF drift detection)
        visual_odom_monitor_node = Node(
            package='hawkbot',
            executable='visual_odom_monitor',
            name='visual_odom_monitor',
            output='screen',
            namespace=ns,
            parameters=[{'robot_ns': ns}]
        )
        
        # 6. Trajectory Recorder (records blue trail, handles loop closure & smoothing)
        trajectory_recorder_node = Node(
            package='hawkbot',
            executable='trajectory_recorder',
            name='trajectory_recorder',
            output='screen',
            namespace=ns,
            parameters=[{'robot_ns': ns}]
        )
        
        # 7. Route Follower (Pure Pursuit path follower on recorded blue loop)
        route_follower_node = Node(
            package='hawkbot',
            executable='route_follower',
            name='route_follower',
            output='screen',
            namespace=ns,
            parameters=[{'robot_ns': ns}]
        )

        # 8. Odom Recalibrator (Resets odom_combined origin to robot base footprint when distance > 0.5m)
        odom_recalibrator_node = Node(
            package='hawkbot',
            executable='odom_recalibrator',
            name='odom_recalibrator',
            output='screen',
            namespace=ns,
            parameters=[{'robot_ns': ns, 'max_odom_distance': 0.5}]
        )

        ld.add_action(cartographer_node)
        ld.add_action(cartographer_grid_node)
        ld.add_action(initial_pose_bridge_node)
        ld.add_action(scan_match_monitor_node)
        ld.add_action(visual_odom_monitor_node)
        ld.add_action(trajectory_recorder_node)
        ld.add_action(route_follower_node)
        ld.add_action(odom_recalibrator_node)
        
    # 9. Follow-the-Leader node (Robot2 follows Robot1's blue trail)
    follow_leader_node = Node(
        package='hawkbot',
        executable='follow_leader',
        name='follow_leader',
        output='screen',
        namespace='robot2',
        parameters=[{
            'leader_ns': 'robot1',
            'follower_ns': 'robot2',
            'min_separation': 0.35,
        }]
    )
    ld.add_action(follow_leader_node)

    # Launch RViz configuration for multi-robot navigation
    rviz_config = os.path.join(
        get_package_share_directory('hawkbot_navigation2'), 'rviz', 'hawkbot_navigation2.rviz'
    )
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rviz', 'hawkbot_navigation2.rviz'
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
    )
    ld.add_action(rviz_node)
        
    return ld
