from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import launch.actions
import launch_ros.actions
from launch_ros.actions import Node
import os


def generate_launch_description():
    rviz_config = "/home/hawkbot/ROS2_WS/src/slam_gmapping/rviz/slam.rviz"

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        ),
        launch_ros.actions.Node(
            package='slam_gmapping', executable='slam_gmapping', output='screen', parameters=[{'use_sim_time':use_sim_time}]),
    ])
