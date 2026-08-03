import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution



def generate_launch_description():

    robot_ip = LaunchConfiguration('ip')
    cam_quality = LaunchConfiguration('cam_quality')
    robot_param = LaunchConfiguration('robot_param')
    ip_launch_arg = DeclareLaunchArgument(
        'ip', default_value=TextSubstitution(text='')
    )
    cam_quality_launch_arg = DeclareLaunchArgument(
        'cam_quality', default_value=TextSubstitution(text='7')
    )
    robot_param_launch_arg = DeclareLaunchArgument(
        'robot_param', default_value=TextSubstitution(text='')
    )

    robot_node1= LaunchDescription([
        Node(
            package='hawkbot',
            executable='hawkbot_node',
            output='screen',
            arguments=[robot_ip, "1", cam_quality,robot_param]
        )
    ])

    robot_node2= LaunchDescription([
        Node(
            package='hawkbot',
            executable='hawkbot_node',
            output='screen',
            arguments=[robot_ip, "2", cam_quality,robot_param]
        )
    ])
    imu_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('imu_complementary_filter'), 'launch'),
            '/complementary_filter.launch.py'])
    )


    ydlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ydlidar_ros2_driver'), 'launch'),
            '/ydlidar_launch.py']),
        launch_arguments={'robot_ip': robot_ip}.items(),
    )

    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf.launch.py'])
    )

    tf_trans_base_to_link = LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0','base_footprint', 'base_link']
        ),
    ])
    tf_trans_base_to_left_wheel = LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.035', '0.08', '0.006', '0.0', '0.0', '0.0', 'base_link', 'left_wheel_link']
        ),
    ])
    tf_trans_base_to_right_wheel = LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.0351', '-0.08', '0.006', '0.0', '0.0', '0.0', 'base_link', 'right_wheel_link']
        ),
    ])
    tf_trans_base_to_font_wheel = LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.10', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'font_wheel_link']
        ),
    ])
    tf_trans_base_link_to_laser = LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.01', '0.0', '0.06', '0.0', '0.0', '0.0', 'base_footprint', 'base_scan']
        ),
    ])
    tf_trans_base_to_camera = LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.12', '0.0', '0.04', '0.0', '0.0', '0.0', 'base_footprint', 'camera_link']
        ),
    ])

    tf_trans_base_to_gyro = LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'gyro_link']
        ),
    ])

    urdf = os.path.join(
        get_package_share_directory('hawkbot'),
        "hawkbot05.urdf.xacro")


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # parameters=[{'use_sim_time': 'false'}],
        arguments=[urdf]
    )

    return LaunchDescription([
        ip_launch_arg,
        cam_quality_launch_arg,
        robot_param_launch_arg,
        robot_node1,
        robot_node2,
        imu_filter_node,
        ydlidar_node,
        localization_node,
        tf_trans_base_to_link,
        tf_trans_base_link_to_laser,
        tf_trans_base_to_camera,
        tf_trans_base_to_font_wheel,
        tf_trans_base_to_gyro,
        tf_trans_base_to_left_wheel,
        tf_trans_base_to_right_wheel,
        robot_state_publisher_node,
    ])
