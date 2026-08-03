import os
import yaml
import subprocess
import signal
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

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

def clean_yaml_dict(d):
    """Recursively clean up the yaml dictionary to prevent ROS2 type errors.
    Specifically, PyYAML might parse '1e-9' as a string, causing mixed-type lists.
    """
    if isinstance(d, dict):
        return {k: clean_yaml_dict(v) for k, v in d.items()}
    elif isinstance(d, list):
        # Check if it's a mixed list of floats and strings that can be floats
        cleaned_list = []
        is_float_list = True
        for item in d:
            if isinstance(item, bool):
                is_float_list = False
                break
            elif isinstance(item, float):
                cleaned_list.append(item)
            elif isinstance(item, int):
                cleaned_list.append(float(item))
            elif isinstance(item, str):
                try:
                    cleaned_list.append(float(item))
                except ValueError:
                    is_float_list = False
                    break
            else:
                is_float_list = False
                break
        
        if is_float_list:
            return cleaned_list
        else:
            return [clean_yaml_dict(item) for item in d]
    return d

def load_yaml_params(file_path, node_name):
    with open(file_path, 'r') as f:
        config = yaml.safe_load(f)
        if node_name in config and 'ros__parameters' in config[node_name]:
            params = config[node_name]['ros__parameters']
            return clean_yaml_dict(params)
        return {}

def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('ip').perform(context)
    cam_quality = LaunchConfiguration('cam_quality').perform(context)
    robot_param = LaunchConfiguration('robot_param').perform(context)
    robot_ns = LaunchConfiguration('robot_ns').perform(context)
    port_offset = LaunchConfiguration('port_offset').perform(context)
    cleanup = LaunchConfiguration('cleanup').perform(context)
    enable_lidar = LaunchConfiguration('enable_lidar').perform(context)
    launch_demux = LaunchConfiguration('launch_demux').perform(context)

    nodes = []

    # Clean up stale processes before launching new nodes (if cleanup enabled)
    if cleanup.lower() in ('true', '1', 'yes'):
        nodes.append(OpaqueFunction(function=cleanup_stale_processes))

    # Frame prefix handling
    frame_prefix = f"{robot_ns}/" if robot_ns else ""

    # Topic remappings for HBSDK absolute topics.
    # HBSDK internally creates nodes that publish/subscribe to absolute topics
    # (e.g. /imu/data_raw, /odom). When using namespaces, we need to remap
    # these into the robot's namespace so downstream nodes can find them.
    if robot_ns:
        hbsdk_remappings = [
            ('/imu/data_raw', f'/{robot_ns}/imu/data_raw'),
            ('/odom', f'/{robot_ns}/odom'),
            ('/battery', f'/{robot_ns}/battery'),
            ('/motor_speed', f'/{robot_ns}/motor_speed'),
            ('/cmd_vel', f'/{robot_ns}/cmd_vel'),
            ('/image_raw/compressed', f'/{robot_ns}/image_raw/compressed'),
            ('/image/image_raw', f'/{robot_ns}/image/image_raw'),
            ('/robot_param', f'/{robot_ns}/robot_param'),
            ('/sound', f'/{robot_ns}/sound'),
            ('/servo', f'/{robot_ns}/servo'),
            ('/pid', f'/{robot_ns}/pid'),
        ]
    else:
        hbsdk_remappings = []

    # 1. HawkBot nodes
    nodes.append(Node(
        package='hawkbot',
        executable='hawkbot_node',
        output='screen',
        arguments=[robot_ip, "1", cam_quality, robot_param, port_offset],
        namespace=robot_ns,
        remappings=hbsdk_remappings,
    ))

    # Video node (type 2) uses same port offset and remappings as type 1.
    # It fails to bind to telemetry sockets, preventing duplicate telemetry
    # while successfully starting camera streams.
    nodes.append(Node(
        package='hawkbot',
        executable='hawkbot_node',
        output='screen',
        arguments=[robot_ip, "2", cam_quality, robot_param, port_offset],
        namespace=robot_ns,
        remappings=hbsdk_remappings,
    ))

    # 2. IMU Complementary Filter
    nodes.append(Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        namespace=robot_ns,
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[
            ('/imu/data_raw', f'/{robot_ns}/imu/data_raw'),
            ('/imu/data', f'/{robot_ns}/imu/data'),
        ] if robot_ns else []
    ))

    # 3. Odom Corrector Node (republishes IMU and Odom with corrected frame IDs and covariances)
    if robot_ns:
        nodes.append(Node(
            package='hawkbot',
            executable='odom_corrector',
            name='odom_corrector',
            output='screen',
            namespace=robot_ns,
            parameters=[{'robot_ns': robot_ns, 'publish_tf': False}]
        ))

    # 4. YDLidar
    if enable_lidar.lower() in ('true', '1', 'yes'):
        if launch_demux.lower() in ('true', '1', 'yes'):
            nodes.append(Node(
                package='hawkbot',
                executable='lidar_demux',
                name='lidar_demux',
                output='screen',
                arguments=['--ip1', robot_ip]
            ))

        share_dir_ydlidar = get_package_share_directory('ydlidar_ros2_driver')
        ydlidar_yaml_path = os.path.join(share_dir_ydlidar, 'params', 'ydlidar.yaml')
        ydlidar_params = load_yaml_params(ydlidar_yaml_path, 'ydlidar_ros2_driver_node')
        ydlidar_params['robot_ip'] = robot_ip
        ydlidar_params['frame_id'] = f'{frame_prefix}base_scan'

        nodes.append(Node(
            package='hawkbot',
            executable='wait_and_launch_ydlidar',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            namespace=robot_ns,
            arguments=[robot_ip],
            parameters=[ydlidar_params],
            remappings=[('/scan', f'/{robot_ns}/scan')] if robot_ns else [],
        ))

    # 5. Robot Localization (EKF)
    share_dir_ekf = get_package_share_directory('robot_localization')
    ekf_yaml_path = os.path.join(share_dir_ekf, 'params', 'ekf.yaml')
    ekf_params = load_yaml_params(ekf_yaml_path, 'ekf_filter_node')
    ekf_params['map_frame'] = f'{frame_prefix}map'
    ekf_params['odom_frame'] = f'{frame_prefix}odom_combined'
    ekf_params['base_link_frame'] = f'{frame_prefix}base_footprint'
    ekf_params['world_frame'] = f'{frame_prefix}odom_combined'

    # Remap inputs to the ones corrected by odom_corrector if namespace is active
    if robot_ns:
        ekf_params['odom0'] = 'odom_corrected'
        ekf_params['imu0'] = 'imu/data_corrected'
    else:
        ekf_params['odom0'] = 'odom'
        ekf_params['imu0'] = 'imu/data'

    nodes.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        namespace=robot_ns,
        parameters=[ekf_params],
    ))

    # 6. Static Transforms
    tf_transforms = [
        ('0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link'),
        ('-0.011', '0.04', '0.025', '0.0', '0.0', '0.0', 'base_link', 'left_wheel_link'),
        ('-0.011', '-0.04', '0.025', '0.0', '0.0', '0.0', 'base_link', 'right_wheel_link'),
        ('-0.05', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'font_wheel_link'),
        ('-0.009', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_scan'),
        ('0.06', '0.0', '0.06', '0.0', '0.0', '0.0', 'base_footprint', 'camera_link'),
        ('0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'gyro_link'),
    ]

    for tx, ty, tz, rx, ry, rz, parent, child in tf_transforms:
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[tx, ty, tz, rx, ry, rz, f'{frame_prefix}{parent}', f'{frame_prefix}{child}'],
            namespace=robot_ns,
        ))

    # 7. Robot State Publisher
    urdf = os.path.join(get_package_share_directory('hawkbot'), "hawkbot.urdf.xacro")
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_ns,
        parameters=[{'frame_prefix': frame_prefix}],
        arguments=[urdf]
    ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ip', default_value=TextSubstitution(text='')),
        DeclareLaunchArgument('cam_quality', default_value=TextSubstitution(text='7')),
        DeclareLaunchArgument('robot_param', default_value=TextSubstitution(text='')),
        DeclareLaunchArgument('robot_ns', default_value=TextSubstitution(text='')),
        DeclareLaunchArgument('port_offset', default_value=TextSubstitution(text='10')),
        DeclareLaunchArgument('cleanup', default_value=TextSubstitution(text='True')),
        DeclareLaunchArgument('enable_lidar', default_value=TextSubstitution(text='True')),
        DeclareLaunchArgument('launch_demux', default_value=TextSubstitution(text='True')),
        OpaqueFunction(function=launch_setup)
    ])
