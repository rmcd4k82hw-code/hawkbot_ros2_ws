import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hawkbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hawkbot',
    maintainer_email='hawkbot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hawkbot_node = hawkbot.hawkbot_node:main',
            'teleop_keyboard = hawkbot.teleop_key:main',
            'lidar_demux = hawkbot.lidar_demux:main',
            'cmd_vel_relay = hawkbot.cmd_vel_relay:main',
            'follower_controller = hawkbot.follower_controller:main',
            'wait_and_launch_ydlidar = hawkbot.wait_and_launch_ydlidar:main',
            'odom_corrector = hawkbot.odom_corrector:main',
            'cartographer_initial_pose_bridge = hawkbot.cartographer_initial_pose_bridge:main',
            'scan_match_monitor = hawkbot.scan_match_monitor:main',
            'visual_odom_monitor = hawkbot.visual_odom_monitor:main',
            'localization_debug = hawkbot.localization_debug:main',
            'kinematic_tf_filter = hawkbot.kinematic_tf_filter:main',
            'trajectory_recorder = hawkbot.trajectory_recorder:main',
            'trajectory_control_menu = hawkbot.trajectory_control_menu:main',
            'route_follower = hawkbot.route_follower:main',
            'odom_recalibrator = hawkbot.odom_recalibrator:main',
            'trail_verifier = hawkbot.trail_verifier:main',
            'follow_leader = hawkbot.follow_leader:main',
            'follow_leader_menu = hawkbot.follow_leader_menu:main'
        ],
    },
)
