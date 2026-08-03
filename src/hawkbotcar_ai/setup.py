from setuptools import find_packages, setup

package_name = 'hawkbotcar_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'qrTracker = hawkbotcar_ai.qrTracker:main',
            'HandCtrlCar = hawkbotcar_ai.HandCtrlCar:main',
        ],
    },
)
