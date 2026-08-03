from setuptools import find_packages, setup

package_name = 'hawkbot_mediapipe'

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
            'HandDetector = hawkbot_mediapipe.HandDetector:main',
            'PoseDetector = hawkbot_mediapipe.PoseDetector:main',
            'Holistic = hawkbot_mediapipe.Holistic:main',
            'FaceMesh = hawkbot_mediapipe.FaceMesh:main',
            'FaceEyeDetection = hawkbot_mediapipe.FaceEyeDetection:main',
            'FaceLandmarks = hawkbot_mediapipe.FaceLandmarks:main',
            'FaceDetection = hawkbot_mediapipe.FaceDetection:main',
            'Objectron = hawkbot_mediapipe.Objectron:main',
            'VirtualPaint = hawkbot_mediapipe.VirtualPaint:main',
            'HandCtrl = hawkbot_mediapipe.HandCtrl:main',
            'GestureRecognition = hawkbot_mediapipe.GestureRecognition:main',
            
            
            
        ],
    },
)
