from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='todo@example.com',
    description='Vision system for detecting charger handle and car charging port',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'charger_detector = perception.charger_detector:main',
            'port_detector = perception.port_detector:main',
            'perception_action_server = perception.perception_action_server:main',
            'lidar_safety_node = perception.lidar_safety_node:main',
            'yolo_detector_node = perception.yolo_detector_node:main',
            'mediapipe_detector_node = perception.mediapipe_detector_node:main',
        ],
    },
)
