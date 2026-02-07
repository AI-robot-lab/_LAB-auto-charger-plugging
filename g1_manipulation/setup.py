from setuptools import setup

package_name = 'g1_manipulation'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Arm IK control for grasping charger and inserting plug',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = g1_manipulation.arm_controller:main',
            'gripper_controller = g1_manipulation.gripper_controller:main',
        ],
    },
)
