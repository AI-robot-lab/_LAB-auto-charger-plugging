from setuptools import setup

package_name = 'mission_control'

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
    description='State machine for coordinating the charger plugging mission',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = mission_control.state_machine:main',
            'state_machine_action_client = mission_control.state_machine_action_client:main',
        ],
    },
)
