from setuptools import setup
from glob import glob

package_name = 'robot_brain'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/launch', ['launch/distance_keeper.launch.py']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Distance keeping logic driven by obstacle_detector /obstacles',
    license='MIT',
    entry_points={
        'console_scripts': [
            'distance_keeper = robot_brain.distance_keeper:main',
            'relay_qos = robot_brain.relay_qos:main',
            'follow_signal_publisher = robot_brain.follow_signal_publisher:main',
            'joy_to_serial = robot_brain.joy_to_serial:main'
        ],
    },
)
