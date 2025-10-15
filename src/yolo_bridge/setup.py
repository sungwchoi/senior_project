from setuptools import setup

package_name = 'yolo_bridge'

setup(
    name='yolo_bridge',
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/yolo.yaml']),
        ('share/' + package_name + '/launch', ['launch/yolo_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='you',
    maintainer_email='you@example.com',
    description='YOLOv5 inference node (ROS2)',
    license='MIT',
    scripts=['scripts/yolo_node'],
    #entry_points={'console_scripts': ['yolo_node = yolo_bridge.yolo_node:main']},
)
