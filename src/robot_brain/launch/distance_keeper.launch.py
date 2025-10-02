from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    cfg = join(get_package_share_directory('robot_brain'), 'config', 'distance_keeper.yaml')
    return LaunchDescription([
        Node(
            package='robot_brain',
            executable='distance_keeper',
            name='robot_brain',
            output='screen',
            parameters=[cfg],
        )
    ])
