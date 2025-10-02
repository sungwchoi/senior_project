from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_obstacle_detector = get_package_share_directory('obstacle_detector')

    # 1. 컴포넌트들을 담을 '컨테이너'를 정의합니다.
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # 2. 컨테이너에 로드할 VoxelGrid 필터 '컴포넌트'를 정의합니다.
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::VoxelGrid', # 실행 파일 이름이 아닌 플러그인 클래스 이름
                    name='voxel_grid_filter',
                    parameters=[{'leaf_size': 0.1}],
                    remappings=[
                        ('input', '/velodyne_points'),
                        ('output', '/velodyne_points_filtered')
                    ]
                ),
            ],
            output='screen',
    )

    # 3. 기존의 obstacle_detector launch 파일을 포함시킵니다.
    obstacle_detector_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_obstacle_detector, 'launch', 'obstacle_extractor_and_tracker.launch')
        )
    )

    return LaunchDescription([
        container,
        obstacle_detector_launch
    ])