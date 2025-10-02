import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 각 패키지의 launch 파일이 있는 디렉토리 경로를 찾아옵니다.
    pkg_velodyne_driver = get_package_share_directory('velodyne_driver')
    pkg_velodyne_pointcloud = get_package_share_directory('velodyne_pointcloud')
    pkg_obstacle_detector = get_package_share_directory('obstacle_detector')

    # 1. Velodyne 드라이버 실행 launch 파일을 포함시킵니다.
    velodyne_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_velodyne_driver, 'launch', 'velodyne_driver_node-VLP16-launch.py')
        )
    )

    # 2. Velodyne 변환 노드 실행 launch 파일을 포함시킵니다.
    velodyne_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_velodyne_pointcloud, 'launch', 'velodyne_transform_node-VLP16-launch.py')
        )
    )

    # 3. 우리가 만들었던 필터 및 장애물 탐지기 실행 launch 파일을 포함시킵니다.
    filter_and_detect_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
            os.path.join(pkg_obstacle_detector, 'launch', 'filter_and_detect.launch.py')
         )
    )

    # 위에서 정의한 3개의 launch 파일을 모두 실행하도록 등록합니다.
    return LaunchDescription([
        velodyne_driver_launch,
        velodyne_pointcloud_launch,
        filter_and_detect_launch
    ])