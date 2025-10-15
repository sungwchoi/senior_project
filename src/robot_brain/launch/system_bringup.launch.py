# system_bringup.launch.py
# 한 번에: velodyne driver -> transform -> obstacle_detector -> relay_qos -> distance_keeper -> rviz2
# 충돌/초기화 순서를 줄이기 위해 TimerAction으로 순차 기동, respawn, 네임스페이스, 스위치 인자 제공

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node as RosNode  # 별칭(위의 Node와 이름 충돌 피함)

def generate_launch_description():
    # ----- 공통 인자 -----
    ns               = LaunchConfiguration('namespace')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    use_rviz         = LaunchConfiguration('use_rviz')
    rviz_config      = LaunchConfiguration('rviz_config')
    enable_velodyne  = LaunchConfiguration('enable_velodyne')
    enable_detector  = LaunchConfiguration('enable_detector')
    enable_yolo_detector = LaunchConfiguration('enable_yolo_detector') # <<< 추가: YOLO 감지기 스위치
    enable_qos       = LaunchConfiguration('enable_qos')
    enable_keeper    = LaunchConfiguration('enable_keeper')
    enable_camera = LaunchConfiguration('enable_camera')

    # Velodyne 네트워크/프레임 설정
    velodyne_frame   = LaunchConfiguration('velodyne_frame')
    velodyne_ip      = LaunchConfiguration('velodyne_ip')
    velodyne_port    = LaunchConfiguration('velodyne_port')

    # ----- 패키지/런치 경로 -----
    # /opt/ros/humble/share/** 에 설치된 공식 런치를 그대로 재사용
    velodyne_driver_launch = PathJoinSubstitution([
        FindPackageShare('velodyne_driver'), 'launch', 'velodyne_driver_node-VLP16-launch.py' 
    ]) # 
    velodyne_transform_launch = PathJoinSubstitution([
        FindPackageShare('velodyne_pointcloud'), 'launch', 'velodyne_transform_node-VLP16-launch.py'
    ]) # 

    # ⚠️ 여기 패키지 이름 주의!
    # 네 폴더명은 obstacle_detector_2 이지만, 실제 package.xml의 <name>이 대부분 "obstacle_detector" 로 되어 있음.
    # 네가 평소에 사용하던 명령이 `ros2 launch obstacle_detector obstacle_extractor_and_tracker_2D.launch` 였다면
    # 아래 FindPackageShare('obstacle_detector') 그대로 두면 됨.
    # 만약 package.xml 이름이 정말 "obstacle_detector_2" 라면 'obstacle_detector' 를 'obstacle_detector_2' 로 바꿔줘.
    obstacle_pkg_share = FindPackageShare('obstacle_detector')  # ← 필요 시 'obstacle_detector_2' 로 변경
    obstacle_launch = PathJoinSubstitution([
        obstacle_pkg_share, 'launch', 'obstacle_extractor_and_tracker_2D.launch'
    ])

    yolo_bridge_launch = PathJoinSubstitution([
        FindPackageShare('yolo_bridge'), 'launch', 'yolo_bridge.launch.py'
    ])

    vlp16_calib = PathJoinSubstitution([   
        FindPackageShare('velodyne_pointcloud'), 'params', 'VLP16db.yaml'
    ])

    static_map_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_base_link',
        output='screen',
        arguments=['0','0','0','0','0','0','map','base_link']
    )


    velodyne_driver = Node(
        condition=IfCondition(enable_velodyne),
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='screen',
        parameters=[{
            'device_ip': velodyne_ip,
            'port': velodyne_port,
            'frame_id': velodyne_frame,
            'model': 'VLP16',
            'rpm': 500.0,  # 네가 원하는 500 RPM
            'gps_time': False,
            'time_offset': 0.0,
            'enabled': True,
            'read_once': False,
            'read_fast': False,
            'repeat_delay': 0.0,
            'timestamp_first_packet': False
        }],
        #namespace=ns
    )


    velodyne_transform = TimerAction(
        condition=IfCondition(enable_velodyne),
        period=2.0,
        actions=[
            Node(
                package='velodyne_pointcloud',
                executable='velodyne_transform_node',
                name='velodyne_transform_node',
                output='screen',
                parameters=[{
                    'model': 'VLP16',
                    'calibration': vlp16_calib,
                    'frame_id': velodyne_frame,
                    'use_sim_time': use_sim_time
                }],
            # relative 이름으로 고정 → PushRosNamespace('robot') 아래서 /robot 접두사 자동 부착
                remappings=[
                    ('velodyne_packets', '/robot/velodyne_packets'),
                    ('velodyne_points',  '/robot/velodyne_points'),
                ]
            )
        ]   
    )


    

    pcl2scan = TimerAction(
        condition=IfCondition(LaunchConfiguration('enable_pcl2scan')),   # Velodyne 켜질 때만
        period=2.5,  # transform(2.0s) 뜬 다음 조금 여유
        actions=[
            RosNode(
                package='pointcloud_to_laserscan',
                executable=LaunchConfiguration('pcl2scan_exec'),
                name='pointcloud_to_laserscan',
                output='screen',
                parameters=[{
                # LaserScan을 어느 frame 기준으로 만들지.
                # TF가 준비 안됐으면 velodyne_frame(=센서 프레임)으로 두는 게 가장 안전.
                    'target_frame': velodyne_frame,   # 'base_link'로 하고 싶으면 TF가 있어야 함
                    'transform_tolerance': 0.01,

                # Z-슬라이스: target_frame 좌표계에서 이 높이 구간의 포인트만 LaserScan에 반영
                    'min_height': -0.50,   # 바닥 근처 노이즈 컷
                    'max_height':  1.5,

                # 스캔 각도/해상도(라디안). 필요 시 조정.
                    'angle_min': -1.5708 ,
                    'angle_max':  1.5708,
                    'angle_increment': 0.003,  # ≈0.17° (원하면 0.00436 ≈0.25°)

                    'range_min': 0.30,
                    'range_max': 2.75,
                    'use_inf': True
                }],
                remappings=[
                    ('cloud_in', '/robot/velodyne_points'),  # 입력: 변환된 포인트클라우드
                    ('scan',  '/robot/scan'),             # 출력: 원하는 LaserScan 토픽
                ]
            )
        ]
    )



    obstacle_extractor = TimerAction(
        condition=IfCondition(enable_detector),
        period=3.5,
        actions=[
            Node(
                package='obstacle_detector',
                executable='obstacle_extractor_node',
                name='obstacle_extractor',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            # 입력 LaserScan만 정확히 리맵
                remappings=[('scan', '/robot/scan_reliable'),
                            ('obstacles', '/robot/obstacles_raw')]
            )
        ]
    )

    obstacle_tracker = TimerAction(
        condition=IfCondition(enable_detector),
        period=4.0,  # extractor보다 약간 뒤에 시작
        actions=[
            Node(
                package='obstacle_detector',
                executable='obstacle_tracker_node',
                name='obstacle_tracker',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=[('obstacles', '/robot/obstacles_raw'),
                        # 최종 출력 토픽(원하면 기존 이름으로)
                            ('tracked_obstacles', '/robot/obstacles')]
            )
        ]
    )

    camera_node = Node(
        condition=IfCondition(enable_camera),
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
    # 네임스페이스 그룹 안에 있으므로 토픽은 자동으로 /robot/image_raw 가 됩니다.
    # YOLO가 /camera/image_raw 를 구독하므로 리맵핑이 필요합니다.
        remappings=[
            ('/image_raw', '/camera/image_raw'), 
        ]
    )


    yolo_detector = TimerAction(
        condition=IfCondition(enable_yolo_detector),
        period=1.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(yolo_bridge_launch),
                # yolo_bridge.launch.py에 넘겨줄 인자가 있다면 여기에 추가할 수 있습니다.
                # launch_arguments={'arg_name': 'value'}.items()
            )
        ]
    )


    # ----- robot_brain: relay_qos -----
    # 토픽 QoS 릴레이. 센서/처리체인 연결 이슈 있을 때 유용. 죽으면 재시작.
    relay_qos = Node(
        condition=IfCondition(enable_qos),
        package='robot_brain',
        executable='relay_qos',
        name='relay_qos',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'in_topic':  'scan',           # 절대 토픽으로 지정해도 OK
            'out_topic': 'scan_reliable',
        
        }],
        #remappings=[]
    )

    # ----- robot_brain: distance_keeper -----
    # 네 패키지의 config/distance_keeper.yaml을 로드 (필요 시 파라미터 키 확인)
    distance_keeper = Node(
        condition=IfCondition(enable_keeper),
        package='robot_brain',
        executable='distance_keeper',
        name='distance_keeper',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        # ▼▼▼ 실제 파라미터 파일 경로 맞추기 ▼▼▼
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('robot_brain'), 'config', 'distance_keeper.yaml'])
            # /opt/… 가 아니라 워크스페이스 빌드 후 share에서 참조하려면 FindPackageShare 사용 권장
            # 여기서는 개발 편의상 상대경로/절대경로 모두 허용. 문제 시 아래 두 줄 중 하나 선택:
            # Path 1) 설치 경로 참조:
            # PathJoinSubstitution([FindPackageShare('robot_brain'), 'config', 'distance_keeper.yaml']),
            # Path 2) 소스 트리 절대경로 직접 지정(예시):
            # '/home/…/senior_project/src/robot_brain/config/distance_keeper.yaml'
        ],
        remappings=[
            ('/obstacles','/robot/obstacles'), 
            ('/follow_cmd','/robot/follow_cmd'),
            ('/range_error','/robot/range_error'),
            ('/distance_markers','/robot/distance_markers'),
        ],
    )

    # ----- RViz2 -----
    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    base_frame = LaunchConfiguration('base_frame')

    static_tf_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_velodyne',
    # 인자: x y z yaw pitch roll parent child
    # 라이다가 base_link 좌표계에서 (0,0,0) 에 있다고 가정. 실제 위치/방향 있으면 값 넣어주세요.
        arguments=['0', '0', '0', '0', '0', '0', base_frame, velodyne_frame],
        output='screen'
    )

    # ----- 네임스페이스로 묶기 (멀티로봇 대비/토픽 충돌 회피) -----
    group = GroupAction([
        PushRosNamespace(ns),

        #SetRemap(src='velodyne_points', dst='velodyne_points'),

        static_map_base,
        static_tf_velodyne,
        velodyne_driver,
        velodyne_transform,
        pcl2scan,
        obstacle_extractor,
        obstacle_tracker,
        #camera_node,
        yolo_detector, 
        #obstacle_detector,
        relay_qos,
        distance_keeper,
        rviz2
    ])

    

    return LaunchDescription([
        # 보기 좋게 컬러 로그
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        # 인자 선언
        DeclareLaunchArgument('pcl2scan_exec', default_value='pointcloud_to_laserscan_node'),
        DeclareLaunchArgument('namespace', default_value='robot', description='ROS namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        DeclareLaunchArgument('base_frame', 
                            default_value='base_link',
                            description='Parent frame for velodyne'),

        # RViz 기본 설정: 네 레포의 rviz 샘플로 시작하고 싶다면 아래 경로 바꿔도 됨
        # 예: obstacle_detector_2/resources/obstacle_detector.rviz
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([FindPackageShare('obstacle_detector'), 'resources', 'obstacle_detector.rviz']),  # dummy
            description='Path to RViz config (.rviz).'
        ),

        # 활성/비활성 스위치
        DeclareLaunchArgument('enable_velodyne', default_value='true'),
        DeclareLaunchArgument('enable_detector', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true', description='Enable USB camera'),
        DeclareLaunchArgument('enable_yolo_detector', default_value='true', description='Enable camera-based YOLO detector'), # <<< 추가
        DeclareLaunchArgument('enable_qos', default_value='true'),
        DeclareLaunchArgument('enable_keeper', default_value='true'),
        DeclareLaunchArgument('enable_pcl2scan', default_value='true'),


        # Velodyne 네트워크/프레임 인자 (환경 맞춰 수정)
        DeclareLaunchArgument('velodyne_frame', default_value='velodyne'),
        DeclareLaunchArgument('velodyne_ip',    default_value='192.168.1.201'),  # ← 네 라이다 IP로 수정
        DeclareLaunchArgument('velodyne_port',  default_value='2368'),

        camera_node,
        group
    ])
