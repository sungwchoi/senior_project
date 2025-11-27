# system_bringup.launch.py
# velodyne driver -> transform -> obstacle_detector -> relay_qos -> distance_keeper -> rviz2
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
    # ### NEW: follow_signal 관련 인자
    enable_follow_signal = LaunchConfiguration('enable_follow_signal')
    adjust_deadband      = LaunchConfiguration('adjust_deadband')
    hysteresis           = LaunchConfiguration('hysteresis')
    follow_serial_enable = LaunchConfiguration('follow_serial_enable')
    follow_serial_port   = LaunchConfiguration('follow_serial_port')
    follow_serial_baud   = LaunchConfiguration('follow_serial_baud')



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
        #package='launchtf2_ros',
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
            'rpm': 600.0, 
            'gps_time': False,
            'time_offset': 0.0,
            'enabled': True,
            'read_once': False,
            'read_fast': False,
            'repeat_delay': 0.0,
            'timestamp_first_packet': False
        }],
        
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
                    'use_sim_time': use_sim_time,
                    'organize_cloud': False,          # ### OPT: 불필요한 정렬 비활성(CPU save)
                }],
            
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
                    'transform_tolerance': 0.03,

                # Z-슬라이스: target_frame 좌표계에서 이 높이 구간의 포인트만 LaserScan에 반영
                    'min_height': -0.20,   # 바닥 근처 노이즈 컷
                    'max_height':  1.25,

                # 스캔 각도/해상도(라디안). 필요 시 조정.
                    'angle_min': -1.5708 ,
                    'angle_max':  1.5708,
                    'angle_increment': 0.00436,  # ≈0.17° (원하면 0.00436 ≈0.25°)

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
                parameters=[
                    {'use_sim_time': use_sim_time},
                # --- 아래 파라미터들을 전부 추가 ---
                    {'active': True},
                    {'use_scan': True},
                    {'use_pcl': False},
                    {'use_pcl2': False},
                    
                    {'use_split_and_merge': True},
                    {'circles_from_visibles': True},
                    {'discard_converted_segments': False},
                    {'transform_coordinates': False},

                    # --- 🌟 다리 감지를 위한 튜닝값 ---
                    {'min_group_points': 4},         # 10 -> 4 (다리가 가늘어도 잡히도록)
                    {'max_group_distance': 0.10},    # 0.08 -> 0.10 (조금 여유롭게)
                    {'distance_proportion': 0.006},  # (XML 값)
                    {'max_split_distance': 0.18},    # (XML 값)
                    {'max_merge_separation': 0.20},  # (XML 값)
                    {'max_merge_spread': 0.20},      # (XML 값)
                    
                    # --- 🌟 distance_keeper와 범위를 맞춤 ---
                    {'min_circle_radius': 0.04},     # (신규 추가, 5cm 근처)
                    {'max_circle_radius': 0.20},     # 0.3 -> 0.20 (18cm 근처)
                    
                    {'radius_enlargement': 0.10},    # 0.15 -> 0.10
                    {'frame_id': velodyne_frame}
                ],                
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
    # ----- robot_brain: relay_qos -----Bus 001 Device 024: ID 1871:0d01 Aveo Technology Corp. USB2.0 Camera
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
            'in_topic':  'scan',           
            'out_topic': 'scan_reliable',
        
        }],
        #remappings=[]
    )
    # ----- robot_brain: distance_keeper -----
    distance_keeper = Node(
        condition=IfCondition(enable_keeper),
        package='robot_brain',
        executable='distance_keeper',
        name='distance_keeper',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        
        parameters=[
            {'use_sim_time': use_sim_time},
            #PathJoinSubstitution([FindPackageShare('robot_brain'), 'config', 'distance_keeper.yaml'])
        
        ],
        remappings=[
            ('/obstacles','/robot/obstacles'), 
            ('/follow_cmd','/robot/follow_cmd'),
            ('/range_error','/robot/range_error'),
            ('/distance_markers','/robot/distance_markers'),
        ],
    )
    # ### NEW: robot_brain: follow_signal_publisher
    follow_signal = Node(
        condition=IfCondition(enable_follow_signal),
        package='robot_brain',
        executable='follow_signal_publisher',
        name='follow_signal_publisher',
        output='screen',
        parameters=[{
            'in_topic': '/robot/range_error',          # distance_keeper 출력
            'forward_topic': '/robot/cmd_forward',
            'backward_topic': '/robot/cmd_backward',
            'state_topic': '/robot/range_state',
            'adjust_deadband': adjust_deadband,
            'hysteresis': hysteresis,
            'enable_serial': follow_serial_enable,
            'serial_port': follow_serial_port,
            'serial_baud': follow_serial_baud,
            'serial_on_change_only': True,
            'serial_min_period': 0.4
        }]
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

    joy_node = Node(
        condition=IfCondition(LaunchConfiguration('enable_joystick')),
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': LaunchConfiguration('joy_dev'), 'deadzone': 0.05, 'autorepeat_rate': 20.0}]
    )
    
    joy_to_serial = Node(
        condition=IfCondition(LaunchConfiguration('enable_joystick')),
        package='robot_brain',
        executable='joy_to_serial',  # setup.py에서 console_script로 등록했거나, python script path 사용
        name='joy_to_serial',
        output='screen',
        parameters=[{
            'joy_topic': 'joy',
            'axis_x': LaunchConfiguration('joy_axis_x'),
            'axis_y': LaunchConfiguration('joy_axis_y'),
            'hat_x': LaunchConfiguration('joy_hat_x'),
            'hat_y': LaunchConfiguration('joy_hat_y'),
            'deadzone': LaunchConfiguration('joy_deadzone'),
            'prefer_hat': LaunchConfiguration('joy_prefer_hat'),
            'port': LaunchConfiguration('joystick_serial_port'),
            'baud': LaunchConfiguration('joystick_serial_baud'),
            'payload_mode': LaunchConfiguration('joystick_payload_mode'),
            'on_change_only': True,
            'min_period': 0.15,

            'invert_x': LaunchConfiguration('joy_invert_x'),
            'invert_y': LaunchConfiguration('joy_invert_y'),
        }]
    )


    # ----- 네임스페이스로 묶기 (멀티로봇 대비/토픽 충돌 회피) -----
    group = GroupAction([
        PushRosNamespace(ns),
        static_map_base,
        static_tf_velodyne,
        velodyne_driver,
        velodyne_transform,
        pcl2scan,
        obstacle_extractor,
        obstacle_tracker,
        yolo_detector,
        relay_qos,
        distance_keeper,
        follow_signal,
        joy_node,
        joy_to_serial,
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
# generate_launch_description
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

        # ### NEW: follow_signal 인자
        DeclareLaunchArgument('enable_follow_signal', default_value='true', description='Enable follow signal publisher'),
        DeclareLaunchArgument('adjust_deadband', default_value='0.30', description='± deadband (m)'),
        DeclareLaunchArgument('hysteresis', default_value='0.05', description='hysteresis (m)'),
        DeclareLaunchArgument('follow_serial_enable', default_value='false', description='Enable USB serial to Arduino'),
        DeclareLaunchArgument('follow_serial_port', default_value='/dev/ttyACM0', description='Arduino serial port (or /dev/arduino_follow)'),
        DeclareLaunchArgument('follow_serial_baud', default_value='115200', description='Arduino serial baud'),

        # NEW: Joystick 신호확인
        DeclareLaunchArgument('enable_joystick', default_value='true'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('joystick_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('joystick_serial_baud', default_value='115200'),
        DeclareLaunchArgument('joystick_payload_mode', default_value='char'),  # 'char'|'word'|'both'
        DeclareLaunchArgument('joy_axis_x', default_value='0'),
        DeclareLaunchArgument('joy_axis_y', default_value='1'),
        DeclareLaunchArgument('joy_hat_x',  default_value='6'),
        DeclareLaunchArgument('joy_hat_y',  default_value='7'),
        DeclareLaunchArgument('joy_deadzone', default_value='0.4'),
        DeclareLaunchArgument('joy_prefer_hat', default_value='true'),
        DeclareLaunchArgument('joy_invert_x', default_value='false'),
        DeclareLaunchArgument('joy_invert_y', default_value='false'),


        camera_node,
        group
    ])
