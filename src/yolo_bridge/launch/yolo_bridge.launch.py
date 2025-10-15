# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import SetEnvironmentVariable
# # from launch.substitutions import EnvironmentVariable

# def generate_launch_description():
#     return LaunchDescription([
#         SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
#         Node(
#             package='yolo_bridge',
#             executable='yolo_node',
#             name='yolo_node',
#             output='screen',
#             parameters=[{
#                 'weights': '/home/yasric/senior_project/third_party/yolov5n/best_v1.pt',
#                 'conf_thres': 0.25,
#                 'device': 'cpu',       # 'cuda:0' 가능하면 변경
#                 'subscribe_image': '/camera/image_raw',
#                 'publish_overlay': '/yolo/image',
#                 'publish_boxes': '/yolo/boxes',
#             }],
#             env={
#                 'PYTHONNOUSERSITE': '0',
#                 # yolov5 파이썬 모듈을 직접 import 한다면(예: from models.common import …) 아래 줄도 유용
#                 'PYTHONPATH': '/home/yasric/senior_project/yolov5'
#             },
#             remappings=[
#                 ('image', '/camera/image_raw'),
#             ]
#         )
#     ])




# launch/yolo_bridge.launch.py
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess

# def generate_launch_description():
#     cmd = (
#         "source /home/yasric/senior_project/install/setup.bash"
#         " && python3 -m yolo_bridge.yolo_node"
#         " --ros-args"
#         " -p weights:=/home/yasric/senior_project/third_party/yolov5n/best_v1.pt"
#         " -p conf_thres:=0.25"
#         " -p device:=cpu"
#         " -p subscribe_image:=/camera/image_raw"
#         " -p publish_overlay:=/yolo/image"
#         " -p publish_boxes:=/yolo/boxes"
#         " -r image:=/camera/image_raw"
#     )

#     return LaunchDescription([
#         ExecuteProcess(
#             cmd=['bash','-lc', cmd],
#             output='screen',
#             # yolov5 소스를 직접 import한다면 필요한 경우에만 PYTHONPATH 추가
#             env={'PYTHONPATH': '/home/yasric/senior_project/yolov5'}
#         )
#     ])


# launch/yolo_bridge.launch.py (핵심 부분)
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    cmd = (
        "source /opt/ros/humble/setup.bash"                   # ← 추가!
        " && source /home/yasric/senior_project/install/setup.bash"
        " && python3 -m yolo_bridge.yolo_node"
        " --ros-args"
        " -p weights:=/home/yasric/senior_project/third_party/yolov5n/best_v1.pt"
        " -p conf_thres:=0.25"
        " -p device:=cpu"
        " -p subscribe_image:=/camera/image_raw"
        " -p publish_overlay:=/yolo/image"
        " -p publish_boxes:=/yolo/boxes"
        " -r image:=/camera/image_raw"
    )
    return LaunchDescription([
        ExecuteProcess(
            cmd=['bash','-lc', cmd],
            output='screen',
            # yolov5 소스를 직접 import할 때만 필요
            env={
                'HOME': '/home/yasric',
                'ROS_LOG_DIR': '/home/yasric/.ros/log',
                #'PYTHONNOUSERSITE': '1',
                'PYTHONPATH': '/home/yasric/senior_project/yolov5',
            }
        )
    ])
