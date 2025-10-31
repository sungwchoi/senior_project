# launch/yolo_bridge.launch.py 
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    cmd = (
        "source /opt/ros/humble/setup.bash"                   
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
            
            
            env={
                'HOME': '/home/yasric',
                'ROS_LOG_DIR': '/home/yasric/.ros/log',
                #'PYTHONNOUSERSITE': '1',
                'PYTHONPATH': '/home/yasric/senior_project/yolov5',
            }
        )
    ])
