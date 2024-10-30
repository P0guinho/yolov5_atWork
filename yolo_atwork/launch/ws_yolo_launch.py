from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    cameraLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch/rs_launch.py')
        ),
    )

    ld = LaunchDescription([
        Node(
            package='yolo_atwork',
            executable='ws_recog_yolo',
        ),
        
        Node(
            package='yolo_atwork',
            executable='ws_pose_estimate',
        )
    ])

    ld.add_action(cameraLaunch)
    return ld