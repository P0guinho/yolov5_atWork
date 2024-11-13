from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory
import yaml

includeRobotURDF: bool = False
includeWSdetection: bool = False
includeObjDetection: bool = True
includeATTCdetection: bool = True
includeTTS: bool = True

def generate_launch_description():

    ld = LaunchDescription([])
    
    cameraLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch/rs_launch.py')
        ),
        launch_arguments={
            'json_file_path': '/home/gabri/at_work/src/yolo_atwork/yolo_atwork/configs/default.json'
        }.items()
    )
    ld.add_action(cameraLaunch)
    
    if includeTTS:
        ttsLaunch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('text_to_speech'),
                            'text_to_speech.launch.py')
            )
        )
        ld.add_action(ttsLaunch)
    
    if includeRobotURDF:
        robotTFlaunch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('sim_gazebo_classic'),
                            'launch/realbot_spawn.launch.py')
            )
        )
        ld.add_action(robotTFlaunch)
    
    if includeATTCdetection:
        ATTClaunch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('aruco_pose_estimation'),
                            'launch/aruco_pose_estimation.launch.py')
            )
        )
        ld.add_action(ATTClaunch)
    
    if includeWSdetection:
        WSlaunch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('yolo_atwork'),
                            'ws_yolo.launch.py')
            )
        )
        ld.add_action(WSlaunch)

    if includeObjDetection:
        OBJdetection = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('yolo_atwork'),
                            'obj_yolo.launch.py')
            )
        )
        ld.add_action(OBJdetection)
    
    return ld