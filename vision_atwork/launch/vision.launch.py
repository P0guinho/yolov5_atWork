import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():
    
    realsense_pkg = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(realsense_pkg, 'launch', 'rs_launch.py')
        ])
    )

    yolo_recognition = Node(
            package='yolo_atwork',
            namespace='yolo_atwork',
            executable='recog_yolo',
            output='screen'  
    )

    yolo_pose_estimate = Node(
            package='yolo_atwork',
            namespace='yolo_atwork2',
            executable='pose_estimate',
            output='screen'
    )
    
    return LaunchDescription([
        realsense_launch,
        yolo_recognition,
        yolo_pose_estimate
    
    ])