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
        launch_arguments={
            'json_file_path': '/home/gabri/at_work/src/yolo_atwork/yolo_atwork/configs/default.json'
        }.items()
    )
    robotTFlaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sim_gazebo_classic'),
                         'launch/realbot_spawn.launch.py')
        )
    )

    ld = LaunchDescription([
        Node(
            package='yolo_atwork',
            executable='obj_recog_yolo',
        ),
        
        Node(
            package='yolo_atwork',
            executable='obj_pose_estimate',
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2'
        )
    ])

    ld.add_action(cameraLaunch)
    ld.add_action(robotTFlaunch)
    return ld