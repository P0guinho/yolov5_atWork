import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    gazebo_sim_launch_path = os.path.join(get_package_share_directory('sim_gazebo_classic'), 'launch')
    laser_merge_launch_path = os.path.join(get_package_share_directory('ros2_laser_scan_merger'), 'launch')
    navigation_launch_path = os.path.join(get_package_share_directory('navigation_atwork'), 'launch')

    gazebo_sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_sim_launch_path, '/spawn_robotGZClassic.launch.py'])
        )

    laser_merge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([laser_merge_launch_path, '/merge_2_scan.launch.py'])
        )

    navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navigation_launch_path, '/slam.launch.py'])
        )

    rviz_config_dir = os.path.join(get_package_share_directory('main_pkg'), 'rviz', 'main2.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])


    return LaunchDescription([
        gazebo_sim_launch,
        laser_merge_launch,
        navigation_launch,
        rviz_node

    ])