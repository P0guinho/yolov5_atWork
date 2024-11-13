import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    map_to_odom = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["2.6", "0.4", "0", "0", "0", "0", "map", "odom"],    
        )
    
    map_to_stacking = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0.675", "2.8891", "0", "0", "0", "0", "map", "odom"],    
        )

    urdf_pkg_launch_path = os.path.join(get_package_share_directory('sim_gazebo_classic'), 'launch')
    lidars_pkg_launch_path = os.path.join(get_package_share_directory('sllidar_ros2'), 'launch')
    laser_merge_launch_path = os.path.join(get_package_share_directory('ros2_laser_scan_merger'), 'launch')
    laser_matcher_launch_path = os.path.join(get_package_share_directory('ros2_laser_scan_matcher'), 'launch')

    urdf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([urdf_pkg_launch_path, '/realbot_spawn.launch.py'])
        )
    
    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidars_pkg_launch_path, '/two_lidars.launch.py'])
        )

    laser_merge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([laser_merge_launch_path, '/merge_2_scan.launch.py'])
        )
    
    laser_matcher_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([laser_matcher_launch_path, '/odometry_font.launch.py'])
        )
    
    moveTarget_service = Node(
        package="ugiebugie_navigation",
        executable="ugie_bugie_navigator",
        output="screen",
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'rviz',
        'ugabuga.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
    
    return LaunchDescription([
        map_to_odom,
        urdf_launch,
        lidar_launch,
        laser_merge_launch,
        laser_matcher_launch,
        moveTarget_service,
        rviz
    ])