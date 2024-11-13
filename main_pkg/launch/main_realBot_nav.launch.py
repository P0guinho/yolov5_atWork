import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    map_to_odom = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        )

    odom_to_baseFootprint = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"]
        )

    urdf_pkg_launch_path = os.path.join(get_package_share_directory('sim_gazebo_classic'), 'launch')
    lidars_pkg_launch_path = os.path.join(get_package_share_directory('sllidar_ros2'), 'launch')
    laser_merge_launch_path = os.path.join(get_package_share_directory('ros2_laser_scan_merger'), 'launch')
    laser_matcher_launch_path = os.path.join(get_package_share_directory('ros2_laser_scan_matcher'), 'launch')
    navigation_pkg = get_package_share_directory('navigation_atwork')

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
    
    
    print("Path:", os.path.join(navigation_pkg, 'launch', 'slam_slamTB.launch.py'))
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(navigation_pkg, 'launch', 'slam_slamTB.launch.py')
        ])
    )

    '''
    dynamic_obstacle = Node(
            package='navigation_atwork',
            executable='subscriberFP',
            output='screen',
            name='minimal_subscriber'
    )
    '''
    
    rviz_config_dir = os.path.join(get_package_share_directory('main_pkg'), 'rviz', 'navSlamTB.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])


    return LaunchDescription([
        #map_to_odom,
        #odom_to_baseFootprint,
        urdf_launch,
        lidar_launch,
        laser_merge_launch,
        laser_matcher_launch,
        navigation_launch,
        #dynamic_obstacle
        rviz_node
    ])