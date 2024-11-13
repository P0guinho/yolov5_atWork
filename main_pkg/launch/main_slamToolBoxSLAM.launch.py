import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    gazebo_sim_launch_path = os.path.join(get_package_share_directory('sim_gazebo_classic'), 'launch')
    laser_merge_launch_path = os.path.join(get_package_share_directory('ros2_laser_scan_merger'), 'launch')
    navigation_pkg = get_package_share_directory('navigation_atwork')

    gazebo_sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_sim_launch_path, '/spawn_robotGZClassic.launch.py'])
        )

    laser_merge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([laser_merge_launch_path, '/merge_2_scan.launch.py'])
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
        gazebo_sim_launch,
        laser_merge_launch,
        navigation_launch,
        #dynamic_obstacle
        rviz_node
    ])