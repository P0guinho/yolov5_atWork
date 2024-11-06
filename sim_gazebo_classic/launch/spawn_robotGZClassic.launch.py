#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import Command
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

# this is the function launch  system will look for
def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    sim_gazeboClassic_pkg_path = get_package_share_directory('sim_gazebo_classic')

    description_package_name = "sim_gazebo_classic"
    install_dir = get_package_prefix(description_package_name)

    gazebo_models_path = os.path.join(sim_gazeboClassic_pkg_path, 'worlds')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    #WORLD PATH
    world = os.path.join(
        sim_gazeboClassic_pkg_path,
        'worlds',
        'jgworld.world'
    )

    #GAZEBO LAUNCH
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ####### DATA INPUT ##########
    urdf_file = 'atWorkBot.urdf.xacro'
    package_description = "sim_gazebo_classic"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # Position and orientation
    # [X, Y, Z]
    position = [1.5, -1.5, 0.5]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Entity Name or robot
    entity_name = 'NWatBot'

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_rviz.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        #joint_state_publisher_gui,
        spawn_robot,
        #rviz_node
    ])