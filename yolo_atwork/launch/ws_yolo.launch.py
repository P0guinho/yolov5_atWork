from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
import yaml
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    #Camera
    cameraLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch/rs_launch.py')
        ),
    )
    
    aruco_params_file = os.path.join(
        get_package_share_directory('yolo_atwork'),
        'aruco_parameters.yaml'
    )
    
    with open(aruco_params_file, 'r') as file:
        config = yaml.safe_load(file)

    config = config["/aruco_node"]["ros__parameters"]

    # declare configuration parameters
    marker_size_arg = DeclareLaunchArgument(
        name='marker_size',
        default_value=str(config['marker_size']),
        description='Size of the aruco marker in meters',
    )

    aruco_dictionary_id_arg = DeclareLaunchArgument(
        name='aruco_dictionary_id',
        default_value=config['aruco_dictionary_id'],
        description='ID of the aruco dictionary to use',
    )

    image_topic_arg = DeclareLaunchArgument(
        name='image_topic',
        default_value=config['image_topic'],
        description='Name of the image RGB topic to subscribe to',
    )

    use_depth_input_arg = DeclareLaunchArgument(
        name='use_depth_input',
        default_value=str(config['use_depth_input']),
        description='Use depth input for pose estimation',
        choices=['true', 'false', 'True', 'False']
    )

    depth_image_topic_arg = DeclareLaunchArgument(
        name='depth_image_topic',
        default_value=config['depth_image_topic'],
        description='Name of the depth image topic to subscribe to',
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        name='camera_info_topic',
        default_value=config['camera_info_topic'],
        description='Name of the camera info topic to subscribe to',
    )

    camera_frame_arg = DeclareLaunchArgument(
        name='camera_frame',
        default_value=config['camera_frame'],
        description='Name of the camera frame where the estimated pose will be',
    )

    detected_markers_topic_arg = DeclareLaunchArgument(
        name='detected_markers_topic',
        default_value=config['detected_markers_topic'],
        description='Name of the topic to publish the detected markers messages',
    )

    markers_visualization_topic_arg = DeclareLaunchArgument(
        name='markers_visualization_topic',
        default_value=config['markers_visualization_topic'],
        description='Name of the topic to publish the pose array for visualization of the markers',
    )

    output_image_topic_arg = DeclareLaunchArgument(
        name='output_image_topic',
        default_value=config['output_image_topic'],
        description='Name of the topic to publish the image with the detected markers',
    )
    
    robotTFlaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sim_gazebo_classic'),
                         'launch/realbot_spawn.launch.py')
        )
    )
    
    ld = LaunchDescription([
        marker_size_arg,
        aruco_dictionary_id_arg,
        image_topic_arg,
        use_depth_input_arg,
        depth_image_topic_arg,
        camera_info_topic_arg,
        camera_frame_arg,
        detected_markers_topic_arg,
        markers_visualization_topic_arg,
        output_image_topic_arg,
        
        Node(
            package='yolo_atwork',
            executable='ws_recog_yolo',
        ),
        
        Node(
            package='yolo_atwork',
            executable='ws_pose_estimate',
        ),
        
        Node(
            package='yolo_atwork',
            executable='container_detection',
            parameters=[{
                "marker_size": LaunchConfiguration('marker_size'),
                "aruco_dictionary_id": LaunchConfiguration('aruco_dictionary_id'),
                "image_topic": LaunchConfiguration('image_topic'),
                "use_depth_input": LaunchConfiguration('use_depth_input'),
                "depth_image_topic": LaunchConfiguration('depth_image_topic'),
                "camera_info_topic": LaunchConfiguration('camera_info_topic'),
                "camera_frame": LaunchConfiguration('camera_frame'),
                "detected_markers_topic": LaunchConfiguration('detected_markers_topic'),
                "markers_visualization_topic": LaunchConfiguration('markers_visualization_topic'),
                "output_image_topic": LaunchConfiguration('output_image_topic'),
            }],
            output='screen',
            emulate_tty=True
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
        )
    ])

    ld.add_action(cameraLaunch)
    ld.add_action(robotTFlaunch)
    return ld