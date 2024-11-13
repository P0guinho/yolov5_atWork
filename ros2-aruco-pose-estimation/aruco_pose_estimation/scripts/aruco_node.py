#!/usr/bin/env python3
"""
ROS2 wrapper code taken from:
https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main

This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

    /aruco_image (sensor_msgs.msg.Image)
       Annotated image with marker locations and ids, with markers drawn on it

Parameters:
    marker_size - size of the markers in meters (default .065)
    aruco_dictionary_id - dictionary that was used to generate markers (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/color/image_raw)
    camera_info_topic - camera info topic to subscribe to (default /camera/camera_info)
    camera_frame - camera optical frame to use (default "camera_depth_optical_frame")
    detected_markers_topic - topic to publish detected markers (default /aruco_markers)
    markers_visualization_topic - topic to publish markers visualization (default /aruco_poses)
    output_image_topic - topic to publish annotated image (default /aruco_image)

Author: Simone Giampà
Version: 2024-01-29

"""

#To run code:
#ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py

# ROS2 imports
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import message_filters

# Python imports
import numpy as np
import cv2

# Local imports for custom defined functions
from aruco_pose_estimation.utils import ARUCO_DICT
from aruco_pose_estimation.pose_estimation import pose_estimation, is_pixel_in_polygon

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, TransformStamped, Pose
from aruco_interfaces.msg import ArucoMarkers
from tutorial_interfaces.msg import Atworkobjects, Atworkobjectsarray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from vision_msgs.msg import ObjectHypothesisWithPose, Detection2D
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.action import ActionClient
from text_to_speech_msgs.action import TTS
from text_to_speech_msgs.msg import Config


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.initialize_parameters()
        
        self.i: int = 0 #Give a identification number to msgs to prevent two having the same name

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            # check if the dictionary_id is a valid dictionary inside ARUCO_DICT values
            if dictionary_id not in ARUCO_DICT.values():
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(self.dictionary_id_name)
            )
            options = "\n".join([s for s in ARUCO_DICT])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions to the camera info and camera image topics

        # camera info topic for the camera calibration parameters
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data
        )

        # select the type of input to use for the pose estimation
        if (bool(self.use_depth_input)):
            # use both rgb and depth image topics for the pose estimation

            # create a message filter to synchronize the image and depth image topics
            self.image_sub = message_filters.Subscriber(self, Image, self.image_topic,
                                                        qos_profile=qos_profile_sensor_data)
            self.depth_image_sub = message_filters.Subscriber(self, Image, self.depth_image_topic,
                                                              qos_profile=qos_profile_sensor_data)

            # create synchronizer between the 2 topics using message filters and approximate time policy
            # slop is the maximum time difference between messages that are considered synchronized
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_image_sub], queue_size=10, slop=0.05
            )
            self.synchronizer.registerCallback(self.rgb_depth_sync_callback)
        else:
            # rely only on the rgb image topic for the pose estimation

            # create a subscription to the image topic
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data
            )

        self.detection: Detection2D = Detection2D()
        
        #Prevent getting an error where the code tries to use a null value self.detection
        self.detection.bbox.center.position.x = 0
        self.detection.bbox.center.position.y = 0
        self.detection.bbox.size_x = 0
        self.detection.bbox.size_y = 0
        
        self.yolo_sub = self.create_subscription(Detection2D,
                                                   'yolo/object_detection',
                                                   self.yolo_sub,
                                                   10)
        
        # Set up publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.foundIDs = []
        self.aruco_pub = self.create_publisher(Atworkobjectsarray, self.detected_markers_topic, 10)
        
        self.text2speech_client = ActionClient(self, TTS, '/text_to_speech/tts')

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # code for updated version of cv2 (4.7.0)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        # old code version
        # self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        # self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()

    def yolo_sub(self, msg: Detection2D):   
        if msg.id == "cont_blue" or msg.id == "cont_red": self.detection = msg
    
    def info_callback(self, info_msg):
        self.info_msg = info_msg
        # get the intrinsic matrix and distortion coefficients from the camera info
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

        self.get_logger().info("Camera info received.")
        self.get_logger().info("Intrinsic matrix: {}".format(self.intrinsic_mat))
        self.get_logger().info("Distortion coefficients: {}".format(self.distortion))
        self.get_logger().info("Camera frame: {}x{}".format(self.info_msg.width, self.info_msg.height))

        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        # convert the image messages to cv2 format
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        # create the ArucoMarkers and PoseArray messages
        markers = ArucoMarkers()

        # Set the frame id and timestamp for the markers and pose array
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp

        """
        # OVERRIDE: use calibrated intrinsic matrix and distortion coefficients
        self.intrinsic_mat = np.reshape([615.95431, 0., 325.26983,
                                         0., 617.92586, 257.57722,
                                         0., 0., 1.], (3, 3))
        self.distortion = np.array([0.142588, -0.311967, 0.003950, -0.006346, 0.000000])
        """
        
        # call the pose estimation function
        frame, markers, corners = pose_estimation(rgb_frame=cv_image, depth_frame=None,
                                         aruco_detector=self.aruco_detector,
                                         marker_size=self.marker_size, matrix_coefficients=self.intrinsic_mat,
                                         distortion_coefficients=self.distortion, markers=markers)

        msg = Atworkobjectsarray()
        
        # if some markers are detected
        if len(markers.marker_ids) > 0: 
            for i in range(len(markers.marker_ids)):
                #Publish ATTC
                aruco = Atworkobjects()
                
                #If couldnt get color, dont send anything
                if i <= len(markers.colors) - 1:
                    aruco.color = markers.colors[i]
                else:
                    #Table tag
                    self.generateTF("camera_color_frame", "Table Tag ID " + str(markers.marker_ids[i]), markers.poses[i])
                    self.get_logger().info("Table tag ID " + str(markers.marker_ids[i]))
                    return
                
                if is_pixel_in_polygon((corners[i][0][0][0], corners[i][0][0][1]), np.array([(self.detection.bbox.center.position.x - self.detection.bbox.size_x, self.detection.bbox.center.position.y - self.detection.bbox.size_y),
                                                                                             (self.detection.bbox.center.position.x + self.detection.bbox.size_x, self.detection.bbox.center.position.y - self.detection.bbox.size_y),
                                                                                             (self.detection.bbox.center.position.x + self.detection.bbox.size_x, self.detection.bbox.center.position.y + self.detection.bbox.size_y),
                                                                                             (self.detection.bbox.center.position.x - self.detection.bbox.size_x, self.detection.bbox.center.position.y + self.detection.bbox.size_y)])):
                    self.get_logger().info("wtf was that")
                    self.get_logger().info("Container tag ID " + str(markers.marker_ids[i]))
                    self.generateStaticTF("camera_color_optical_frame", markers.colors[i] + " Container", markers.poses[i])
                    return
                
                
                aruco.id = markers.marker_ids[i]
                aruco.name = aruco.color + " ATTC No. " + str(self.i)
                self.get_logger().info("ATTC tag ID " + str(markers.marker_ids[i]))
                
                pos = ObjectHypothesisWithPose()
                pos.pose.pose = markers.poses[i]

                #Rviz and openCV use different axis orientations
                x = pos.pose.pose.position.x
                y = pos.pose.pose.position.y
                z = pos.pose.pose.position.z
                pos.pose.pose.position.x = z
                pos.pose.pose.position.y = y
                pos.pose.pose.position.z = x
                aruco.detection.results.append(pos)
                
                msg.objects.append(aruco)
                
                #Generate TF
                self.generateTF("camera_color_frame", aruco.name, markers.poses[i])
                
                if aruco.id in self.foundIDs:
                    pass
                else:
                    #self.saySomeShit("Achei um " + markers.colors[i] + " A T T C", 5.0, 'pt-br')
                    self.foundIDs.append(aruco.id)
            
            self.aruco_pub.publish(msg)

    def saySomeShit(self, text, volume: float, language):
        goal_msg = TTS.Goal()
        goal_msg.text = text
        goal_msg.config.volume = volume
        goal_msg.config.language = language

        self.text2speech_client.wait_for_server()

        return self.text2speech_client.send_goal_async(goal_msg)

    def depth_image_callback(self, depth_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

    def rgb_depth_sync_callback(self, rgb_msg: Image, depth_msg: Image):

        # convert the image messages to cv2 format
        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")

        # create the ArucoMarkers and PoseArray messages
        markers = ArucoMarkers()
        pose_array = PoseArray()

        # Set the frame id and timestamp for the markers and pose array
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = rgb_msg.header.stamp
        pose_array.header.stamp = rgb_msg.header.stamp

        # call the pose estimation function
        frame, pose_array, markers = pose_estimation(rgb_frame=cv_image, depth_frame=cv_depth_image,
                                                     aruco_detector=self.aruco_detector,
                                                     marker_size=self.marker_size, matrix_coefficients=self.intrinsic_mat,
                                                     distortion_coefficients=self.distortion, pose_array=pose_array, markers=markers)

        # if some markers are detected
        if len(markers.marker_ids) > 0:
            # Publish the results with the poses and markes positions
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

        # publish the image frame with computed markers positions over the image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

    def generateTF(self, parent_name, child_name, pos: Pose):
        transform = TransformStamped()
        
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_name
        transform.child_frame_id = child_name
        
        transform.transform.translation.x = pos.position.x
        transform.transform.translation.y = pos.position.y
        transform.transform.translation.z = pos.position.z
        
        transform.transform.rotation.x = pos.orientation.x
        transform.transform.rotation.y = pos.orientation.y
        transform.transform.rotation.z = pos.orientation.z
        transform.transform.rotation.w = pos.orientation.w
        
        self.tf_broadcaster.sendTransform(transform)
    
    def generateStaticTF(self, parent_name, child_name, pos: Pose):
        transform = TransformStamped()
        
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_name
        transform.child_frame_id = child_name
        
        transform.transform.translation.x = pos.position.x
        transform.transform.translation.y = pos.position.y
        transform.transform.translation.z = pos.position.z
        
        transform.transform.rotation.x = pos.orientation.x
        transform.transform.rotation.y = pos.orientation.y
        transform.transform.rotation.z = pos.orientation.z
        transform.transform.rotation.w = pos.orientation.w
        
        self.static_tf_broadcaster.sendTransform(transform)
    
    def initialize_parameters(self):
        # Declare and read parameters from aruco_params.yaml
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="use_depth_input",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Use depth camera input for pose estimation instead of RGB image",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="depth_image_topic",
            value="/camera/depth/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Depth camera topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="detected_markers_topic",
            value="/aruco_markers",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish detected markers as array of marker ids and poses",
            ),
        )

        self.declare_parameter(
            name="markers_visualization_topic",
            value="/aruco_poses",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish markers as pose array",
            ),
        )

        self.declare_parameter(
            name="output_image_topic",
            value="/aruco_image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish annotated images with markers drawn on them",
            ),
        )

        # read parameters from aruco_params.yaml and store them
        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        self.dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {self.dictionary_id_name}")

        self.use_depth_input = (
            self.get_parameter("use_depth_input").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Use depth input: {self.use_depth_input}")

        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Input image topic: {self.image_topic}")

        self.depth_image_topic = (
            self.get_parameter("depth_image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Input depth image topic: {self.depth_image_topic}")

        self.info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image camera info topic: {self.info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        # Output topics
        self.detected_markers_topic = (
            self.get_parameter("detected_markers_topic").get_parameter_value().string_value
        )

        self.markers_visualization_topic = (
            self.get_parameter("markers_visualization_topic").get_parameter_value().string_value
        )

        self.output_image_topic = (
            self.get_parameter("output_image_topic").get_parameter_value().string_value
        )


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
