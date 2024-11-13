
import rclpy
from rclpy.node import Node

import sys
import cv2
import numpy as np
from std_srvs.srv import Empty
from tutorial_interfaces.msg import Atworkobjects, Atworkobjectsarray
from tutorial_interfaces.srv import Detectionsorganized
from rclpy.action import ActionClient
from text_to_speech_msgs.action import TTS
from text_to_speech_msgs.msg import Config
import time

class Detections_Organizer(Node):
    def __init__(self):
        super().__init__('detections_organizer')

        #Subscribers
        self.aruco_detection_sub = self.create_subscription(Atworkobjectsarray, '/aruco/markers', self.aruco_callback, 10)
        self.aruco_detection_sub

        self.object_detection_sub = self.create_subscription(Atworkobjects, 'yolo/object_poses', self.detections_callback, 10)
        self.object_detection_sub
        
        self.text2speech_client = ActionClient(self, TTS, '/text_to_speech/tts')

        #Serviços
        self.srvClean = self.create_service(Empty, 'list_cleanner', self.srvCallback)
        self.srv = self.create_service(Detectionsorganized, 'get_all_detections_organized', self.getOrganizedList)
        self.global_detections_list = {}
    
    def aruco_callback(self, aruco_data):
        for detection in aruco_data.objects:
            self.global_detections_list[detection.id] = detection
    
    def detections_callback(self, detection):
        self.global_detections_list[detection.name] = detection
    
    def getOrganizedList(self, request, response):
        
        def sortingKey(val):
            score = val.id
            return score
        
        self.get_logger().info(str(len(self.global_detections_list.values())))
        organized_list = sorted(self.global_detections_list.values(), key = sortingKey, reverse=True)
        detections = Atworkobjectsarray()
        detections.objects = organized_list
        response = detections
        
        for det in organized_list:
            self.saySomeShit("Achei um " + det.name, 5.0, 'pt')
            time.sleep(2.0)
        
        self.get_logger().info('Detecções Organizadas')
        return response

    def srvCallback(self, request, response):
        self.global_detections_list = {}
        self.get_logger().info('Detecções Limpas')
        return response

    def saySomeShit(self, text, volume: float, language):
        goal_msg = TTS.Goal()
        goal_msg.text = text
        goal_msg.config.volume = volume
        goal_msg.config.language = language

        self.text2speech_client.wait_for_server()

        return self.text2speech_client.send_goal_async(goal_msg)
        
def main(args=None):
    rclpy.init(args=args)

    detections_organizer = Detections_Organizer()

    rclpy.spin(detections_organizer)

    detections_organizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()