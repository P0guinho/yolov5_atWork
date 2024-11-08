
import rclpy
from rclpy.node import Node

import sys
import cv2
import numpy as np
from std_srvs.srv import Empty
from tutorial_interfaces.msg import Atworkobjects, Atworkobjectsarray
from tutorial_interfaces.srv import Detectionsorganized

class Detections_Organizer(Node):
    def __init__(self):
        super().__init__('detections_organizer')

        #Subscribers
        self.aruco_detection_sub = self.create_subscription(Atworkobjectsarray, '/aruco/markers', self.aruco_callback, 10)
        self.aruco_detection_sub

        self.object_detection_sub = self.create_subscription(Atworkobjects, 'yolo/object_poses', self.detections_callback, 10)
        self.object_detection_sub

        #Serviços
        self.srvClean = self.create_service(Empty, 'list_cleanner', self.srvCallback)
        self.srv = self.create_service(Detectionsorganized, 'get_all_detections_organized', self.getOrganizedList)
        self.global_detections_list = {}
    
    def aruco_callback(self, aruco_data):
        for detection in aruco_data.detections:
            self.global_detections_list[detection.id] = detection
    
    def detections_callback(self, detection):
        self.global_detections_list[detection.name] = detection
    
    def getOrganizedList(self, request, response):
        
        def sortingKey(val):
            score = val
            return score
        
        organized_list = sorted(self.global_detections_list.values(), key = sortingKey, reverse=True)
        detections = Detectionsorganized()
        detections.cubes.detections = organized_list
        self.get_logger().info('Detecções Organizadas')
        return detections

    def srvCallback(self, request, response):
        self.global_detections_list = {}
        self.get_logger().info('Detecções Limpas')
        return response
        
def main(args=None):
    rclpy.init(args=args)

    detections_organizer = Detections_Organizer()

    rclpy.spin(detections_organizer)

    detections_organizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()