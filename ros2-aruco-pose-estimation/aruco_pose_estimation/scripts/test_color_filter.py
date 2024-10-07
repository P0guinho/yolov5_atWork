import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

bridge = CvBridge()

class PoseEstimator(Node):

    def __init__(self):
        super().__init__('workspace_pose_estimator')
        
        self.lower_blue = np.array([80, 170, 60])
        self.upper_blue = np.array([110, 255, 100])
        
        self.lower_red = np.array([0, 160, 80])
        self.upper_red = np.array([30, 200, 170]) 
        
        self.image_sub = self.create_subscription(Image,
                                                   '/camera/camera/color/image_raw',
                                                   self.filter_img,
                                                   10)

    def filter_img(self, msg): #update received camera image
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    
        mask = cv2.inRange(img, self.lower_red, self.upper_red) 
        
        result = cv2.bitwise_and(img, img, mask = mask) 

        cv2.imshow('img orginial', bridge.imgmsg_to_cv2(msg, "bgr8"))
        cv2.imshow('mask', mask) 
        cv2.imshow('result', result) 
        
        cv2.waitKey(1)


#--------------------Main--------------------
def main(args=None):
    rclpy.init(args=args)

    pose_estimator = PoseEstimator()
    rclpy.spin(pose_estimator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()