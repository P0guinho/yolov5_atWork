import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2D, BoundingBox2D
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

bridge = CvBridge()

class PoseEstimator(Node):

    #--------------------Initialize--------------------
    def __init__(self):
        super().__init__('object_pose_estimator')
        
        self.cam_img = cv2.cvtColor(cv2.imread('/home/gabri/at_work/src/yolo_atwork/yolo_atwork/thor.png'), cv2.COLOR_RGB2GRAY) #prevents this stupid code from trying to use a variable with no value
        self.depth_img = cv2.cvtColor(cv2.imread('/home/gabri/at_work/src/yolo_atwork/yolo_atwork/luna_depth.png'), cv2.COLOR_RGB2GRAY)

        self.depth_sub = self.create_subscription(Image,
                                                   '/camera/camera/depth/image_rect_raw',
                                                   self.depth_info,
                                                   10)
        self.yolo_sub = self.create_subscription(Detection2D,
                                                   'yolo/object_detection',
                                                   self.find_pose,
                                                   10)
    
    def depth_info(self, msg): #update received depth image
        self.depth_img = bridge.imgmsg_to_cv2(msg, "16UC1")
        
    def find_pose(self, msg: Detection2D):
        print(self.depth_img[int(msg.bbox.center.position.y)][int(msg.bbox.center.position.x)])

#--------------------Main--------------------
def main(args=None):
    rclpy.init(args=args)

    pose_estimator = PoseEstimator()
    rclpy.spin(pose_estimator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()