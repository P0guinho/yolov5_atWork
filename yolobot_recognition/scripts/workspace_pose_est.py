import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2D, BoundingBox2D
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D, Vector3
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

bridge = CvBridge()

class PoseEstimator(Node):

    #--------------------Initialize--------------------
    def __init__(self):
        super().__init__('workspace_pose_estimator')

        self.cam_img: Image
        self.depth_img: Image

        self.canny_thre1: int = 30
        self.canny_thre2: int = 100

        self.Fx: float
        self.Fy: float
        self.Cx: float
        self.Cy: float

        self.image_sub = self.create_subscription(Image,
                                                   '/camera/camera/color/image_raw',
                                                   self.image_info,
                                                   10)
        self.depth_sub = self.create_subscription(Image,
                                                   '/camera/camera/depth/image_rect_raw',
                                                   self.depth_info,
                                                   10)
        self.depth_info_sub = self.create_subscription(CameraInfo,
                                                   '/camera/camera/depth/camera_info',
                                                   self.receive_cam_coefs,
                                                   10)
        self.yolo_sub = self.create_subscription(Detection2D,
                                                   'yolo/workspace_detection',
                                                   self.find_pose,
                                                   10)

    #--------------------Receive info--------------------
    def receive_cam_coefs(self, msg: CameraInfo):
        self.Fx = msg.k[0]
        self.Fy = msg.k[4]
        self.Cx = msg.k[2]
        self.Cy = msg.k[5]
        self.fart(1000)
        self.destroy_subscription(self.depth_info_sub)

    def image_info(self, msg): #update received camera image
        self.cam_img = bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_info(self, msg): #update received depth image
        self.depth_img = bridge.imgmsg_to_cv2(msg, "16UC1")

    #--------------------Estimate pose--------------------
    def find_pose(self, msg: Detection2D):
        imgC = self.cropIMG(self.cam_img, msg.bbox) #cropped color img
        imgD = self.cropIMG(self.depth_img, msg.bbox) #cropped depth img

        if msg.id == "virt_wall":
            self.find_virtWall_pose(imgC, imgD)

    def find_virtWall_pose(self, color: Image, depth: Image):
        contouredImg = color.copy()
        color = cv2.GaussianBlur(color, (7, 7), 1) #blur img
        color = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY) #convert img to grayscale

        color = cv2.Canny(color, self.canny_thre1, self.canny_thre2) #get edges

        dilKernel = np.ones((3, 3))
        color = cv2.dilate(color, dilKernel) #dilate img

        contours, hierarchy = cv2.findContours(color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #get contours
        for cont in contours:
            area = cv2.contourArea(cont)

            if area > 1000: #checking the area helps reducing noise
                cv2.drawContours(contouredImg, contours, -1, (255, 0, 255), 3)

                perimeter = cv2.arcLength(cont, True) #get shape perimeter
                approx = cv2.approxPolyDP(cont, 0.02 * perimeter, True) #approximate what shape it is
                print("Wall has " + str(len(approx)) + " points with an area of " + str(area))
        """
        for point in approx:
            p = point[0]
            x = p[0]
            y = p[1]
            print(self.findPixelCoords(x, y))
        """
        cv2.imshow("Virtual Wall", contouredImg)
        cv2.imshow("Depth", depth)
        cv2.waitKey(1)

    #--------------------Utils--------------------
    def cropIMG(self, img: Image, bbox: BoundingBox2D):
        box_center = bbox.center.position

        corner1 = Pose2D()
        corner1.x = float(box_center.x - (bbox.size_x / 2))
        corner1.y = float(box_center.y - (bbox.size_y / 2))
        corner1.theta = 0.0

        corner2 = Pose2D()
        corner2.x = float(box_center.x + (bbox.size_x / 2))
        corner2.y = float(box_center.y + (bbox.size_y / 2))
        corner2.theta = 0.0

        #In slicing an image, x and y are flipped
        i = img[int(corner1.y):int(corner2.y),
                int(corner1.x):int(corner2.x)].copy()

        return i
    
    def fart(self, intensity: int):
        print("Node farted with an intensity of", str(intensity), "kiloFarts")


    """def findPixelCoords(self, x: int, y: int) -> Vector3:
        invFx = 1/self.Fx
        invFy = 1/self.Fy

        coords: Vector3 = Vector3()
        coords.z = self.depth_img[x][y] * 0.001
        coords.x = (x - self.Cx) * coords.z * invFx
        coords.y = (y - self.Cy) * coords.z * invFy

        return coords"""
        



#--------------------Main--------------------
def main(args=None):
    rclpy.init(args=args)

    pose_estimatior = PoseEstimator()
    rclpy.spin(pose_estimatior)
    rclpy.shutdown()


if __name__ == '__main__':
    main()