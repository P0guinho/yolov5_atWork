import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2D, BoundingBox2D
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D, Point
from cv_bridge import CvBridge, CvBridgeError

from custom_msgs.msg import Fourpoints

import cv2
import numpy as np

bridge = CvBridge()

class PoseEstimator(Node):

    #--------------------Initialize--------------------
    def __init__(self):
        super().__init__('workspace_pose_estimator')
        
        self.cam_img: Image = bridge.cv2_to_imgmsg(cv2.imread('thor.png')) #prevents this stupid code from trying to use a variable with no value
        self.depth_img: Image = bridge.cv2_to_imgmsg(cv2.imread('luna_depth.png'))

        self.canny_thre1: int = 30
        self.canny_thre2: int = 50

        self.Fx: float = 1.0
        self.Fy: float = 1.0
        self.Cx: float = 1.0
        self.Cy: float = 1.0

        self.cam_rot: float = 15 #xx is the rotation in the robot, in degrees
        self.dist_to_mod: float = 25 #vertical distance from the pivot of the cam to the mod, in mm. xx when in the robot

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
        
        self.virtWall_pub = self.create_publisher(
            Fourpoints,
            "yolo/virtWall_estimation_pose",
            10
        )

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
        box = msg.bbox

        if msg.id == "virt_wall":
            self.find_virtWall_pose(imgC, imgD, box)

    def find_virtWall_pose(self, color: Image, depth: Image, box: BoundingBox2D):
        contouredImg = color.copy()
        color = cv2.GaussianBlur(color, (7, 7), 1) #blur img
        color = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY) #convert img to grayscale

        color = cv2.Canny(color, self.canny_thre1, self.canny_thre2) #get edges

        dilKernel = np.ones((3, 3))
        color = cv2.dilate(color, dilKernel) #dilate img

        contours, hierarchy = cv2.findContours(color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #get contours
        approx = [[[0, 0]]] #prevent getting an error that code tries to iterate over an empty approx
        for cont in contours:
            area = cv2.contourArea(cont)

            if area > 1000: #checking the area helps reducing noise
                cv2.drawContours(contouredImg, contours, -1, (255, 0, 255), 3)

                perimeter = cv2.arcLength(cont, True) #get shape perimeter
                approx = cv2.approxPolyDP(cont, 0.02 * perimeter, True) #approximate what shape it is
                print("Wall has " + str(len(approx)) + " points with an area of " + str(area))

        msg: Fourpoints = Fourpoints()
        msg.name = 0
        corner_pos = []
        
        for point in approx:
            p = point[0]
            pixel_pos: Pose2D = Pose2D
            pixel_pos.x = p[0]
            pixel_pos.y = p[1]
            pixel_pos.theta = 0.0
            print("x:", pixel_pos.x, "y:", pixel_pos.y)

            pixel_pos = self.uncropIMG(box, pixel_pos)

            x = pixel_pos.x
            y = pixel_pos.y
            pos = self.findPixelCoords(x, y, self.depth_img)
            
            #if approx.index(point) <= 3: #prevent the list from having more than 4 points
            corner_pos.append(pos)

        msg.corner1 = corner_pos[0]
        msg.corner2 = corner_pos[1]
        msg.corner3 = corner_pos[2]
        msg.corner4 = corner_pos[3]
        
        cv2.imshow("Virtual Wall", contouredImg)
        cv2.waitKey(1)
        
        self.virtWall_pub.publish(msg)

    #--------------------Utils--------------------
    def cropIMG(self, img: Image, bbox: BoundingBox2D):
        box_center = bbox.center.position

        corner1 = [0, 0]
        corner1[0] = float(box_center.x - (bbox.size_x / 2))
        corner1[1] = float(box_center.y - (bbox.size_y / 2))

        corner2 = [0, 0]
        corner2[0] = float(box_center.x + (bbox.size_x / 2))
        corner2[1] = float(box_center.y + (bbox.size_y / 2))

        #In slicing an image, x and y are flipped
        i = img[int(corner1[1]):int(corner2[1]),
                int(corner1[0]):int(corner2[0])].copy()

        return i
    
    def uncropIMG(self, box: BoundingBox2D, pixel: Pose2D) -> Pose2D: #Take pixel pos in cropped img and transform into pos in uncropped img
        new_pos: Pose2D = Pose2D
        new_pos.x = int((box.center.position.x - (box.size_x / 2)) + pixel.x)
        new_pos.y = int((box.center.position.y - (box.size_y / 2)) + pixel.y)
        new_pos.theta = 0.0
        return new_pos

    def fart(self, intensity: int) -> None:
        print("Node farted with an intensity of", str(intensity), "kiloFarts")

    def findPixelCoords(self, x: int, y: int, d: Image) -> Point:
        coords: Point = Point()

        invFx = 1/self.Fx
        invFy = 1/self.Fy

        coords.x = d[y][x] * 0.001
        coords.y = ((x - 320) * coords.x * invFx) + 0.032 #320 is the width of the img divided by 2 (aka cx);
        #                                                  0.032 is the distance between the depth module and the center of the cam
        coords.z = (y - self.Cy) * coords.x * invFy
        

        return coords
        



#--------------------Main--------------------
def main(args=None):
    rclpy.init(args=args)

    pose_estimator = PoseEstimator()
    rclpy.spin(pose_estimator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()