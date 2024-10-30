import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2D, BoundingBox2D
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D, Point, TransformStamped, Pose

from cv_bridge import CvBridge, CvBridgeError
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from custom_msgs.msg import Fourpoints

import cv2
import numpy as np

bridge = CvBridge()

class PoseEstimator(Node):

    #--------------------Initialize--------------------
    def __init__(self):
        super().__init__('object_pose_estimator')
        
        self.cam_img = cv2.cvtColor(cv2.imread('/home/gabri/at_work/src/yolo_atwork/yolo_atwork/thor.png'), cv2.COLOR_RGB2GRAY) #prevents this stupid code from trying to use a variable with no value
        self.depth_img = cv2.cvtColor(cv2.imread('/home/gabri/at_work/src/yolo_atwork/yolo_atwork/luna_depth.png'), cv2.COLOR_RGB2GRAY)

        self.canny_thre1: int = 30
        self.canny_thre2: int = 50

        self.Fx: float = 1.0
        self.Fy: float = 1.0
        self.Cx: float = 1.0
        self.Cy: float = 1.0
        
        self.cameraCoefs = np.zeros((3, 3))
        self.rotMat = np.zeros((3, 3))
        
        self.i: int = 0 #Give a identification number to tfs/msgs to prevent two having the same name
        
        self.cam_rot = np.deg2rad(30)

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
                                                   'yolo/object_detection',
                                                   self.find_pose,
                                                   10)
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    #--------------------Receive info--------------------
    def receive_cam_coefs(self, msg: CameraInfo):
        self.Fx = msg.k[0] #562.10894889
        self.Fy = msg.k[4] #560.09619249
        self.Cx = msg.k[2] #357.50110921
        self.Cy = msg.k[5] #248.34527238
        
        self.cameraCoefs[0, 0] = self.Fx
        self.cameraCoefs[0, 2] = self.Cx
        self.cameraCoefs[1, 1] = self.Fy
        self.cameraCoefs[1, 2] = self.Cy
        self.cameraCoefs[2, 2] = 1.0
        
        self.rotMat[1, 1] = 1.0
        self.rotMat[0, 0] = np.cos(self.cam_rot)
        self.rotMat[2, 0] = -np.sin(self.cam_rot)
        self.rotMat[0, 2] = np.sin(self.cam_rot)
        self.rotMat[2, 2] = np.cos(self.cam_rot)

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
        
        #Detect contours
        #imgC = cv2.GaussianBlur(imgC, (7, 7), 1) #blur img
        
        bins=np.array([0, 51, 102, 153, 204, 255])
        imgC[:, :, :] = np.digitize(imgC[:, :, :], bins, right=True) * 51
        
        imgC = cv2.cvtColor(imgC, cv2.COLOR_BGR2GRAY) #convert img to grayscale
        
        cv2.imshow("ImgC", imgC) 

        imgC = cv2.Canny(imgC, self.canny_thre1, self.canny_thre2) #get edges

        dilKernel = np.ones((3, 3))
        imgC = cv2.dilate(imgC, dilKernel) #dilate img

        contours, hierarchy = cv2.findContours(imgC, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #get contours
        approx = [[[0, 0]]] #prevent getting an error that code tries to iterate over an empty approx
        contouredImg = imgC.copy()
        for cont in contours:
            area = cv2.contourArea(cont)

            if area > 1000: #checking the area helps reducing noise
                cv2.drawContours(contouredImg, contours, -1, (255, 0, 255), 3)

                perimeter = cv2.arcLength(cont, True) #get shape perimeter
                approx = cv2.approxPolyDP(cont, 0.02 * perimeter, True) #approximate what shape it is
                #self.get_logger().info("Object has " + str(len(approx)) + " points with an area of " + str(area))

        
        
        
        #Get position (middle)
        corner_pos = []
        averageYPos = 0
        for i, point in enumerate(approx):
            if i == 0: #For some reason, the first element is always wrong and it is too much work to fix it
                continue
            
            p = point[0]
            pixel_pos: Pose2D = Pose2D
            pixel_pos.x = p[0]
            pixel_pos.y = p[1]
            pixel_pos.theta = 0.0

            pixel_pos = self.uncropIMG(box, pixel_pos)
            
            #Take the position of the pixel more "inside" the shape, to prevent getting part of the table position
            if pixel_pos.x <= box.center.position.x - box.size_x / 4:
                pixel_pos.x += 5
            elif pixel_pos.x >= box.center.position.x + box.size_x / 4:
                pixel_pos.x -= 5
            
            if pixel_pos.y <= box.center.position.y - box.size_y / 4:
                pixel_pos.y += 5
            elif pixel_pos.y >= box.center.position.y + box.size_y / 4:
                pixel_pos.y -= 5

            x = pixel_pos.x
            y = pixel_pos.y
            pos = self.findPixelCoords(x, y, self.depth_img)
            
            if pos.x == 0:
                continue
            
            #Sun Tzu: A Arte da Gambiarra 
            if i == 1:
                averageYPos = pos.y
            else:
                if abs(averageYPos) - abs(pos.y) <= -7 or abs(averageYPos) - abs(pos.y) >= 7:
                    continue
                else:
                    averageYPos += pos.y
                    averageYPos /= i
            
            #if approx.index(point) <= 3: #prevent the list from having more than 4 points
            corner_pos.append(pos)
        
        pos: Pose = Pose()
        
        """Because of noise in  the depth image, corner.x may be 0 and, because of that, y and z will be fucked up.
            To prevent having a [0, 0, 0] on one of these arrays, before adding a value, we check first if corner.x is higher than 0.
            Also, the lowestPos array starts as [1000, 1000, 1000], so anything higher than 0 (a true value) can replace it, and the same thing for highestPos"""
        highestPos = [0, 0, 0]
        lowestPos = [1000, 1000, 1000]
        for i, corner in enumerate(corner_pos):
            #if corner.x == 0:
                #continue
                        
            if corner.x < lowestPos[0]:
                lowestPos[0] = corner.x
            elif corner.x > highestPos[0]:
                highestPos[0] = corner.x
                
            if corner.y < lowestPos[1]:
                lowestPos[1] = corner.y
            elif corner.y > highestPos[1]:
                highestPos[1] = corner.y
                    
            if corner.z < lowestPos[2]:
                lowestPos[2] = corner.z
            elif corner.z > highestPos[2]:
                highestPos[2] = corner.z
                    
        #self.get_logger().info("lowest:" + str(lowestPos))
        #self.get_logger().info("highest:" + str(highestPos))
        #self.get_logger().info("--------------------------")
        
        pos.position.x = (lowestPos[0] + highestPos[0]) / 2.0
        pos.position.y = (lowestPos[1] + highestPos[1]) / 2.0
        pos.position.z = (lowestPos[2] + highestPos[2]) / 2.0
        
        if pos.position.x == 500:
            self.get_logger().info("Couldnt find object position")
            return
        
        #Take the cam rotation in the y axis into account for the x and z axis
        OGx = pos.position.x
        OGz = pos.position.z
        pos.position.x = (OGx * np.cos(self.cam_rot)) + (OGz * np.sin(self.cam_rot))
        pos.position.z = -(OGx * np.sin(self.cam_rot)) + (OGz * np.cos(self.cam_rot))
        
        self.get_logger().info("x: " + str(pos.position.x) + ", y: " + str(pos.position.y) + ", z: " + str(pos.position.z))
        
        

        cv2.imshow("Object", contouredImg)
        cv2.waitKey(1)
        
        self.i += 1
    
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
        
        new_pos.x = (int(box.center.position.x - int(box.size_x / 2)) + pixel.x)
        new_pos.y = (int(box.center.position.y - int(box.size_y / 2)) + pixel.y)
        new_pos.theta = 0.0
        
        return new_pos

    def fart(self, intensity: int) -> None:
        self.get_logger().info("Node farted with an intensity of " + str(intensity) + " kiloFarts")

    def findPixelCoords(self, x: float, y: float, d: Image) -> Point:
        coords: Point = Point()

        invFx = 1/self.Fx
        invFy = 1/self.Fy

        coords.x = d[y][x] * 0.001
        coords.y = ((x - self.Cx) * coords.x * invFx) + 0.037 #0.037 is the distance between the depth module and the center of the cam
        coords.z = (y - self.Cy) * coords.x * invFy
        
        return coords

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
        
        self.tf_static_broadcaster.sendTransform(transform)
        

#--------------------Main--------------------
def main(args=None):
    rclpy.init(args=args)

    pose_estimator = PoseEstimator()
    rclpy.spin(pose_estimator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()