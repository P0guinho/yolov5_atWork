import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D, Point, TransformStamped, Pose, PoseWithCovariance
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tutorial_interfaces.msg import Atworkobjects, Atworkobjectsarray

from PIL import Image as IMG #If we import it as Image it conflits with ros' Image and explodes the robot

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
                
        self.cam_rot = np.deg2rad(30)
        
        self.object_ids = {
            "BarraPretaMenor": 1,
            "BarraCinzaMenor" : 2,
            "BarraPretaMaior" : 3,
            "BarraCinzaMaior" : 4,
            "Parafuso" : 5,
            #"Porca" : 6,
            "Porca" : 7,   
        }
        self.object_colors = {
            "BarraPretaMenor": "Preta",
            "BarraCinzaMenor" : "Cinza",
            "BarraPretaMaior" : "Preta",
            "BarraCinzaMaior" : "Cinza",
            "Parafuso" : "Preta",
            #"Porca" : "Cinza",
            "Porca" : "Cinza",   
        }

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
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.object_pub = self.create_publisher(Atworkobjects,
                                                'yolo/object_poses',
                                                10)

    #--------------------Receive info--------------------
    def receive_cam_coefs(self, msg: CameraInfo):
        self.Fx = 606.1556396484375 #msg.k[0]
        self.Fy = 605.013427734375 #msg.k[4]
        self.Cx = 341.7103271484375 #msg.k[2]
        self.Cy = 241.36676025390625 #msg.k[5]
        
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
        
        self.get_logger().info(str(self.cameraCoefs))

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
        
        if len(imgC) == 0:
            return
        
        #Detect contours
        #imgC = cv2.GaussianBlur(imgC, (7, 7), 1) #blur img
        
        bins=np.array([0, 51, 102, 153, 204, 255])
        imgC[:, :, :] = np.digitize(imgC[:, :, :], bins, right=True) * 51
        
        imgC = cv2.cvtColor(imgC, cv2.COLOR_BGR2GRAY) #convert img to grayscale
        
        cv2.imshow("ImgC", imgC)
        
        #Resize the image to take the position of the contours pixels more "inside" the shape, to prevent getting part of the table position
        pilIMG = IMG.fromarray(imgC)        
        pilIMG.thumbnail((pilIMG.size[0] / 1.25, pilIMG.size[1] / 1.25))
        newIMG = np.asarray(pilIMG)
        
        newIMG = cv2.Canny(newIMG, self.canny_thre1, self.canny_thre2) #get edges

        dilKernel = np.ones((3, 3))
        newIMG = cv2.dilate(newIMG, dilKernel) #dilate img

        contours, hierarchy = cv2.findContours(newIMG, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #get contours
        approx = [[[0, 0]]] #prevent getting an error that code tries to iterate over an empty approx
        contouredImg = newIMG.copy()
        for cont in contours:
            area = cv2.contourArea(cont)

            if area > 1000: #checking the area helps reducing noise
                cv2.drawContours(contouredImg, contours, -1, (255, 0, 255), 3)

                perimeter = cv2.arcLength(cont, True) #get shape perimeter
                approx = cv2.approxPolyDP(cont, 0.02 * perimeter, True) #approximate what shape it is
                #self.get_logger().info("Object has " + str(len(approx)) + " points with an area of " + str(area))

        
        
        
        #Get position (middle)
        corner_pos = []
        for i, point in enumerate(approx):
            p = point[0]
            pixel_pos: Pose2D = Pose2D
            pixel_pos.x = p[0]
            pixel_pos.y = p[1]
            pixel_pos.theta = 0.0
            
            #Transform pixel pos in the small image to the pos in the normal image (len(imgC) is the length in the y axis and len(imgC[0]) in the x axis)
            pixel_pos.x = p[0] + ((len(imgC[0]) - len(newIMG[0])) / 2)
            pixel_pos.y = p[1] + ((len(imgC) - len(newIMG)) / 2)

            pixel_pos = self.uncropIMG(box, pixel_pos)
            
            x = pixel_pos.x
            y = pixel_pos.y
            pos = self.findPixelCoords(int(x), int(y), self.depth_img)
        
            
            if pos.x == 0:
                continue
            
            corner_pos.append(pos)
        
        pos: Pose = Pose()
        
        """Because of noise in  the depth image, corner.x may be 0 and, because of that, y and z will be fucked up.
            To prevent having a [0, 0, 0] on one of these arrays, before adding a value, we check first if corner.x is higher than 0.
            Also, the lowestPos array starts as [1000, 1000, 1000], so anything higher than 0 (a true value) can replace it, and the same thing for highestPos"""
        highestPos = [0, 0, 0]
        lowestPos = [1000, 1000, 1000]
        
        for i, corner in enumerate(corner_pos):
            if corner.x < lowestPos[0]:
                lowestPos[0] = corner.x
            elif corner.x > highestPos[0]:
                highestPos[0] = corner.x
                    
            if corner.z < lowestPos[2]:
                lowestPos[2] = corner.z
            elif corner.z > highestPos[2]:
                highestPos[2] = corner.z
                
        
        
        pos.position.x = (lowestPos[0] + highestPos[0]) / 2.0
        pos.position.y = -(self.findPixelCoords(int(box.center.position.x), int(box.center.position.y), self.depth_img).y) #Y is inverted
        pos.position.z = (lowestPos[2] + highestPos[2]) / 2.0
        
        pos.orientation.x = 0.0
        pos.orientation.y = 0.0
        pos.orientation.z = 0.0
        pos.orientation.w = 1.0
        
        if pos.position.x == 500:
            self.get_logger().info("Couldnt find object position")
            return
        
        self.generateTF("camera_depth_frame", msg.id, pos)
        self.get_logger().info("x: " + str(pos.position.x) + ", y: " + str(pos.position.y) + ", z: " + str(pos.position.z))
        
        obj = Atworkobjects()
        if self.object_ids.get(msg.id) == None:
            return
        
        obj.id = self.object_ids.get(msg.id)
        obj.name = msg.id
        obj.color = self.object_colors.get(msg.id)
        obj.detection = msg
        
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = obj.name
        result.hypothesis.score = 10000.0
        result.pose.pose = pos
        obj.detection.results.append(result)
        
        self.object_pub.publish(obj)
        
        

        cv2.imshow("Object", contouredImg)
        cv2.waitKey(1)
    
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
        coords.y = ((x - self.Cx) * coords.x * invFx) #+ 0.037 #0.037 is the distance between the depth module and the center of the cam
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
        
        self.tf_broadcaster.sendTransform(transform)
        

#--------------------Main--------------------
def main(args=None):
    rclpy.init(args=args)

    pose_estimator = PoseEstimator()
    rclpy.spin(pose_estimator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()