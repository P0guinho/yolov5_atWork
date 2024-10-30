import rclpy
import os, sys
from rclpy.node import Node

from vision_msgs.msg import Detection2D
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
import time
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective
from utils.general import check_img_size, check_requirements, check_imshow, colorstr, non_max_suppression, \
    apply_classifier, scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from utils.plots import colors, plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        weights='/home/gabri/at_work/src/yolo_atwork/yolo_atwork/prototipo1.pt'  # model.pt path(s)
        self.imgsz=640  # inference size (pixels)
        self.conf_thres=0.7  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        self.classes=None  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False  # class-agnostic NMS
        self.augment=False  # augmented inference
        self.visualize=False  # visualize features
        self.line_thickness=3  # bounding box thickness (pixels)
        self.hide_labels=False  # hide labels
        self.hide_conf=True  # hide confidences
        self.half=False  # use FP16 half-precision inference
        self.stride = 32
        device_num='cpu'  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False  # show results
        save_crop=False  # save cropped prediction boxes
        nosave=False  # do not save images/videos
        update=False  # update all models
        name='exp'  # save results to project/name

        # Initialize
        set_logging()
        self.device = select_device(device_num)
        self.half &= self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(weights, map_location=self.device)  # load FP32 model
        stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.imgsz, s=stride)  # check image size
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names
        if self.half:
            self.model.half()  # to FP16

        # Second-stage classifier
        self.classify = False
        if self.classify:
            self.modelc = load_classifier(name='resnet50', n=2)  # initialize
            self.modelc.load_state_dict(torch.load('resnet50.pt', map_location=self.device)['model']).to(self.device).eval()

        # Dataloader
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once

        self.subscription = self.create_subscription(
            Image,
            'camera/camera/color/image_raw',
            self.camera_callback,
            10)
        self.publisher = self.create_publisher(
            Detection2D,
            "yolo/workspace_detection",
            10
        )
        self.subscription  # prevent unused variable warning
        self.publisher

    def camera_callback(self, data):
        t0 = time.time()
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        # check for common shapes
        s = np.stack([letterbox(x, self.imgsz, stride=self.stride)[0].shape for x in img], 0)  # shapes
        self.rect = np.unique(s, axis=0).shape[0] == 1  # rect inference if all shapes equal
        if not self.rect:
            print('WARNING: Different stream shapes detected. For optimal performance supply similarly-shaped streams.')

        # Letterbox
        img0 = img.copy()
        img = img[np.newaxis, :, :, :]        

        # Stack
        img = np.stack(img, 0)

        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = self.model(img,
                     augment=self.augment,
                     visualize=increment_path(save_dir / 'features', mkdir=True) if self.visualize else False)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        t2 = time_synchronized()

        # Apply Classifier
        if self.classify:
            pred = apply_classifier(pred, self.modelc, img, img0)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            s = f'{i}: '
            s += '%gx%g ' % img.shape[2:]  # print string

            if len(det):
                i: int = len(det) - 1
                
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
                
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    #label is the actual label
                    label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                    
                    #covariance is the text to be shown
                    covariance = f'{self.names[c]} {conf:.2f}'
                    plot_one_box(xyxy, img0, label=covariance, color=colors(c, True), line_thickness=self.line_thickness)

                #self.get_logger().info(str(len(det)))
                
                #for detectPoses in det:
                    msg = Detection2D()

                    #Message bounding box
                    msg.bbox.center.position.x = (float(det[i][0]) + float(det[i][2])) / 2.0
                    msg.bbox.center.position.y = (float(det[i][1]) + float(det[i][3])) / 2.0
                    msg.bbox.center.theta = 0.0

                    msg.bbox.size_x = float(det[i][2]) - float(det[i][0])
                    msg.bbox.size_y = float(det[i][3]) - float(det[i][1])
                    
                    #Detection id = Detection name/label
                    msg.id = label

                    self.publisher.publish(msg)
                    i -= 1 #go from the last element to the first because this loop reverses the array for some fucking reason


        cv2.imshow("IMAGE", img0)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
