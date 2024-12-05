import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point
import cv2
import torch
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, \
    strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized,\
    TracedModel

from utils.datasets import letterbox

import os
from glob import glob
from ament_index_python.packages import get_package_share_directory

from detect_interfaces.msg import BoundingBox  # Import the custom message type


class ObjectDetection(Node):
    def __init__(self):
        super().__init__("ObjectDetection")
        # Parameters
        self.declare_parameter("weights", "sign.pt", ParameterDescriptor(description="Weights file"))
        self.declare_parameter("conf_thres", 0.25, ParameterDescriptor(description="Confidence threshold"))
        self.declare_parameter("iou_thres", 0.45, ParameterDescriptor(description="IOU threshold"))
        self.declare_parameter("device", "", ParameterDescriptor(description="Name of the device"))
        self.declare_parameter("img_size", 640, ParameterDescriptor(description="Image size"))

        weights_file = self.get_parameter("weights").get_parameter_value().string_value
        self.weights = os.path.join(get_package_share_directory('object_detection'), 'weights', weights_file)
        self.conf_thres = self.get_parameter("conf_thres").get_parameter_value().double_value
        self.iou_thres = self.get_parameter("iou_thres").get_parameter_value().double_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.img_size = self.get_parameter("img_size").get_parameter_value().integer_value

        # Camera info and frames
        self.rgb_image = None

        self.cv_bridge = CvBridge()
        
        # Subscribers
        self.rs_sub = self.create_subscription(Image, '/image/image_raw', self.rs_callback, 1)

        #pub
        self.publisher = self.create_publisher(String, '/detect/signs', 1)  # Create a publisher for object labels

        # Publisher for bounding box coordinates
        self.bb_publisher = self.create_publisher(BoundingBox, '/detect/bounding_box', 1)

        self.signs = String()

        # Initialize YOLOv7
        set_logging()
        self.device = select_device(self.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(self.weights, map_location=self.device) # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.img_size, s=self.stride)  # check img_size
        self.imgsz = imgsz
        if self.half:
            self.model.half()  # to FP16

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))
        self.old_img_w = self.old_img_h = imgsz
        self.old_img_b = 1

    def rs_callback(self, msg):
        # 將ROS Image轉換成OpenCV格式
        self.rgb_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.YOLOv7_detect()

    def preProccess(self, img):
        img=torch.from_numpy(img).to(self.device)
        img=img.half() if self.half else img.float()  # uint8 to fp16/32
        img=img/255.0
        if len(img.shape)==3:
            img=img[None]
        return img

    def YOLOv7_detect(self):
        img = self.rgb_image
        im0 = img.copy()

        img=letterbox(im0, self.imgsz, stride=self.stride)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  #BGR to RGB
        img = np.ascontiguousarray(img)
        img=self.preProccess(img)

        # Warmup
        if self.device.type != 'cpu' and (self.old_img_b != img.shape[0] or self.old_img_h != img.shape[2] or self.old_img_w != img.shape[3]):
            self.old_img_b = img.shape[0]
            self.old_img_h = img.shape[2]
            self.old_img_w = img.shape[3]
            for i in range(3):
                self.model(img)[0]

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = self.model(img)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)
        t3 = time_synchronized()

        # Process detections   
        detect = False
        choice_highest_cls = False
        found_cls = []

        for i, det in enumerate(pred):  # detections per image
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class

                # determine the hightes sign

                class_indices = [row[-1].item() for row in det]
                if 2.0 in class_indices and 6.0 in class_indices: # left and right
                    choice_highest_cls = True

                    # Dictionary to store class indices and corresponding values
                    class_value_map = {}

                    # Iterate over tensor rows
                    for row in det:
                        # Extract class index and value
                        cls_index = int(row[-1].item())
                        value_ymax = row[3].item()  # Assuming the value you want to extract is always at index 3, change if needed
                        value_ymin = row[1].item()

                        # Store the value corresponding to the class index
                        class_value_map[cls_index] = (value_ymax+value_ymin)/2

                    # Find the values corresponding to class indices 2 and 6
                    value_2 = class_value_map.get(2, None)
                    value_6 = class_value_map.get(6, None)

                    # print(value_2, value_6)

                    if value_2 > value_6:   ## choice left
                        found_cls = [key for key, value in class_value_map.items() if value == value_6]
                    else:
                        found_cls = [key for key, value in class_value_map.items() if value == value_2]

                    # print(found_cls)    

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    label = f'{self.names[int(cls)]} {conf:.2f}'

                    if conf > 0.75: # Limit confidence threshold to 50% for all classes
                        # print(int(cls))
                        if choice_highest_cls and int(cls) != int(found_cls[0]):
                            # print(int(cls), int(found_cls[0]))
                            choice_highest_cls = False
                            continue

                        # Draw a boundary box around each object
                        plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=2)

                        # Publish bounding box coordinates
                        bbox_msg = BoundingBox()
                        bbox_msg.xmin = int(xyxy[0])
                        bbox_msg.ymin = int(xyxy[1])
                        bbox_msg.xmax = int(xyxy[2])
                        bbox_msg.ymax = int(xyxy[3])
                        self.bb_publisher.publish(bbox_msg)

                        self.signs.data = self.names[int(cls)]
                        self.publisher.publish(self.signs)
                        detect = True

            if detect == False:
                self.signs.data = ""
                self.publisher.publish(self.signs)

            cv2.imshow("YOLOv7 Object detection result RGB", cv2.resize(im0, None, fx=1.5, fy=1.5))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def main(args=None):
    """Run the main function."""
    rclpy.init(args=args)
    with torch.no_grad():
        node = ObjectDetection()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
