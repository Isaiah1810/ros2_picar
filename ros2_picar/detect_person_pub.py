import numpy as np
import cv2
from rknnlite.api import RKNNLite
import time
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from yolo_utils import yolov5_post_process
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cvzone

OBJ_THRESH = 0.25
NMS_THRESH = 0.45
IMG_SIZE = 640
RKNN_MODEL = 'yolov5n.rknn'

CLASSES = ("person", "bicycle", "car", "motorbike ", "aeroplane ", "bus ", "train", "truck ", "boat", "traffic light",
           "fire hydrant", "stop sign ", "parking meter", "bench", "bird", "cat", "dog ", "horse ", "sheep", "cow", "elephant",
           "bear", "zebra ", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
           "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife ",
           "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza ", "donut", "cake", "chair", "sofa",
           "pottedplant", "bed", "diningtable", "toilet ", "tvmonitor", "laptop	", "mouse	", "remote ", "keyboard ", "cell phone", "microwave ",
           "oven ", "toaster", "sink", "refrigerator ", "book", "clock", "vase", "scissors ", "teddy bear ", "hair drier", "toothbrush ")

class Publisher(Node):
    def __init__(self):
        super().__init__('person_detecor_publisher')
        self.subsciber = self.create_subscription(Image, "image", self.image_callback, 10)
        self.publisher = self.create_publisher(Image, 'yolo_image', 10)
        self.frame = np.zeros((640, 640, 3))
        self.bridge = CvBridge()
        self.rknn_lite = RKNNLite(verbose=False)
        self.rknn_lite.load_rknn(RKNN_MODEL)
        ret = self.rknn_lite.init_runtime(core_mask=self.rknn_lite.NPU_CORE_0_1_2)
    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.new_frame = self.frame
        self.detect()

    def detect(self):
            img = self.frame
            img = cv2.resize(img, (640,640))
            img2 = np.expand_dims(img, 0)
            outputs = self.rknn_lite.inference(inputs=[img2], data_format=['nhwc'])
            if (not isinstance(outputs, type(None))):
                input0_data = outputs[0]
                input1_data = outputs[1]
                input2_data = outputs[2]
                input0_data = input0_data.reshape([3, -1]+list(input0_data.shape[-2:]))
                input1_data = input1_data.reshape([3, -1]+list(input1_data.shape[-2:]))
                input2_data = input2_data.reshape([3, -1]+list(input2_data.shape[-2:]))
                input_data = list()
                input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
                input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
                input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))
                boxes, classes, scores = yolov5_post_process(input_data)
                points = []
                if(not isinstance(boxes, type(None)) and not isinstance(classes, type(None))
                and not isinstance(scores, type(None))):
                    for box, score, cl in zip(boxes, scores, classes):
                        top, left, right, bottom = box
                        top = int(top)
                        left = int(left)
                        right = int(right)
                        bottom = int(bottom)
                        mid = ((left + (right-left)//2), (bottom + (bottom+top)//2))
                        points.append((mid, score, CLASSES[cl]))
                    
                msg = self.bridge.cv2_to_imgmsg(self.new_frame, "passthrough")
                self.publisher.publish(msg)
        
if __name__ == '__main__':
    try:
        rclpy.init()
        pub = Publisher()
        rclpy.spin(pub)
    except KeyboardInterrupt:
        rclpy.shutdown()