import numpy as np
import cv2
from rknnlite.api import RKNNLite
import time
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from yolo_utils import yolov5_post_process


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
        self.publisher = self.create_publisher(String, 'person', 10)

    def detect(self):
        rknn_lite = RKNNLite(verbose=False)
        rknn_lite.load_rknn(RKNN_MODEL)
        ret = rknn_lite.init_runtime(core_mask=rknn_lite.NPU_CORE_0_1_2)
        cap = cv2.VideoCapture('/dev/video21')
        while True:
            ret, img = cap.read()
            img = cv2.resize(img, (640,640))
            img2 = np.expand_dims(img, 0)
            outputs = rknn_lite.inference(inputs=[img2], data_format=['nhwc'])
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
            msg = String()
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
                msg.data = str(points)
            self.publisher.publish(msg)
        
if __name__ == '__main__':
    try:
        rclpy.init()
        pub = Publisher()
        pub.detect()
    except KeyboardInterrupt:
        rclpy.shutdown()