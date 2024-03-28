import rclpy
from rclpy import Node
import cv2
from time import time, sleep
from sensor_msgs.msg import Image
import numpy as np
import cv_bridge

fps = 15

width = 320
height = 240

class ServerSubscriber(Node):
    def __init__(self):
        super().__init__("rtsp_server_node")
        self.subscriber = self.create_subscription(Image, "image", self.image_callback, 10)
        self.bridge = cv_bridge.CvBridge()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        