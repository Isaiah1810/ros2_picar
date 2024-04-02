import cv2
import time
import subprocess
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading
from datetime import datetime
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

# RTSP server configuration
RTSP_URL = "rtsp://localhost:8554/live"
WIDTH = 640
HEIGHT = 480

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("rtsp_image_streamer")
        self.image_subcriber = self.create_subscription(Image, "image", self.image_callback, 10)
        self.yolo_subcriber = self.create_subscription(Image, "yolo_image", self.yolo_callback, 10)
        self.image = np.zeros([HEIGHT, WIDTH, 3], dtype=np.uint8)
        self.yolo = np.zeros([HEIGHT, WIDTH, 3], dtype=np.uint8)
        self.bridge = CvBridge()

        # Start the ffmpeg process to create the RTSP server
        ffmpeg_command = ['ffmpeg', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', '{}x{}'.format(WIDTH, HEIGHT), '-i', '-', '-rtsp_flags', 'listen', '-f', 'rtsp', RTSP_URL]
        self.ffmpeg_process = subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.publish_to_stream()

    def yolo_callback(self, msg):
        self.yolo = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.publish_to_stream()

    def publish_to_stream(self):
        # Concatenate images horizontally
        self.yolo = cv2.resize(self.yolo, (WIDTH, HEIGHT))
        self.image = cv2.resize(self.image, (WIDTH, HEIGHT))
        #concatenated_image = np.concatenate((self.yolo, self.image), axis=1)

        # Write concatenated image to ffmpeg process stdin
        self.ffmpeg_process.stdin.write(self.yolo.tobytes())

def main():
    try:
        rclpy.init()
        s = ImageSubscriber()
        rclpy.spin(s)
    except KeyboardInterrupt:
        print('Shutting down the server')
        s.ffmpeg_process.kill()

if __name__ == '__main__':
    main()
