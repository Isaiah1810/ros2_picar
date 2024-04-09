from rclpy.node import Node
import rclpy
from ackermann_msgs.msg import AckermannDrive
from velocity_controller import DriveInterface, SteeringInterface
from std_msgs.msg import String

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__("driving_subcriber")
        self.ack_subscriber = self.create_subscription(AckermannDrive, "ack_vel", self.subscription_callback, 10)
        self.e_stop_subscriber = self.create_subscription(String, "e_stop", self.e_stop_callback, 10)
        self.driver = DriveInterface()
        self.steering = SteeringInterface()
    
    def subscription_callback(self, msg):
        self.steering.set_dir(msg.steering_angle)
        self.driver.set_motor_velocity(msg.speed)

    def e_stop_callback(self, msg):
        self.driver.set_motor_velocity(0)
        self.steering.set_dir(0)