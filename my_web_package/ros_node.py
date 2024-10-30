import os
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from threading import Thread

class ROSNode(Node):
    def __init__(self):
        super().__init__('ros_node')
        self.get_logger().info('Starting ROSNode')

        # Declare parameters with default values
        self.declare_parameter('front_topic', '/front/compressed')
        self.declare_parameter('rear_topic', '/rear/compressed')
        self.declare_parameter('gripper_topic', '/gripper/compressed')
        self.declare_parameter('thermal_topic', '/thermal/compressed')
        self.declare_parameter('motion_topic', '/motion/compressed')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_service', '/enable')

        # Retrieve parameters
        self.front_topic = self.get_parameter('front_topic').get_parameter_value().string_value
        self.rear_topic = self.get_parameter('rear_topic').get_parameter_value().string_value
        self.gripper_topic = self.get_parameter('gripper_topic').get_parameter_value().string_value
        self.thermal_topic = self.get_parameter('thermal_topic').get_parameter_value().string_value
        self.motion_topic = self.get_parameter('motion_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.enable_service = self.get_parameter('enable_service').get_parameter_value().string_value

        # Get the directory where this script is located
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.placeholder_path = os.path.join(script_dir, 'static', 'images', 'placeholder.png')

        # Initialize Twist message
        self.twist_msg = Twist()
        self.cliWorks = False

        # Service Client for SetBool
        self.enable_client = self.create_client(SetBool, self.enable_service)

        # ROS2 Subscribers
        self.front_subscriber = self.create_subscription(
            CompressedImage, self.front_topic, self.front_image_callback, 10)

        self.rear_subscriber = self.create_subscription(
            CompressedImage, self.rear_topic, self.rear_image_callback, 10)

        self.gripper_subscriber = self.create_subscription(
            CompressedImage, self.gripper_topic, self.gripper_image_callback, 10)

        self.thermal_subscriber = self.create_subscription(
            CompressedImage, self.thermal_topic, self.thermal_image_callback, 10)

        self.motion_subscriber = self.create_subscription(
            CompressedImage, self.motion_topic, self.motion_image_callback, 10)

        self.images = {
            "front": None,
            "rear": None,
            "gripper": None,
            "thermal": None,
            "motion": None
        }

        self.picked_image = "front"  # Default image

        # Twist publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.twist_thread = Thread(target=self.publish_twist_loop)
        self.twist_thread.start()

    def call_enable_service(self, status):
        req = SetBool.Request()
        req.data = status 
        future = self.enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            self.get_logger().info(f'Service call succeeded: {future.result().message}')
        else:
            self.get_logger().error('Service call failed: No response from server.')

    def front_image_callback(self, msg):
        self.images["front"] = self.decode_image(msg)

    def rear_image_callback(self, msg):
        self.images["rear"] = self.decode_image(msg)

    def gripper_image_callback(self, msg):
        self.images["gripper"] = self.decode_image(msg)

    def thermal_image_callback(self, msg):
        self.images["thermal"] = self.decode_image(msg)

    def motion_image_callback(self, msg):
        self.images["motion"] = self.decode_image(msg)

    def decode_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def publish_twist_loop(self):
        rate = self.create_rate(60)  # 60Hz
        while rclpy.ok():
            self.cmd_vel_publisher.publish(self.twist_msg)
            rate.sleep()

    def update_twist(self, linear_x, linear_y, angular_z):
        self.twist_msg.linear.x = linear_x
        self.twist_msg.linear.y = linear_y
        self.twist_msg.angular.z = angular_z

def main(args=None):
    rclpy.init(args=args)
    node = ROSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
