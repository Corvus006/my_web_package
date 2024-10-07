import cv2
import time
import numpy as np
from flask import Flask, render_template, Response, request, jsonify
from threading import Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool  # Importiere den SetBool Service

class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.get_logger().info('Starting WebNode')
        self.app = Flask(__name__)

        # Initialize Twist message
        self.twist_msg = Twist()

        self.cliWorks = False

        # Service Client for SetBool
        self.enable_client = self.create_client(SetBool, 'eduard/enable')

        # Subscriber for Images
        self.front_subscriber = self.create_subscription(
            CompressedImage,
            '/front/compressed',
            self.front_image_callback,
            10)
        
        self.rear_subscriber = self.create_subscription(
            CompressedImage,
            '/rear/compressed',
            self.rear_image_callback,
            10)
        
        self.gripper_subscriber = self.create_subscription(
            CompressedImage,
            '/gripper/compressed',
            self.gripper_image_callback,
            10)
        
        self.thermal_subscriber = self.create_subscription(
            CompressedImage,
            '/thermal/compressed',
            self.thermal_image_callback,
            10)
        
        self.motion_subscriber = self.create_subscription(
            CompressedImage,
            '/motion/compressed',
            self.motion_image_callback,
            10)

        self.images = {
            "front": None,
            "rear": None,
            "gripper": None,
            "thermal": None,
            "motion": None
        }

        self.picked_image = "front"  # Default image

        # Twist publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.twist_thread = Thread(target=self.publish_twist_loop)
        self.twist_thread.start()

        @self.app.route('/')
        def home():
            return render_template('index.html')

        # Video stream route
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_image_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/control_data', methods=['POST'])
        def control_data():
            data = request.json
            try:
                linear_x = float(data.get('linear_x', 0.0))  # Ensure the value is a float
                linear_y = float(data.get('linear_y', 0.0))
                angular_z = float(data.get('angular_z', 0.0))
                image_index = data.get('image_index')
                enable = data.get('enable', False)
                disable= data.get('disable',False) 
            except ValueError:
                return jsonify({"error": "Invalid control data values"}), 400

            print(f"Received control data: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}, disable={disable}, enable={enable}")
            # Update the twist message (this will be published at 60Hz in a loop)
            self.update_twist(linear_x, linear_y, angular_z)
            
            if enable  and not self.cliWorks:
                self.cliWorks = True
                self.call_enable_service(True)
                self.cliWorks = False
            elif disable  and not self.cliWorks:
                self.cliWorks = True
                self.call_enable_service(False)
                self.cliWorks = False
            elif enable  and disable and not self.cliWorks:
                enable = False
                disable = False
            elif self.cliWorks:
                print("Something did go wrong with the service")
            
            # Update the selected image
            image_topics = ["front", "rear", "gripper", "thermal", "motion"]
            if image_index is not None and 0 <= image_index < len(image_topics):
                self.picked_image = image_topics[image_index]
            else:
                return jsonify({"error": "Invalid image index"}), 400

            return jsonify({"status": "success"})


        self.web_thread = Thread(target=self.run_web)
        self.web_thread.start()

    def call_enable_service(self, status):
        req = SetBool.Request()
        req.data = status 
        future = self.enable_client.call_async(req)

        # Wait for the service to complete, with a timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)  # Timeout after 5 seconds

        if future.result() is not None:
            self.get_logger().info(f'Service call succeeded: {future.result().message}')
        elif future.done() and future.result() is None:
            self.get_logger().error('Service call failed: No response from server.')
        else:
            self.get_logger().error('Service call failed: Service timed out or encountered an error.')

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
        """Helper function to decode a compressed ROS2 image message."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def generate_image_stream(self):
        while True:
            if self.images[self.picked_image] is not None:
                # Convert image to JPEG
                ret, jpeg = cv2.imencode('.jpg', self.images[self.picked_image])
                frame = jpeg.tobytes()
                
                # Stream the image in the correct format
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            time.sleep(0.1)

    def update_twist(self, linear_x, linear_y, angular_z):
        """Update the Twist message with new control data."""
        self.twist_msg.linear.x = linear_x
        self.twist_msg.linear.y = linear_y
        self.twist_msg.angular.z = angular_z
        print(f"Updated Twist: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}")

    def publish_twist_loop(self):
        """Publish the Twist message at 60Hz."""
        rate = self.create_rate(60)  # 60Hz
        while rclpy.ok():
            self.cmd_vel_publisher.publish(self.twist_msg)
            rate.sleep()

    def run_web(self):
        self.app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('WebNode stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
