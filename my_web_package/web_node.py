import cv2
import time
import numpy as np
from flask import Flask, render_template, Response, request, jsonify
from threading import Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.get_logger().info('Starting WebNode')
        self.app = Flask(__name__)

        # Subscriber for compressed images
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
            linear_x = data.get('linear_x')
            linear_y = data.get('linear_y')
            angular_z = data.get('angular_z')
            enable_robot = data.get('enable_robot')
            disable_robot = data.get('disable_robot')
            image_index = data.get('image_index')

            print(f"Received control data: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}")
            print(f"Enable robot: {enable_robot}, Disable robot: {disable_robot}")
            
            # Update the selected image
            image_topics = ["front", "rear", "gripper", "thermal", "motion"]

            # Ensure image_index and image_topics are valid
            if image_index is None or not isinstance(image_index, int):
                return jsonify({"error": "Invalid image_index"}), 400
            
            if image_topics is None or not isinstance(image_topics, list):
                return jsonify({"error": "Invalid image_topics"}), 400

            if 0 <= image_index < len(image_topics):
                self.picked_image = image_topics[image_index]
            else:
                print(f"Invalid image index: {image_index}")
                return jsonify({"error": "Image index out of range"}), 400

            return jsonify({"status": "success"})

        # Start Flask in a separate thread to avoid blocking the ROS2 node
        self.web_thread = Thread(target=self.run_web)
        self.web_thread.start()

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
