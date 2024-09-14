import cv2
import time
import numpy as np
from flask import Flask, render_template, Response
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
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_topic/compressed',
            self.image_callback,
            10)
        
        self.current_image = None  # Placeholder for the current image

        @self.app.route('/')
        def home():
            return render_template('index.html')

        # Video stream route
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_image_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

        # Start Flask in a separate thread to avoid blocking the ROS2 node
        self.web_thread = Thread(target=self.run_web)
        self.web_thread.start()

    def image_callback(self, msg):
        # Decode the received image
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #self.get_logger().info('Image received and processed')

    def generate_image_stream(self):
        while True:
            if self.current_image is not None:
                # Convert image to JPEG
                ret, jpeg = cv2.imencode('.jpg', self.current_image)
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
