import os
import cv2
import time
from flask import Flask, render_template, Response, request, jsonify
from my_web_package.ros_node import ROSNode
import rclpy
from threading import Thread

class WebApp:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.app = Flask(__name__)

        @self.app.route('/')
        def home():
            return render_template('index.html')

        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_image_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/control_data', methods=['POST'])
        def control_data():
            data = request.json
            try:
                linear_x = float(data.get('linear_x', 0.0))
                linear_y = float(data.get('linear_y', 0.0))
                angular_z = float(data.get('angular_z', 0.0))
                image_index = data.get('image_index')
                enable = data.get('enable', False)
                disable = data.get('disable', False)
            except ValueError:
                return jsonify({"error": "Invalid control data values"}), 400

            self.ros_node.update_twist(linear_x, linear_y, angular_z)
            
            if enable and not self.ros_node.cliWorks:
                self.ros_node.call_enable_service(True)
            elif disable and not self.ros_node.cliWorks:
                self.ros_node.call_enable_service(False)

            image_topics = ["front", "rear", "gripper", "thermal", "motion"]
            if image_index is not None and 0 <= image_index < len(image_topics):
                self.ros_node.picked_image = image_topics[image_index]
            else:
                return jsonify({"error": "Invalid image index"}), 400

            return jsonify({"status": "success"})

    def generate_image_stream(self):
        while True:
            if self.ros_node.images[self.ros_node.picked_image] is not None:
                ret, jpeg = cv2.imencode('.jpg', self.ros_node.images[self.ros_node.picked_image])
                frame = jpeg.tobytes()
            else:
                with open(self.ros_node.placeholder_path, 'rb') as f:
                    frame = f.read()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            time.sleep(0.1)

    def run(self):
        self.app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

def main():
    rclpy.init()
    ros_node = ROSNode()
    web_app = WebApp(ros_node)
    
    # Run Flask in a separate thread so ROS2 can continue running
    web_thread = Thread(target=web_app.run)
    web_thread.start()

    # Spin ROS node
    try:
        rclpy.spin(ros_node)
    finally:
        web_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()