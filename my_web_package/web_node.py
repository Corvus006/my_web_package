import rclpy
from rclpy.node import Node
from flask import Flask, render_template
from threading import Thread
import os


class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.get_logger().info('Starting WebNode')
        self.app = Flask(__name__)
        
        @self.app.route('/')
        def home():
            return render_template('index.html')

        # Starte Flask in einem separaten Thread, um die ROS2-Node nicht zu blockieren
        self.web_thread = Thread(target=self.run_web)
        self.web_thread.start()

    def run_web(self):
        self.app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('WebNode wurde gestoppt.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
