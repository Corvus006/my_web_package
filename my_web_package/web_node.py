import rclpy
from rclpy.node import Node
from flask import Flask, render_template_string
from threading import Thread

class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.get_logger().info('Starting WebNode')
        self.app = Flask(__name__)

        @self.app.route('/')
        def home():
            return render_template_string('<h1>Willkommmen zu meiner ROS2-Node Website!</h1>')
        
        self.web_thread = Thread(target=self.run_web)
        self.web_thread.start()
    
    def run_web(self):
        self.app.run(host='0.0.0.0',port=5000)

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
    
