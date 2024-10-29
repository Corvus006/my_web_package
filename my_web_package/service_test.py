import rclpy
from rclpy.node import Node
from edu_robot.srv import SetMode  # Import the SetMode service

class TestSetModeServer(Node):
    def __init__(self):
        super().__init__('test_set_mode_server')
        # Create the SetMode service
        self.srv = self.create_service(SetMode, '/wgg/set_mode', self.handle_set_mode)
        self.get_logger().info('Test SetMode server ready')

    def handle_set_mode(self, request, response):
        # Process the received request and provide a response
        self.get_logger().info(f'Received mode request: {request.mode._mode}')
        
        # Simulate some processing and set a response message
        response.success = True
        response.message = f"Mode {request.mode._mode} set successfully"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TestSetModeServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
