import rclpy
from rclpy.node import Node
from edu_robot.srv import SetMode  # Import the SetMode service

class TestSetModeServer(Node):
    def __init__(self):
        super().__init__('test_set_mode_server')
        
        # Declare a parameter for the SetMode service name
        self.declare_parameter('set_mode_service_name', '/set_mode')
        
        # Retrieve the parameter
        set_mode_service_name = self.get_parameter('set_mode_service_name').get_parameter_value().string_value

        # Create the SetMode service using the parameterized name
        self.srv = self.create_service(SetMode, set_mode_service_name, self.handle_set_mode)
        self.get_logger().info(f'Test SetMode server ready on "{set_mode_service_name}"')

    def handle_set_mode(self, request, response):
        # Process the received request and provide a response
        self.get_logger().info(f'Received mode request: {request.mode.mode}')
        
        # Simulate some processing and set a response message
        response.success = True
        response.message = f"Mode {request.mode.mode} set successfully"
        
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
