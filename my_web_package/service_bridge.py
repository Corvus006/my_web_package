import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from edu_robot.srv import SetMode  # Import the SetMode service

class BoolService(Node):
    def __init__(self):
        super().__init__('service_bridge')

        # Declare parameters with default values
        self.declare_parameter('enable_service_name', '/enable')
        self.declare_parameter('set_mode_service_name', '/set_mode')
        self.declare_parameter('mode_true', 2)
        self.declare_parameter('mode_false', 1)

        # Retrieve parameters
        enable_service_name = self.get_parameter('enable_service_name').get_parameter_value().string_value
        set_mode_service_name = self.get_parameter('set_mode_service_name').get_parameter_value().string_value
        self.mode_true = self.get_parameter('mode_true').get_parameter_value().integer_value
        self.mode_false = self.get_parameter('mode_false').get_parameter_value().integer_value

        # Create the 'set_bool_service' service
        self.srv = self.create_service(SetBool, enable_service_name, self.handle_set_bool)
        
        # Create a client for the SetMode service
        self.cli = self.create_client(SetMode, set_mode_service_name)
        
        self.get_logger().info(f'Service "{enable_service_name}" ready')
        self.get_logger().info(f'SetMode client connected to "{set_mode_service_name}"')

    def handle_set_bool(self, request, response):
        # This is where the request value is processed
        self.get_logger().info(f'Received request: {request.data}')
        
        # Create a request for the SetMode service
        mode_request = SetMode.Request()
        mode_request.mode.mode = self.mode_true if request.data else self.mode_false

        # Wait for the SetMode service to become available
        if self.cli.wait_for_service(timeout_sec=5.0):
            # Call the service and process the response
            future = self.cli.call_async(mode_request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"SetMode service response: {future.result().message}")
            else:
                self.get_logger().error("Error calling SetMode service")
        else:
            self.get_logger().error("SetMode service not available")

        # Return the response for the SetBool service
        response.success = True if request.data else False
        response.message = f"Input was {'True' if request.data else 'False'}. Mode set to {mode_request.mode.mode}."
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BoolService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
