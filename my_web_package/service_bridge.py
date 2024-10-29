import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from edu_robot.srv import SetMode  # Import the SetMode service

class BoolService(Node):
    def __init__(self):
        super().__init__('service_bridge')
        # Create the 'set_bool_service' service
        self.srv = self.create_service(SetBool, '/wgg/enable', self.handle_set_bool)
        # Create a client for the SetMode service
        self.cli = self.create_client(SetMode, '/wgg/set_mode')
        self.get_logger().info('Service "set_bool_service" ready')

    def handle_set_bool(self, request, response):
        # This is where the request value is processed
        self.get_logger().info(f'Received request: {request.data}')
        
        # Create a request for the SetMode service
        mode_request = SetMode.Request()
        if request.data:
            mode_request.mode._mode = 2  # Send 2 if True
        else:
            mode_request.mode._mode = 1  # Send 1 if False
        # { mode: { mode: N } }
        
        # Wait for the SetMode service to become available
        if self.cli.wait_for_service(timeout_sec=5.0):
            # Call the service and process the response
            future = self.cli.call_async(mode_request)
            if future.result() is not None:
                self.get_logger().info(f"SetMode service response: {future.result().message}")
            else:
                self.get_logger().error("Error calling SetMode service")
        else:
            self.get_logger().error("SetMode service not available")

        # Return the response for the SetBool service
        if request.data:
            response.success = True
            response.message = "Input was True. Mode set to 2."
        else:
            response.success = False
            response.message = "Input was False. Mode set to 1."
        
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
