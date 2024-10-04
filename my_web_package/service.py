#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class BoolService(Node):
    def __init__(self):
        super().__init__('bool_service')
        # Erstellen des Dienstes 'set_bool_service'
        self.srv = self.create_service(SetBool, 'eduard/enable', self.handle_set_bool)
        self.get_logger().info('Service "set_bool_service" bereit')

    def handle_set_bool(self, request, response):
        # Hier wird der Anfragewert verarbeitet
        self.get_logger().info(f'Anfrage erhalten: {request.data}')
        
        if request.data:
            response.success = True
            response.message = "Input was True."
        else:
            response.success = False
            response.message = "Input was False."
        
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
