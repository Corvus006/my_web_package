#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from edu_robot.srv import SetMode  # Import der SetMode-Nachricht

class BoolService(Node):
    def __init__(self):
        super().__init__('boo')
        # Erstellen des Dienstes 'set_bool_service'
        self.srv = self.create_service(SetBool, 'eduard/enable', self.handle_set_bool)
        # Erstellen des Clients für den SetMode-Service
        self.cli = self.create_client(SetMode, 'eduard/set_mode')
        self.get_logger().info('Service "set_bool_service" bereit')

    def handle_set_bool(self, request, response):
        # Hier wird der Anfragewert verarbeitet
        self.get_logger().info(f'Anfrage erhalten: {request.data}')
        
        # Erstellen der Anfrage für SetMode
        mode_request = SetMode.Request()
        if request.data:
            mode_request.mode = 2  # Sende 2 bei True
        else:
            mode_request.mode = 1  # Sende 1 bei False
        
        # Warten, bis der Service verfügbar ist
        if self.cli.wait_for_service(timeout_sec=5.0):
            # Service aufrufen und die Antwort verarbeiten
            future = self.cli.call_async(mode_request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"SetMode Service Antwort: {future.result().message}")
            else:
                self.get_logger().error("Fehler beim Aufruf des SetMode-Service")
        else:
            self.get_logger().error("SetMode-Service nicht verfügbar")

        # Antwort für SetBool zurückgeben
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
