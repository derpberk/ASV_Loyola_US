import rclpy
from rclpy.node import Node
from .submodulos.KMLMissionGeneration import KMLMissionGenerator

from geometry_msgs.msg import Point
from asv_interfaces.srv import Newwaypoint
from mavros_msgs.srv import CommandBool


#TODO: Everything

class Coordinator_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('mission_filename', '1')
        self.mission_filename = self.get_parameter('mission_filename').get_parameter_value().string_value
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        self.samplepoint_service = self.create_service(Newwaypoint, 'new_samplepoint', self.new_samplepoint_callback)
        self.mqtt_send_info = self.create_client(CommandBool, 'MQTT_send_info')
        self.arm_vehicle = self.create_client(CommandBool, 'arm_vehicle')
        self.collect_sample = self.create_client(CommandBool, 'get_water_module_sample')

    def declare_topics(self):
        dummy=0


    def __init__(self):
        #start the node
        super().__init__('Dronekit_node')

        #declare parameter of drone IP
        self.parameters()

        #declarations
        self.samplepoints=[] #list of waypoints to follow
        #declare the services
        self.declare_services()

    def call_service(self, client,  msg):
        # TODO: raise error to avoid infinite wait if service is not up, after all means a module is not active
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    return response
                break

    def main(args=None):
        # init ROS2
        rclpy.init(args=args)
        # start a class that servers the services
        coordinator_node = Coordinator_node()
        # loop the services
        rclpy.spin(coordinator_node)
        # after close connection shut down ROS2
        rclpy.shutdown()

    if __name__ == '__main__':
        main()