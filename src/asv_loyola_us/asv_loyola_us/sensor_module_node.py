import rclpy
from rclpy.node import Node
#from .submodulos.SensorModule import WaterQualityModule
import json
import time
import random
from asv_interfaces.srv import Takesample

#TODO: EVERYTHING

class Sensor_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('database_name', 'LOCAL_DATABASE.db')
        self.database_name = self.get_parameter('database_name').get_parameter_value().string_value
        self.declare_parameter('USB_string', "/dev/ttyACM0")
        self.USB_string = self.get_parameter('USB_string').get_parameter_value().string_value
        self.declare_parameter('timeout', 6)
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.declare_parameter('baudrate', 115200)
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.declare_parameter('pump_parameters', None)
        self.pump_parameters = self.get_parameter('pump_parameters').get_parameter_value()

    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        self.get_sample = self.create_service(Takesample, 'get_sample', self.get_sample_callback)

    def declare_topics(self):
        dummy=0


    def __init__(self):
        #start the node
        super().__init__('Dronekit_node')

        #declare parameter of drone IP
        self.parameters()


        #TODO: Study how nested parameters work, it should be passed as JSON
        #pump_parameters = json.loads(config['WATER']['pump_parameters'])

        """modulo_de_sensores = WaterQualityModule(database_name=self.database_name,
                                                USB_string=self.USB_string,
                                                timeout=self.USB_string,
                                                baudrate=self.baudrate,
                                                pump_parameters=self.pump_parameters)"""
        #declare the services
        self.declare_services()

    def get_sample_callback(self, request, response):
        self.get_logger().info(f"Sample requested")
        if request.debug:
            self.get_logger().info(f"The sample will return a debug value")
            time.sleep(6.0)
            response.ph=random.random()*8.0
            response.turbidity=random.random()*100.0
            response.algae=random.random()*100.0
            response.salinity=random.random()*4.0
            response.o2=random.random()*98.0
        return response


def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    #start a class that servers the services
    sensor_node = Sensor_node()
    #loop the services
    rclpy.spin(sensor_node)
    #after close connection shut down ROS2
    rclpy.shutdown()




if __name__ == '__main__':
    main()


