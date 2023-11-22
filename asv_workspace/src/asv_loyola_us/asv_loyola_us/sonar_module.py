
import sys
import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
from brping import Ping1D
from asv_interfaces.msg import Sonar, Nodeupdate, Status
from asv_interfaces.srv import SonarService
import threading
import time
from datetime import datetime
import traceback
from rclpy.executors import MultiThreadedExecutor
import random
from math import exp, sin, cos

class Sonar_node(Node):

    def parameters(self):

        self.declare_parameter('sonar_connection_type', "USB")
        self.declare_parameter('sonar_device_name', '/dev/SONAR')
        self.sonar_device_name = self.get_parameter('sonar_device_name').get_parameter_value().string_value
        self.sonar_connection_type = self.get_parameter('sonar_connection_type').get_parameter_value().string_value
        self.declare_parameter('debug', True)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value
        

    def declare_topics(self):

        timer_period = 1.0  # seconds
        self.sonar_publisher = self.create_publisher(Sonar, 'sonar', 10)
        self.sonar_publisher_timer = self.create_timer(timer_period, self.sonar_publish)
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)

    def declare_services(self):
        # Create a service host for the sonar
        self.sonar_service = self.create_service(SonarService, 'sonar_measurement_service', self.sonar_service_callback)
    
    def __init__(self):
        super().__init__("sonar_service")

        self.parameters()

        self.declare_topics()
        self.declare_services()
        self.status = Status()
        self.sonar_msg = Sonar()
        

        self.data0 = None
        self.data1 = None

        if self.DEBUG:
            self.get_logger().info(f"Simulating sonar measurements")

        if not self.DEBUG:
            connection_trials = 0
            while True:
                # Try sonar connection
                self.ping_device = Ping1D()
                self.ping_device.connect_serial(self.sonar_device_name, 115200)
                if self.ping_device.get_ping_enable:
                    self.get_logger().info(f"Sonar connected!")
                    break
                else:
                    self.get_logger().info(f"Sonar not connected! Trial: {connection_trials}")
                    connection_trials += 1
                    time.sleep(1)

                if connection_trials > 10:
                    self.get_logger().info(f"Failed to connect to Sonar")
                    break
            
     
# funcion para la comprobacion del sonar , se llama dentro de la clase Ping dada por el sonar a la funcion get_ping_enable que muestra 
# si esta activo la señal de salida acustica, que es la que se encarga de realizar la mediciones
    def sonar_service_callback(self, request, response):

        if not self.DEBUG:

            if self.ping_device: # Si estamos conectados realizamos el checkeo
                if self.ping_device.get_ping_enable: #comprobamos si esta funcionando el sonar 
                    
                    # Get distance data
                    data = self.ping_device.get_distance()
                    if data is None:
                        self.sonar_msg.distance = -1.0
                        self.sonar_msg.confidence = -1.0
                        self.sonar_msg.success = False
                        self.get_logger().info(f"Bad reading from sonar - None Received")
                    else:
                        self.sonar_msg.distance = float(data["distance"])
                        self.sonar_msg.confidence = float(data["confidence"])
                        self.sonar_msg.success = True

                else:                               #Si no esta funcionando, confirmamos de que no lo esta
                    response.success = False
                    self.get_logger().info("Sonar not working")
            
        else:
            # Simulamos la respuesta del sonar
            displaced_lat = self.status.lat - 37.418691117644244
            displaced_long = self.status.lon - 6.001191255201682
            interval_lat = abs(37.418716586727506 - 37.418691117644244)
            interval_long = abs(6.001191255201682 - 6.0007770870275252)
            response.sonar.success = True
            response.sonar.distance = 30 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
            self.get_logger().info(f"Sonar distance: {response.sonar.distance}")
            response.sonar.confidence = 1.0

        # This is common to both cases
        response.sonar.lat = self.status.lat
        response.sonar.lon = self.status.lon
        response.sonar.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

        return response
    
    def status_suscriber_callback(self, msg):
        self.status = msg

    def sonar_publish(self):

        if not self.DEBUG:

            if self.ping_device: #Si estamos concetados realizamos el checkeo
                if self.ping_device.get_ping_enable: #comprobamos si esta funcionando el sonar 
                    data = self.ping_device.get_distance()

                    if data is None:
                        self.sonar_msg.distance = -1.0
                        self.sonar_msg.confidence = -1.0
                        self.sonar_msg.success = False
                        self.get_logger().info(f"Bad reading from sonar - None Received")
                    else:
                        self.sonar_msg.distance = float(data["distance"])
                        self.sonar_msg.confidence = float(data["confidence"])
                        self.sonar_msg.success = True

                else:
                    self.get_logger().info("Sonar not working")
                    self.sonar_msg.distance = -1.0
                    self.sonar_msg.confidence = -1.0
                    self.sonar_msg.success = False

        else:
            
            displaced_lat = self.status.lat - 37.418691117644244
            displaced_long = self.status.lon - 6.001191255201682
            interval_lat = abs(37.418716586727506 - 37.418691117644244)
            interval_long = abs(6.001191255201682 - 6.0007770870275252)
            self.sonar_msg.success = True
            self.sonar_msg.distance = 30 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
            self.sonar_msg.confidence = 1.0
        
        # This is common to both cases
        self.sonar_msg.lat = self.status.lat
        self.sonar_msg.lon = self.status.lon
        self.sonar_msg.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            
        # Send the message
        self.sonar_publisher.publish(self.sonar_msg)
    

def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    try:
        #start a class that servers the services
        sonar_node= Sonar_node()
        
        #loop the node
        rclpy.spin(sonar_node, executor=MultiThreadedExecutor())

        sonar_node.destroy_node()

    except:
        """
        There has been an error with the program, so we will send the error log to the watchdog
        """
        x = rclpy.create_node('sonar_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "sonar_node" #our identity
        msg.message = traceback.format_exc() #the error
        #to be sure the message reaches, we must wait till watchdog is listening (publisher needs time to start up)
        #TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
        # este topic está oculto en echo al usar _
        while publisher.get_subscription_count() == 0: #while no one is listening
            time.sleep(0.01) #we wait
        publisher.publish(msg) #we send the message
        x.get_logger().fatal(msg.message)
        x.destroy_node() #we destroy node and finish
    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
