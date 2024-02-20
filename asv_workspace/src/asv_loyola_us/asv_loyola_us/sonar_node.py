
import sys
import rclpy
from rclpy.node import Node
import serial
from brping import Ping1D
from asv_interfaces.msg import SonarMsg
from sensor_msgs.msg import NavSatFix

import time
from datetime import datetime
import random
from math import exp, sin, cos

class Sonar_node(Node):

    def parameters(self):

        self.declare_parameter('sonar_USB_string', '/dev/SONAR')
        self.sonar_USB_string = self.get_parameter('sonar_USB_string').get_parameter_value().string_value

        self.declare_parameter('sonar_baudrate', 115200)
        self.baudrate = self.get_parameter('sonar_baudrate').get_parameter_value().integer_value

        self.declare_parameter('debug', True)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

        self.declare_parameter('sonar_measurement_frequency', 1.0)
        self.sonar_measurement_frequency = self.get_parameter('sonar_measurement_frequency').get_parameter_value().float_value
        

    def declare_topics(self):

        qos_profile_BEF = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create a publisher for the sonar measurements
        self.sonar_publisher = self.create_publisher(SonarMsg, '/sonar_measurements', qos_profile_BEF)
        self.sonar_publisher_timer = self.create_timer(timer_period, self.sonar_publish)
       
        # Create a subscription to the position of the ASV
        self.asv_position_subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)

    def asv_position_callback(self, msg):

        self.position[0] = msg.latitude
        self.position[1] = msg.longitude

    def connect_to_sonar(self):

        connection_trials = 0
        
        while True:
            # Try sonar connection
            self.ping_device = Ping1D()
            self.ping_device.connect_serial(self.sonar_device_name, self.baudrate)

            if self.ping_device.get_ping_enable:
                self.get_logger().info(f"Sonar connected!")
                return True
                break
            else:
                self.get_logger().info(f"Sonar not connected! Trial: {connection_trials}")
                connection_trials += 1
                time.sleep(1)

            if connection_trials > 10:
                self.get_logger().info(f"Failed to connect to Sonar")
                break
        
        return False


    def __init__(self):
        super().__init__("sonar_service")

        self.parameters()

        self.declare_topics()

        self.sonar_msg = SonarMsg()

        self.position = [0.0, 0.0]

        self.number_of_bad_readings = 0
    

        if self.DEBUG:
            self.get_logger().info(f"Simulating sonar measurements")

        if  not self.DEBUG:

            if self.connect_to_sonar():
                self.get_logger().info(f"Sonar connected")
            else:
                self.get_logger().info(f"Sonar not connected")
                self.free_resources()
                sys.exit(1)
            

    def destroy_usb(self):
        if self.ping_device:
            self.ping_device.close()

    def generate_fake_data(self):

        displaced_lat = self.position[0] - 37.418691117644244
        displaced_long = self.position[1] - 6.001191255201682
        interval_lat = abs(37.418716586727506 - 37.418691117644244)
        interval_long = abs(6.001191255201682 - 6.0007770870275252)
        self.sonar_msg.success = True
        self.sonar_msg.distance = 30 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
        self.sonar_msg.confidence = 1.0
        self.sonar_msg.lat = self.position[0]
        self.sonar_msg.lon = self.position[1]
        self.sonar_msg.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

        return self.sonar_msg

    def sonar_publish(self):

        if not self.DEBUG:

            if self.ping_device and self.ping_device.get_ping_enable: #Si estamos concetados realizamos el checkeo

                data = self.ping_device.get_distance()

                if data is None:

                    self.sonar_msg.distance = -1.0
                    self.sonar_msg.confidence = -1.0
                    self.sonar_msg.success = False
                    self.get_logger().info(f"Bad reading from sonar - None Received")
                    self.number_of_bad_readings += 1

                    if self.number_of_bad_readings > 10:
                        self.get_logger().info("Sonar not working - Trying to reconnect")
                        self.ping_device.close()
                        self.connect_to_sonar()
                else:
                    # Hooraay! We have a good reading

                    self.number_of_bad_readings = 0
                    self.sonar_msg.distance = float(data["distance"])
                    self.sonar_msg.confidence = float(data["confidence"])
                    self.sonar_msg.success = True

            self.sonar_msg.lat = self.position[0]
            self.sonar_msg.lon = self.position[1]
            self.sonar_msg.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")


        else:
            # We are simulating the sonar
            self.sonar_msg = self.generate_fake_data()
        
        # Send the message
        self.sonar_publisher.publish(self.sonar_msg)
    

    def free_resources(self):
        """ Free the serial port resources """
        self.destroy_usb()

def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    
    sonar_node = Sonar_node()
    rclpy.spin(sonar_node)

    # Destroy the node explicitly
    sonar_node.free_resources()
    rclpy.shutdown()


if __name__ == '__main__':
    main()