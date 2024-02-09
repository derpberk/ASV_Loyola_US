import rclpy
from rclpy.node import Node
#from .submodulos.SensorModule import WaterQualityModule
import json
import time
import serial
import random
from asv_interfaces.srv import Takesample, SensorService
from asv_interfaces.msg import Status, Sensor, Nodeupdate
from asv_interfaces.action import SensorSample
from math import exp, sin, cos, tan, tanh,sinh,cosh

from datetime import datetime
from .submodulos.call_service import call_service #to call services
import traceback
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import random
import re

class Sensor_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value
        self.declare_parameter('USB_string', "/dev/SENSOR")
        self.USB_string = self.get_parameter('USB_string').get_parameter_value().string_value
        self.declare_parameter('baudrate', 115200)
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
   
    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        self.sensor_service = self.create_service(SensorService, 'sensor_measurement_service', self.sensor_service_callback)
        

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        timer_period = 2.0  # seconds
        self.sensor_publisher = self.create_publisher(Sensor, 'sensor', 10)
        self.sensor_publisher_timer = self.create_timer(timer_period, self.sensor_publish)
    


    def __init__(self):
        #start the node
        super().__init__('sensor_module')
        
        #declare parameter of drone IP
        self.parameters()

        self.declare_topics()
        self.sensor_msg = Sensor()
        self.status=Status()
        if self.DEBUG:
            self.get_logger().warning("Debug mode enabled")

        if  not self.DEBUG:
            connection_trials = 0
            while True:
                # Try sensor connection
                self.serial = serial.Serial(self.USB_string,self.baudrate, timeout=10)
                
                if self.serial.open:
                    self.get_logger().info(f"Sensor connected!")
                    break
                else:
                    self.get_logger().info(f"Sensor not connected! Trial: {connection_trials}")
                    connection_trials += 1
                    time.sleep(1)

                if connection_trials > 10:
                    self.get_logger().info(f"Failed to connect to Sensor")
                    break
        

    def sensor_service_callback(self,response):
        if  not self.DEBUG:
            
            self.serial.read_all()

            self.serial.write(bytes("mscan\n",'ascii'))
            
            read_data = ""
            while(True):

                if(self.serial.in_waiting > 0):
                    new_incoming_data = self.serial.read()
                    
                    try:
                        decoded_data = new_incoming_data.decode()
                        new_character = str(decoded_data)
                    except:
                        response.get_logger().debug(f"Cannot decode incomming byte!")
                        continue

                    
                    read_data += new_character

                if len(read_data) > 0:
                    if '}' in read_data:
                        break
            
            data = read_data
            self.get_logger().info(data)
            self.sensor_msg.success=True
            pattern = r'data=([^,]+),([^,\]]+)'
            matches = re.findall(pattern, data)
            for match in matches:
                sensor_str = match[0]
                sensor_val = match[1]
                if sensor_str == "Cond":
                    self.get_logger().debug(f"Found Conductivity {sensor_val}")
                    response.sensor.conductivity = float(sensor_val)
                if sensor_str == "TempCT":
                    response.get_logger().debug(f"Found Temperature from an CT.X2 sensor {sensor_val}")
                    self.sensor.temperature_ct = float(sensor_val)
                if sensor_str == "Turbidity":
                    self.get_logger().debug(f"Found Turdibity {sensor_val}")
                    response.sensor.turbidity = float(sensor_val)
                if sensor_str == "pH":
                    self.get_logger().debug(f"Found pH value {sensor_val}")
                    response.sensor.ph = float(sensor_val)
                if sensor_str == "vbat":
                    self.get_logger().debug(f"Found pH value {sensor_val}")
                    self.vbat = float(sensor_val)
            else:                               #Si no esta funcionando, confirmamos de que no lo esta
                    response.success = False
                    self.get_logger().info("Sensor not working")            
        else:
            # Simulamos la respuesta del sensor
            displaced_lat = self.status.lat - 37.418691117644244
            displaced_long = self.status.lon - 6.001191255201682
            interval_lat = abs(37.418716586727506 - 37.418691117644244)
            interval_long = abs(6.001191255201682 - 6.0007770870275252)
            response.sensor.success = True
            response.sensor.conductivity = 30 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
            response.sensor.temperature_ct = 20 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
            response.sensor.turbidity = 10 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
            response.sensor.ph = 40 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 

            self.get_logger().info(f"Conductivity debug value: {response.sensor.coductivity}")
            self.get_logger().info(f"Temperature debug value: {response.sensor.temperature}")
            self.get_logger().info(f"Turbidity debug value: {response.sensor.turbidity}")
            self.get_logger().info(f"PH debug value: {response.sensor.ph}")
            self.get_logger().info(f"Voltage battery debug value: {response.sensor.vbat}")
        
        response.sensor.vbat = (62.5*(self.vbat)-425)
        response.sensor.lat = self.status.lat
        response.sensor.lon = self.status.lon
        response.sensor.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            
        return response




    def status_suscriber_callback(self, msg):
        self.status=msg

    def sensor_publish(self):
        if not self.DEBUG:

            self.serial.read_all()

            self.serial.write(bytes("mscan\n",'ascii'))
            
            read_data = ""
            
            while(True):

                if(self.serial.in_waiting > 0):
                    new_incoming_data = self.serial.read()
                    try:
                        decoded_data = new_incoming_data.decode()
                        new_character = str(decoded_data)
                    except:
                        self.get_logger().debug(f"Cannot decode incomming byte!")
                        continue
                        
                    read_data += new_character

                if len(read_data) > 0:
                    if '}' in read_data:
                        break
            
            data = read_data
            self.get_logger().info(data)
            self.sensor_msg.success=True
            
            pattern = r'data=([^,]+),([^,\]]+)'
            # Find all matches of the pattern in the input string
            matches = re.findall(pattern, data)
            for match in matches:
                sensor_str = match[0]
                sensor_val = match[1]

                if sensor_str == "Cond":
                    #self.get_logger().info(f"Found Conductivity {sensor_val}")
                    self.sensor_msg.conductivity = float(sensor_val)
                if sensor_str == "TempCT":
                    #self.get_logger().info(f"Found Temperature from an CT.X2 sensor {sensor_val}")
                    self.sensor_msg.temperature_ct = float(sensor_val)
                if sensor_str == "Turbidity":
                    #self.get_logger().info(f"Found Turdibity {sensor_val}")
                    self.sensor_msg.turbidity = float(sensor_val)
                if sensor_str == "pH":
                    #self.get_logger().info(f"Found pH value {sensor_val}")
                    self.sensor_msg.ph = float(sensor_val)
                if sensor_str == "vbat":
                    #self.get_logger().info(f"Found battery value {sensor_val}")
                    self.vbat = float(sensor_val)
         
        else:
            # Simulamos la respuesta del sensor
            displaced_lat = self.status.lat - 37.418691117644244
            displaced_long = self.status.lon - 6.001191255201682
            interval_lat = abs(37.418716586727506 - 37.418691117644244)
            interval_long = abs(6.001191255201682 - 6.0007770870275252)
            self.sensor_msg.success = True
            self.sensor_msg.conductivity = 30 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
            self.sensor_msg.temperature_ct = 20 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
            self.sensor_msg.turbidity = 10 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
            self.sensor_msg.ph = 40 * cos(2*3.141592 * displaced_lat / interval_lat) + 30 * sin(2*3.141592 * displaced_long / interval_long) 
            self.vbat=0
            self.get_logger().info(f"Conductivity debug value: {self.sensor_msg.conductivity}")
            self.get_logger().info(f"Temperature debug value: {self.sensor_msg.temperature_ct}")
            self.get_logger().info(f"Turbidity debug value: {self.sensor_msg.turbidity}")
            self.get_logger().info(f"PH debug value: {self.sensor_msg.ph}")
            self.get_logger().info(f"Voltage battery debug value: {self.sensor_msg.vbat}")
        
        self.sensor_msg.vbat = (62.5*(self.vbat)-425)
        self.sensor_msg.lat = self.status.lat
        self.sensor_msg.lon = self.status.lon
        self.sensor_msg.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        
         # Send the message
        self.sensor_publisher.publish(self.sensor_msg)
            
        



def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    try:
        # start a class that servers the services
        sensor_node = Sensor_node()
        # loop the services
        rclpy.spin(sensor_node, executor=MultiThreadedExecutor())
        sensor_node.destroy_node()
    except:
        """
        There has been an error with the program, so we will send the error log to the watchdog
        """
        x = rclpy.create_node('sensor_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "sensor_node" #our identity
        msg.message = traceback.format_exc() #the error
        #to be sure the message reaches, we must wait till watchdog is listening (publisher needs time to start up)
        #TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
        # este topic está oculto en echo al usar _
        while publisher.get_subscription_count() == 0: #while no one is listening
            time.sleep(0.01) #we wait
        publisher.publish(msg) #we send the message
        x.destroy_node() #we destroy node and finish
    #after close connection shut down ROS2
    rclpy.shutdown()




if __name__ == '__main__':
    main()


