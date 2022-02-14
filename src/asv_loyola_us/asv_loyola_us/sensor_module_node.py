import rclpy
from rclpy.node import Node
#from .submodulos.SensorModule import WaterQualityModule
import json
import time
import serial
import random
from asv_interfaces.srv import Takesample
from asv_interfaces.msg import Status
import Jetson.GPIO as GPIO
from datetime import datetime

#TODO: EVERYTHING

class Sensor_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('database_name', 'LOCAL_DATABASE.db')
        self.database_name = self.get_parameter('database_name').get_parameter_value().string_value
        self.declare_parameter('USB_string', "/dev/ttyUSB0")
        self.USB_string = self.get_parameter('USB_string').get_parameter_value().string_value
        self.declare_parameter('pump_channel', 7)
        self.pump_channel = self.get_parameter('pump_channel').get_parameter_value().integer_value
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
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)


    def __init__(self):
        #start the node
        super().__init__('sensor_node')

        #declare parameter of drone IP
        self.parameters()

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pump_channel, GPIO.OUT)
        self.sensor_data = {}

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
        else
            GPIO.output(self.pump_channel, GPIO.HIGH)
            time.sleep(8.0)
            GPIO.output(self.pump_channel, GPIO.LOW)
            self.read_sensor()
            print("after test")
            print(self.sensor_data)


        return response

    def read_sensor(self):
        self.get_logger().info(f"Taking sample")

        self.read_frame()  # Read a frame from the buffer

        str_date = str(datetime.now())  # Leemos la fecha y la convertimos en string
        # Metemos la fecha en el diccionario de variables
        self.sensor_data['DATE'] = str_date

        #TODO: read position
        # Almacenamos la posicion
        self.sensor_data['LATITUD'] = self.status.lat
        self.sensor_data['LONGITUD'] = self.status.lon



    def read_frame(self):
        is_frame_ok = False  # While a frame hasnt correctly readed #
        self.serial.reset_input_buffer()  # Erase the input buffer to start listening

        while not is_frame_ok:

            time.sleep(0.5)  # Polling time. Every 0.5 secs, check the buffer #

            if self.serial.inWaiting() < 27:  # If the frame has a lenght inferior to the minimum of the Header
                continue

            else:

                try:
                    bytes = self.serial.read_all()  # Read all the buffer #

                    bytes = bytes.decode('ascii', 'ignore')  # Convert to ASCII and ignore non readible characters

                    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
                    print(bytes)
                    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

                    frames = bytes.split('<=>')  # Frame separator

                    last_frame = frames[-1].split('#')[
                                 :-1]  # Select the last frame, parse the fields (#) and discard the last value (EOF)
                    print(last_frame)

                    for field in last_frame:  # Iterate over the frame fields

                        data = field.split(':')
                        if len(data) < 2:
                            # This is not a data field #
                            pass
                        else:
                            # This is a data field #
                            sensor_str = data[0]
                            sensor_val = float(data[1])
                            self.sensor_data[sensor_str] = sensor_val

                    is_frame_ok = True

                except Exception as E:

                    print("ERROR READING THE SENSOR. THIS IS NO GOOD!")
                    print("The cause of the exception: " + E)
                    self.serial.reset_input_buffer()

    def status_suscriber_callback(self, msg):
        self.status=msg


def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    try:
        # start a class that servers the services
        sensor_node = Sensor_node()
        # loop the services
        rclpy.spin(sensor_node)
    except:
        """
        There has been an error with the program, so we will send the error log to the watchdog
        """
        GPIO.cleanup() #release GPIO
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


