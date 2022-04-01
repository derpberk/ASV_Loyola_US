import rclpy
from rclpy.node import Node
#from .submodulos.SensorModule import WaterQualityModule
import json
import time
import serial
import random
from asv_interfaces.srv import Takesample, SensorParams
from asv_interfaces.msg import Status, Sensor, Nodeupdate
import Jetson.GPIO as GPIO
from datetime import datetime
from .submodulos.call_service import call_service #to call services
import traceback

class Sensor_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value
        self.declare_parameter('pump', False)
        self.pump_installed = self.get_parameter('pump').get_parameter_value().bool_value
        self.declare_parameter('pump_channel', 7)
        self.pump_channel = self.get_parameter('pump_channel').get_parameter_value().integer_value
        self.declare_parameter('num_samples', 4)
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        self.declare_parameter('USB_string', "/dev/ttyUSB0")
        self.USB_string = self.get_parameter('USB_string').get_parameter_value().string_value
        self.declare_parameter('baudrate', 115200)
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.declare_parameter('timeout', 6)
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.declare_parameter('time_between_samples', 0.5)
        self.time_between_samples = self.get_parameter('time_between_samples').get_parameter_value().double_value


    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        self.get_sample = self.create_service(Takesample, 'get_sample', self.get_sample_callback)
        self.update_parameters_server = self.create_service(SensorParams, 'Sensor_params', self.update_parameters_callback)
        self.get_sample = self.create_service(Takesample, 'get_sample', self.get_sample_callback)

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.sensor_publisher = self.create_publisher(Sensor, 'sensors', 10)


    def __init__(self):
        #start the node
        super().__init__('sensor_node')

        #declare parameter of drone IP
        self.parameters()

        self.declare_topics()

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pump_channel, GPIO.OUT)
        self.sensor_data = Sensor()
        try:
            if self.DEBUG == False:
                self.serial = serial.Serial(self.USB_string, self.baudrate, timeout=self.timeout)
            self.declare_services()
        except ConnectionRefusedError:
            self.get_logger().error(f"Failed to connect to SmartWater module, connection refused")
            self.get_logger().fatal("Sensor module is dead")
            self.destroy_node()
        except OSError:
            self.get_logger().error(f"Failed to connect to SmartWater module, Is it off or disconnected?")
            self.get_logger().fatal("Sensor module is dead")
            self.destroy_node()
        except TimeoutError:
            self.get_logger().error(f"Failed to connect to SmartWater module, timeout")
            self.get_logger().fatal("Sensor module is dead")
            self.destroy_node()
        except:
            error = traceback.format_exc()
            self.get_logger().error(f"Connection to SmartWater could not be made, unknown error:\n {error}")
            self.get_logger().fatal("Sensor module is dead")
            self.destroy_node()
        self.get_logger().info("sensor node initialized")
        #declare the services

    def get_sample_callback(self, request, response):
        self.get_logger().info(f"Sample requested")
        if request.debug:
            self.get_logger().info(f"The sample will return a debug value")
            time.sleep(6.0)
            response.date=str(datetime.now())
            response.ph=random.random()*8.0
            response.ph_volt=random.random()*8.0
            response.ph_temp=15.0+random.random()*10.0
            response.temperature=15.0+random.random()*10.0
            response.salinity=random.random()*4.0
            response.o2_percentage=random.random()*98.0
            response.conductivity = 20 + random.random()*10
            response.conductivity_res = 4+  random.random()*10
            response.oxidation_reduction_potential =  4+  random.random()*10
        else:
            if self.pump_installed:
                GPIO.output(self.pump_channel, GPIO.HIGH)  # start filling the tank
                self.get_logger().info("activando bomba")
                time.sleep(10.0)                          #wait 10 seconds
            else:
                time.sleep(1.0)
            for i in range(self.num_samples):
                self.read_sensor(i, self.num_samples)                        #read smart water
                time.sleep(self.time_between_samples)
            if self.pump_installed:
                GPIO.output(self.pump_channel, GPIO.LOW)  #stop filling the tank
                self.get_logger().info("deteniendo bomba")
            response.sensor.date=self.sensor_data.date       #return values
            response.sensor.smart_water_battery = self.sensor_data.smart_water_battery
            response.sensor.ph_volt = self.sensor_data.ph_volt
            response.sensor.ph_temp = self.sensor_data.ph_temp
            response.sensor.ph = self.sensor_data.ph
            response.sensor.salinity = self.sensor_data.salinity
            response.sensor.o2_percentage = self.sensor_data.o2_percentage
            response.sensor.temperature = self.sensor_data.temperature
            response.sensor.conductivity = self.sensor_data.conductivity
            response.sensor.conductivity_res = self.sensor_data.conductivity_res
            response.sensor.oxidation_reduction_potential = self.sensor_data.oxidation_reduction_potential
        return response

    def read_sensor(self, num, max_samples):
        self.get_logger().info(f"Taking sample {num+1} of {max_samples}")

        is_frame_ok = False  # While a frame hasnt been correctly read #
        #self.serial.reset_input_buffer()  # Erase the input buffer to start listening


        while not is_frame_ok:

            time.sleep(0.5)  # Polling time. Every 0.5 secs, check the buffer #

            if self.DEBUG == False and self.serial.inWaiting() < 27:  # If theGPIO.output(self.pump_channel, GPIO.HIGH) #start filling the tank frame has a lenght inferior to the minimum of the Header
                continue

            else:

                try:
                    if self.DEBUG:
                        bytes = "<=>#5C3F1CE819623CBF#SW3#7#BAT:98#WT:15.42#PH:-5.46#DO:8.0#COND:0.6#ORP:0.545#"
                    else:
                        bytes = self.serial.read_all()  # Read all the buffer 
                        bytes = bytes.decode('ascii', 'ignore')  # Convert to ASCII and ignore non readible characters

                    self.get_logger().debug(f"read: {bytes}")
                    frames = bytes.split('<=>')  # Frame separator

                    last_frame = frames[-1].split('#')[
                    :-1]  # Select the last frame, parse the fields (#) and discard the last value (EOF)
                    self.get_logger().info(f"sensor read: {last_frame}")

                    for field in last_frame:  # Iterate over the frame fields

                        data = field.split(':')
                        if len(data) < 2:
                            # This is not a data field #
                            pass
                        else:
                            # This is a data field #
                            sensor_str = data[0]
                            sensor_val = float(data[1])

                            if sensor_str == "SAMPLE_NUM":
                                self.get_logger().info(f"Found SAMPLE_NUM {sensor_val}")
                            if sensor_str == "BAT":
                                self.get_logger().debug(f"Found Battery {sensor_val}")
                                self.sensor_data.smart_water_battery = sensor_val
                            if sensor_str == "WT":
                                self.get_logger().debug(f"Found temperature {sensor_val}")
                                self.sensor_data.temperature = sensor_val
                            if sensor_str == "PH":
                                self.get_logger().debug(f"Found ph value {sensor_val}")
                                self.sensor_data.ph = sensor_val
                            if sensor_str == "DO":
                                self.get_logger().debug(f"Found Disolved Oxygen {sensor_val}")
                                self.sensor_data.o2 = sensor_val
                            if sensor_str == "COND":
                                self.get_logger().debug(f"Found Conductivity {sensor_val}")
                                self.sensor_data.conductivity = sensor_val
                            if sensor_str == "ORP":
                                self.get_logger().debug(f"Found Oxidation Reduction Potential {sensor_val}")
                                self.sensor_data.oxidation_reduction_potential = sensor_val

                    is_frame_ok = True
                    self.sensor_data.date = str(datetime.now())
                    self.sensor_publisher.publish(self.sensor_data) #send data to MQTT to store in server
                except Exception as E:

                    print("ERROR READING THE SENSOR. THIS IS NO GOOD!")
                    print("The cause of the exception: " + E)
                    #self.serial.reset_input_buffer()

    def status_suscriber_callback(self, msg):
        self.status=msg

    def update_parameters_callback(self, request, response):
        if request.read_only:
            response.pump_channel = self.pump_channel
            response.number_of_samples = self.num_samples
            response.time_between_samples = self.time_between_samples
            response.use_pump = self.pump_installed
            return response
        if request.use_pump != self.pump_installed:
            self.pump_installed=request.use_pump
            self.get_logger().info("pump installed" if request.use_pump else "pump deactivated")
        if self.pump_installed and (request.pump_channel != self.pump_channel):
            self.get_logger().info(f"pump channel changed to {request.pump_channel}")
            self.pump_channel = request.pump_channel
            GPIO.setup(self.pump_channel, GPIO.OUT) #TODO: consider deactivating old channel
        if self.num_samples != request.number_of_samples:
            self.num_samples = request.number_of_samples
            self.get_logger().info(f"number of samples changed to {request.number_of_samples}")
        if response.time_between_samples != self.time_between_samples:
            self.time_between_samples = response.time_between_samples
            self.get_logger().info(f"time between samples changed to {request.time_between_samples} s")
        response.pump_channel = request.pump_channel
        response.number_of_samples = request.number_of_samples
        response.time_between_samples = request.time_between_samples
        response.use_pump = request.use_pump
        return response

    def status_suscriber_callback(self, msg):
        self.status = msg


def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    try:
        # start a class that servers the services
        sensor_node = Sensor_node()
        # loop the services
        rclpy.spin(sensor_node)
        sensor_node.destroy_node()
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


