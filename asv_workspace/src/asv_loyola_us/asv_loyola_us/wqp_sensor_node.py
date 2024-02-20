import rclpy
from rclpy.node import Node
import time
import serial
from asv_interfaces.msg import SensorMsg
from sensor_msgs.msg import NavSatFix
from math import sin, cos
from datetime import datetime
import re

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy




class WQP_Sensor_module(Node):

    #his functions defines and assigns value to the
    def parameters(self):


        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

        self.declare_parameter('wqp_sensor_USB_string', "/dev/SENSOR")
        self.USB_string = self.get_parameter('wqp_sensor_USB_string').get_parameter_value().string_value

        self.declare_parameter('wqp_sensor_baudrate', 115200)
        self.baudrate = self.get_parameter('wqp_sensor_baudrate').get_parameter_value().integer_value

        self.declare_parameter('wqp_sensor_timeout', 10.0)
        self.timeout = self.get_parameter('wqp_sensor_timeout').get_parameter_value().double_value

        self.declare_parameter('wqp_sensor_measurement_frequency', 1.0)
        self.measurement_frequency = self.get_parameter('wqp_sensor_measurement_frequency').get_parameter_value().double_value
    
    def declare_topics(self):
        
        qos_profile_BEF = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

         # Create a subscription to the position of the ASV
        self.asv_position_subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)

        # Create a publisher for the sensor measurements
        self.sensor_publisher = self.create_publisher(SensorMsg, '/wqp_measurements', qos_profile_BEF)
        self.sensor_publisher_timer = self.create_timer(self.measurement_frequency, self.sensor_publish)
    

    def asv_position_callback(self, msg):
        # Save the position of the ASV
        self.position[0] = msg.latitude
        self.position[1] = msg.longitude

    def generate_fake_data(self):
        # Generate fake data

        sensor_measurement = SensorMsg()

        sensor_measurement.success = True
        sensor_measurement.conductivity = 0.0
        sensor_measurement.temperature_ct = 0.0
        sensor_measurement.turbidity = 0.0
        sensor_measurement.ph = 0.0
        sensor_measurement.vbat = 0.0
        sensor_measurement.lat = self.position[0]
        sensor_measurement.lon = self.position[1]
        sensor_measurement.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

        return sensor_measurement


    def __init__(self):

        #start the node
        super().__init__('wqp_sensor_node')
        
        # declare parameter of drone IP
        self.parameters()
        self.declare_topics()

        self.sensor_msg = SensorMsg()

        self.position = [0.0, 0.0]

        self.pattern = r'data=([^,]+),([^,\]]+)'

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
        
    def read_sensor(self):
        
        # Send the sensor a command to take a sample, and read the response
        read_trials = 0
        read_ok = False

        # First, check if the sensor is connected
        if self.serial.is_open:

            # Wait for the response 100ms
            read_data = ""
            time.sleep(0.1)

            # Try 5 times 
            while(read_trials < 5 or not read_ok):
                    
                    # Flush the input buffer
                    self.serial.read_all()
                    
                    # Send the command to the sensor
                    self.serial.write(bytes("mscan\n",'ascii'))

                    # Wait for the response
                    time.sleep(0.1)

                    # Increment the number of trials
                    read_trials += 1
                    
                    # Read the incoming data byte by byte
                    while(self.serial.in_waiting > 0):

                        new_incoming_data = self.serial.read() # Read one byte

                        try:
                            decoded_data = new_incoming_data.decode()
                            new_character = str(decoded_data)
                        except:
                            self.get_logger().debug(f"Cannot decode incomming byte!")
                            continue
                            
                        # Append the new character to the read data
                        read_data += new_character
    
                        if len(read_data) > 0 and '}' in read_data:
                            # We have read the whole message
                            read_ok = True
                            self.get_logger().debug(f"A message has been read from the sensor: {read_data}")
                            break
            

            if read_ok:
                return read_data
            elif read_trials >= 5:
                self.get_logger().info(f"Too many trials to read the sensor!")
                return None
            else:
                return None


    def status_suscriber_callback(self, msg):
        self.status=msg

    def destroy_usb(self):
        if self.serial.open:
            self.serial.close

    def reconnect_sensor(self):

        # Close the serial port
        if self.serial.is_open:
            self.serial.close() # This is important
        
        del self.serial # This is important

        # Try to reconnect
        connection_trials = 0

        while connection_trials < 10:

            # Try sensor connection
            self.serial = serial.Serial(self.USB_string, self.baudrate, timeout=10)
            
            if self.serial.open:
                self.get_logger().info(f"Sensor connected!")
                return True
            else:
                self.get_logger().info(f"Sensor not connected! Trial: {connection_trials}")
                connection_trials += 1
                time.sleep(0.1)

            if connection_trials > 10:
                self.get_logger().info(f"Failed to connect to Sensor")
                return False
    

    def sensor_publish(self):

        if not self.DEBUG:

            data = self.read_sensor()

            # Check if the data is not empty
            if data is None:

                self.get_logger().info(f"Sensor data is empty!")
                # Try to reconnect to the sensor
                reconnect_response = self.reconnect_sensor()

                # If the reconnection is successful, try to read the sensor again
                if reconnect_response:
                    data = self.read_sensor()
                else:
                    self.get_logger().info(f"Failed to reconnect to the sensor!")
                    return

            # Now we have the data, we can parse it
                
            self.get_logger().info(data)

            self.sensor_msg.success=True
            
            # Find all matches of the pattern in the input string
            matches = re.findall(self.pattern, data)
            
            if len(matches) == 0:
                self.get_logger().info(f"No matches found in the sensor data")
                return

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

            self.sensor_msg.vbat = (62.5*(self.vbat)-425)
            self.sensor_msg.lat = self.position[0]
            self.sensor_msg.lon = self.position[1]
            self.sensor_msg.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")  
         
        else:
            
            # We are simulating the sensor
            self.sensor_msg = self.generate_fake_data()
        

        # Send the message
        self.sensor_publisher.publish(self.sensor_msg)
            

def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    
    sensor_module = WQP_Sensor_module()
    rclpy.spin(sensor_module)

    # Destroy the node explicitly
    sensor_module.destroy_usb()
    rclpy.shutdown()





if __name__ == '__main__':
    main()

