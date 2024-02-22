import rclpy
from rclpy.node import Node
import time
import serial
from asv_interfaces.msg import Status, Sensor, Nodeupdate
from asv_interfaces.srv import SensorService
from math import sin, cos
from datetime import datetime
import traceback
from rclpy.executors import MultiThreadedExecutor
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
    
    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        timer_period = 2.0  # seconds
        self.sensor_publisher = self.create_publisher(Sensor, 'sensor', 10)
        self.sensor_publisher_timer = self.create_timer(timer_period, self.sensor_publish)
    
    
    def declare_services(self):
        # Create a service host for the sonar
        self.sonar_service = self.create_service(SensorService, 'sensor_measurement_service', self.sensor_service_callback)


    def __init__(self):
        #start the node
        super().__init__('sensor_module')
        
        #declare parameter of drone IP
        self.parameters()

        self.declare_topics()
        self.sensor_msg = Sensor()
        self.status=Status()
        self.pattern = r'data=([^,]+),([^,\]]+)'
        self.reconnection_tries = 0

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
        waiting_time = 0.0

        # First, check if the sensor is connected
        if self.serial.is_open:

            # Wait for the response 100ms
            read_data = "0"

            # Flush the input buffer
            self.serial.read_all()
            
            # Send the command to the sensor
            self.serial.write(bytes("mscan\n",'ascii'))

            # Wait until the serial port has data
            waiting_time = 0.0
            while(self.serial.in_waiting == 0 and waiting_time < 1.0):
                time.sleep(0.1)
                waiting_time += 0.1
                if waiting_time >= 1:
                    self.get_logger().info(f"Sensor not responding after sending mscan command for 1 second") 
                    return None
            
            
            t0 = time.time() # Initial time

            # Read the incoming data byte by byte
            while(read_data[-1] != '}'):

                new_incoming_data = self.serial.read() # Read one byte

                try:
                    decoded_data = new_incoming_data.decode()
                    new_character = str(decoded_data)
                    # Append the new character to the read data
                    read_data += new_character
                except:
                    self.get_logger().info(f"Error decoding data")
                    return None
                
                if(time.time() - t0 > 1.0 and read_data[-1] != '}'):
                    self.get_logger().info(f"Received data, but incumplete after not responding for 1 second")
                    return None
                

            return read_data[1:]

        else:
            self.get_logger().info(f"Sensor not connected! - serial.is_open if False")
            return None
        

    def status_suscriber_callback(self, msg):
        self.status=msg

    def destroy_usb(self):
        if self.serial.open:

            self.serial.close

    def reconnect_sensor(self):

        # Close the serial port
        if self.serial.is_open:
            self.serial.close() # This is important
        
        del self.serial # This is important

        # Try to reconnect
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
            
            trials = 0
            data = None
            while (trials < 5 and data is None):

                data = self.read_sensor()

                if data is None:
                    self.get_logger().info(f"Failed to read sensor data. Trial: {trials}")
                    trials += 1
       

            # Check if the data is not empty
            if data is None:

                self.get_logger().info(f"Sensor does not respond!")
                # Try to reconnect to the sensor
                reconnect_response = self.reconnect_sensor()

                # If the reconnection is successful, try to read the sensor again
                if reconnect_response:
                    data = self.read_sensor()
                    if data is None:
                        self.get_logger().info(f"Sensor failed 5 times, reconnected, and still failed")
                        return
                else:
                    self.get_logger().info(f"Failed to reconnect to the sensor!")
                    return
                
            
                

            # Now we have the data, we can parse it
                
            self.get_logger().info(data)
            self.sensor_msg.success=True
            
            # Find all matches of the pattern in the input string
            matches = re.findall(self.pattern, data)

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
        sensor_node.destroy_usb()
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


