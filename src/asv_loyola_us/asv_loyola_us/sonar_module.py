from brping import Ping1D
import sys
import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
from asv_interfaces.msg import Sonar, Nodeupdate, Status
import threading
from std_msgs.msg import Float32
import time
import traceback
from rclpy.executors import MultiThreadedExecutor
import random
from math import exp, sin, cos

class Sonar_node(Node):
    def parameters(self):
        self.declare_parameter('serial_number', "DM00R2J8")
        self.serial_number = self.get_parameter('serial_number').get_parameter_value().string_value
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value
        

    def declare_topics(self):
        timer_period = 0.5  # seconds
        self.sonar_publisher = self.create_publisher(Sonar, 'sonar', 10)
        self.sonar_publisher_timer = self.create_timer(timer_period, self.sonar_publish)
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)

    def __init__(self):
        super().__init__("sonar_service")
        self.parameters()
        self.declare_topics()
        self.status=Status()
        self.sonar=Sonar()
        
        self.remembered_port = None
        self.ping_device = None
        self.ping_thread = None
        self.data0= None
        self.data1= None
        # self.declare_parameter('serial_number', "DM00R2J8")
        # self.service = self.create_service(Sonar, "sonar", self.sonar_callback)
        if self.DEBUG==False:
        
            self.port_monitor_thread = threading.Thread(target=self.monitor_usb_port, daemon=True)
            self.port_monitor_thread.start()



 # funcion para buscar por los puertos USB el numero de serie del sonar, si coincide el numero de serie obtiene el puerto donde se ubica, ademas en esta fucnion lo que realizara sera una busqueda continua del sistema.
 #  Se realiza para cuando hay una desconexion del USB y a continuacion conectamos de nuevo pueda volver a funcionar sin que tengamos que levantar el servicio de nuevo. 
 # La busqueda se realiza de manera constante cuando no se encuentra el dispositivo con el numero de serie.


 
    def monitor_usb_port(self):
        while True:
            arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if p.serial_number == self.get_parameter('serial_number').get_parameter_value().string_value # Numero de serie del sonar, cambiar al que se este utilizando
            ]
            if self.remembered_port not in arduino_ports:
                self.get_logger().info("USB device disconnected")
                self.ping_device = None
                self.remembered_port = None

            if not self.ping_device:
                for port in arduino_ports:
                    try:
                        ping_device = Ping1D() 
                        ping_device.connect_serial(port, 115200) #Nos conectamos al puerto hallado por el numero de serie
                        ping_device.initialize()                #Inicializamos
                        ping_device.set_ping_enable(True)       #Empieza a funcionar el sonar
                        self.ping_device = ping_device
                        self.remembered_port = port
                        self.get_logger().info(f"Connected to {port}")
                        break
                    except Exception as e:
                        self.get_logger().info(f"Failed to connect to {port}: {e}")
            rclpy.spin_once(self, timeout_sec=0.5)

            
     
# funcion para la comprobacion del sonar , se llama dentro de la clase Ping dada por el sonar a la funcion get_ping_enable que muestra 
# si esta activo la señal de salida acustica, que es la que se encarga de realizar la mediciones

    def sonar_callback(self, request, response):
        if self.ping_device: #Si estamos concetados realizamos el checkeo
            if self.ping_device.get_ping_enable: #comprobamos si esta funcionando el sonar 
                data=self.ping_device.get_distance()
                response.success = True          #confirmamos que esta fincionando
                response.value=str(data["distance"])
                response.confidence=str(data["confidence"])
                self.get_logger().info("Sonar working")
            else:                               #Si no esta funcionando, confirmamos de que no lo esta
                response.success = False
                self.get_logger().info("Sonar not working")
        return response
    
    def status_suscriber_callback(self, msg):
        self.status = msg

    def sonar_publish(self):
        if self.ping_device: #Si estamos concetados realizamos el checkeo
            if self.ping_device.get_ping_enable: #comprobamos si esta funcionando el sonar 
                data=self.ping_device.get_distance()
                        #confirmamos que esta fincionando
                self.sonar.sonar=float(data["distance"])
                
                # self.get_logger().info("Sonar send data")
                # self.get_logger().info(
                #     f"Result of check {self.sonar}"
                # )
        if self.DEBUG:
            
            self.data0=self.status.lat
            self.data1=self.status.lon
            self.sonar.sonar= cos(self.data0)+sin(self.data1)
            

        self.sonar_publisher.publish(self.sonar)
        



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