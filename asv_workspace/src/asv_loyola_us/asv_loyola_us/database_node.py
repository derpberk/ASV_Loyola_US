from rclpy.action import ActionClient #for defining actions
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from asv_interfaces.msg import Status, Nodeupdate, Location, String, Sensor, Sonar
from asv_interfaces.srv import ASVmode, CommandBool, Newpoint, LoadMission, SensorParams, PlannerParams, CommandStr
from rcl_interfaces.msg import Log
from asv_interfaces.action import Goto
from action_msgs.msg import GoalStatus
from .submodulos.call_service import call_service
from .submodulos.terminal_handler import ping_google, check_ssh_tunelling, start_ssh_tunneling, kill_ssh_tunelling, restart_asv, check_internet
from .submodulos.Database import Database
import json, traceback
from datetime import datetime
import threading
from .submodulos.asv_identity import get_asv_identity
from decimal import Decimal
class DB_node(Node):

    def parameters(self):
        
        self.vehicle_id = get_asv_identity()
        self.declare_parameter('internet_loss_timeout', 30)
        self.internet_loss_timeout = self.get_parameter('internet_loss_timeout').get_parameter_value().integer_value
        self.declare_parameter('database_host', '127.0.0.1')
        self.database_host = self.get_parameter('database_host').get_parameter_value().string_value
        self.declare_parameter('database_port', 3306)
        self.database_port = self.get_parameter('database_port').get_parameter_value().integer_value
        self.declare_parameter('database_user', 'root')
        self.database_user = self.get_parameter('database_user').get_parameter_value().string_value
        self.declare_parameter('database_password', 'password')
        self.database_password = self.get_parameter('database_password').get_parameter_value().string_value
        self.declare_parameter('database_db', 'Datas')
        self.database_db = self.get_parameter('database_db').get_parameter_value().string_value
        


    def declare_services(self):
        self.sensor_parameters_client = self.create_client(SensorParams, 'Sensor_params')
        #self.sendinfo = self.create_service(CommandBool, 'Database_send_info', self.sendinfo_callback)



    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.sensors_subscriber = self.create_subscription(Sensor, 'sensor', self.sensors_subscriber_callback, 10)
        self.sonar_subscriber = self.create_subscription(Sonar, 'sonar', self.sonar_suscriber_callback, 10)

    #def declare_actions(self):

    def __init__(self):
        #start the node
        super().__init__('Database_node')

        #call the parameters
        self.parameters()

        #check if there is internet connection
        while not ping_google(): #ping google has an internal delay
            self.get_logger().error("There is no internet connection, retrying...") 

        #start Database Connection
        try:
            self.get_logger().info(f"Database connecting to {self.database_host}")
            self.database = Database(hostname=self.database_host, port=self.database_port, username=self.database_user, password=self.database_password,database=self.database_db)
        except ConnectionRefusedError:
            self.get_logger().error(f"Connection to Database server was refused")
            self.get_logger().fatal("Database module is dead")
            self.destroy_node()
        except OSError:
            self.get_logger().error(f"Database server was not found")
            self.get_logger().fatal("Database module is dead")
            self.destroy_node()

        except:
            error = traceback.format_exc()
            self.get_logger().fatal(f"Database connection failed, unknown error:\n {error}")
            self.get_logger().fatal("Database module is dead")
            self.destroy_node()
            #TODO raise error if Database fails so that main node knows about it to retry connection.

        #declare variables
        self.status=Status()

        self.call_msg=ASVmode.Request()
        self.enable_planner=CommandBool.Request()
        self.sonar_msg = Sonar()
        self.sensor_msg=Sensor()

        
        self.declare_services()
        


        # declare topics
        self.declare_topics()

    def status_suscriber_callback(self, msg):
        self.status = msg



    def sonar_suscriber_callback(self,msg):
        self.get_logger().fatal("Sensing data")
        self.sonar_msg.date=str(datetime.now())
        self.sonar_msglat=msg.lat
        self.sonar_msglon=msg.lon
        self.sonar_msg_lat = Decimal(self.sonar_msglat)
        self.sonar_msg_lon = Decimal(self.sonar_msglon)
        self.sonar_msg.distance = msg.distance
        self.sonar_msg.lat = self.latitud
        self.sonar_msg.lon = self.longitud
        self.sonar_db= "Sonar"
        self.database.insert_record_data(Date=self.sonar_msg.date,Latitud=self.sonar_msg_lat, Longitud=self.sonar_msg_lon,Sensor=self.sonar_db,ASV=self.vehicle_id,Data=self.sonar_msg.distance)

    def sensors_subscriber_callback(self,msg):
        self.sensor_msg_date=str(datetime.now())
        self.sensor_msg.vbat= msg.vbat
        self.sensor_msg.ph=msg.ph
        self.sensor_msg.turbidity=msg.turbidity
        self.sensor_msg.temperature_ct=msg.temperature_ct
        self.sensor_msg.conductivity=msg.conductivity
        #self.sensor_msg.date=msg.date
        self.sensor_msglat=msg.lat
        self.sensor_msglon=msg.lon
        self.sensor_msg_lat = Decimal(self.sensor_msglat)
        self.sensor_msg_lon = Decimal(self.sensor_msglon)
        self.sensor_msg_batdb= "Battery"
        self.sensor_msg_phdb= "PH"
        self.sensor_msg_condb= "Conductivity"
        self.sensor_msg_turdb= "Turbidity"
        self.sensor_msg_tempdb= "Temperature"
        

        self.database.insert_record_data(Date=self.sensor_msg_date, Latitud=self.sensor_msg_lat, Longitud=self.sensor_msg_lon, Sensor=self.sensor_msg_batdb, ASV=self.vehicle_id, Data=self.sensor_msg.vbat)
        self.database.insert_record_data(Date=self.sensor_msg_date, Latitud=self.sensor_msg_lat, Longitud=self.sensor_msg_lon, Sensor=self.sensor_msg_phdb, ASV=self.vehicle_id, Data=self.sensor_msg.ph)
        self.database.insert_record_data(Date=self.sensor_msg_date, Latitud=self.sensor_msg_lat, Longitud=self.sensor_msg_lon, Sensor=self.sensor_msg_turdb, ASV=self.vehicle_id, Data=self.sensor_msg.turbidity)
        self.database.insert_record_data(Date=self.sensor_msg_date, Latitud=self.sensor_msg_lat, Longitud=self.sensor_msg_lon, Sensor=self.sensor_msg_tempdb, ASV=self.vehicle_id, Data=self.sensor_msg.temperature_ct)
        self.database.insert_record_data(Date=self.sensor_msg_date, Latitud=self.sensor_msg_lat, Longitud=self.sensor_msg_lon, Sensor=self.sensor_msg_condb, ASV=self.vehicle_id, Data=self.sensor_msg.conductivity)





def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    try:
        #start a class that servers the services
        db_node= DB_node()
        
        #loop the node
        rclpy.spin(db_node, executor=MultiThreadedExecutor())

        db_node.destroy_node()
    except:
            """
            There has been an error with the program, so we will send the error log to the watchdog
            """
            x = rclpy.create_node('db_node') #we state what node we are
            publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
            #we create the message
            msg = Nodeupdate()
            msg.node = "db_node" #our identity
            msg.message = traceback.format_exc() #the error
            #to be sure the message reaches, we must wait till wathdog is listening (publisher needs time to start up)
            #TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
            # este topic está oculto en echo al usar _
            while publisher.get_subscription_count() == 0: #while no one is listening
                sleep(0.01) #we wait
            publisher.publish(msg) #we send the message
            x.destroy_node() #we destroy node and finish


if __name__ == '__main__':
    main()