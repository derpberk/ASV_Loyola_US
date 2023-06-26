from rclpy.action import ActionClient #for defining actions
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from asv_interfaces.msg import Status, Nodeupdate, Location, String, Sensor
from asv_interfaces.srv import ASVmode, CommandBool, Newpoint, LoadMission, SensorParams, PlannerParams, CommandStr
from rcl_interfaces.msg import Log
from asv_interfaces.action import Goto
from action_msgs.msg import GoalStatus
from .submodulos.call_service import call_service
from .submodulos.terminal_handler import ping_google, check_ssh_tunelling, start_ssh_tunneling, kill_ssh_tunelling, restart_asv, check_internet
from .submodulos.MQTT import MQTT
from .submodulos.Database import Database
import json, traceback
from datetime import datetime
import threading
import asyncio
import aiomysql
import rclpy
from rclpy.node import Node
#from .submodulos.SensorModule import WaterQualityModule
import json
import time
import serial
import random
from asv_interfaces.srv import Takesample, SensorParams
from asv_interfaces.msg import Status, Sensor, Nodeupdate
from asv_interfaces.action import SensorSample


from datetime import datetime
from .submodulos.call_service import call_service #to call services
import traceback
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class Database_node(Node):

    def parameters(self):
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value
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
        self.sendinfo = self.create_service(CommandBool, 'Database_send_info', self.sendinfo_callback)



    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.sensors_subscriber = self.create_subscription(Sensor, 'sensors', self.sensors_subscriber_callback, 10)
        

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
            self.database = Database(host=self.database_host, port=self.database_port, user=self.database_user, password=self.database_password,db=self.database_db)
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
        self.sensor_params=SensorParams.Request()
        #call services
        # self.declare_services()
        # self.declare_topics()
        #call actions
        #self.declare_actions()
        self.veh_num= None
        self.date= None
        self.Latitude= None
        self.Longitude= None
        self.SAMPLE_NUM=None
        self.ph= None            
        self.batery=  None
        self.o2 = None
        self.temperature =  None
        self.conductivity = None
        self.oxidation_reduction_potencial =  None
        self.sonar= None

        self.database_thread = threading.Thread(target=self.on_sensor_data_recived)
        self.database_thread.start()


    def status_suscriber_callback(self, msg):
        self.status = msg


    def sensors_subscriber_callback(self, msg):

        self.get_logger().info(f"test2")
        self.date= msg.date,
        self.SAMPLE_NUM=self.sensor_read.number_of_samples
        if msg.ph != -1:
            self.ph= msg.ph
            
        if msg.smart_water_battery != -1:
            self.batery=  msg.smart_water_battery
            
        if msg.o2 != -1:
            self.o2 = msg.o2
            
        if msg.temperature != -1:
            self.temperature =  msg.temperature
            
        if msg.conductivity != -1:
            self.conductivity = msg.conductivity
            
        if msg.oxidation_reduction_potential != -1:
            self.oxidation_reduction_potencial =  msg.oxidation_reduction_potential

        if msg.sonar !=-1:
            self.sonar=msg.sonar
            
        # Call the insert_data function asynchronously
        # asyncio.create_task(self.insert_data())
        # self.database.insert_data(ID=self.vehicle_id,SAMPLE_NUM=self.SAMPLE_NUM,TEMP=self.temperature,PH=self.ph,DO=self.o2,LATITUD=self.Latitude,LONGITUD=self.Longitude,COND=self.conductivity,DATE=self.date)
        # self.get_logger().info(f'sensor data sent to database')



    # def insert_data(self):
    #     self.veh_num= self.vehicle_id,
    #     self.Latitude= self.status.lat,
    #     self.Longitude= self.status.lon
        
    #     database.insert_data(ID=self.vehicle_id,SAMPLE_NUM=self.SAMPLE_NUM,TEMP=self.temperature,PH=self.ph,DO=self.o2,LATITUD=self.Latitude,LONGITUD=self.Longitude,COND=self.conductivity,DATE=self.date)
    #     self.get_logger().info(f'sensor data sent to database')
    # # async def insert_data(self):
    # #         # Function to insert data into the database
    # #         async with self.pool.acquire() as conn:
    # #             async with conn.cursor() as cur:
    # #                 await cur.execute('INSERT INTO sensor (ID, SAMPLE_NUM, BAT, TEMP, PH, DO, LATITUD, LONGITUD, COND, ORP, DATE) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)',
    # #                                 (self.vehicle_id, self.SAMPLE_NUM, self.batery, self.temperature, self.ph, self.o2, self.Latitude, self.Longitude, self.conductivity, self.oxidation_reduction_potencial , self.date))

   


    # Database_thread = threading.Thread(target= self.insert_data)
    # Database_thread.start()
    # threading.Thread(target=self.on_sensor_data_recived).start()

    def on_sensor_data_recived(self):
        self.get_logger().info("Database")
        
        while True:
            if self.ping_device: #Si estamos concetados realizamos el checkeo
                if self.ping_device.get_ping_enable: #comprobamos si esta funcionando el sonar 
                    data = self.ping_device.get_distance()
                    value = str(data["distance"])
                    self.data_sonar = value

            self.database.insert_record(self=self, ID=self.id, SAMPLE_NUM=None, BAT=None, TEMP=None, PH=None, DO=None, LATITUD=None, LONGITUD=None, COND=None, ORP=None, SONAR=self.data_sonar, DATE=None)




    # def insert_data(self):
    #     # Function to insert data into the database
    #     current_thread = threading.current_thread()
    #     print("Hilo actual:", current_thread.name)
    #     self.get_logger().info(f"test3")
    #     self.veh_num= self.vehicle_id
    #     self.Latitude= self.status.lat
    #     self.Longitude= self.status.lon
        
    #     # cursor = self.database.connection.cursor()
    #     # query = 'INSERT INTO sensor (ID, SAMPLE_NUM, BAT, TEMP, PH, DO, LATITUD, LONGITUD, COND, ORP,SONAR, DATE) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)' 
    #     # values = (self.vehicle_id,self.SAMPLE_NUM,self.temperature,self.ph,self.o2,self.Latitude,self.Longitude,self.conductivity,self.sonar,self.date)
        

    #     # cursor.execute(query, values)
    #     # self.connection.commit()
    #     # cursor.close()

    #     self.database.insert_data(self.vehicle_id,self.SAMPLE_NUM,self.batery,self.temperature,self.ph,self.o2,self.Latitude,self.Longitude,self.conductivity,self.oxidation_reduction_potencial,self.sonar,self.date)

    # def sendinfo_callback(self, request, response):
    #     try:
    #         if request.value:
    #             self.get_logger().info(f"Requested to send datas")
    #             # create a timer
    #             timer_period = 0.5  # seconds
    #             self.database_timer = self.create_timer(timer_period, self.insert_data)
    #             self.get_logger().info('Datas start sending')
    #             response.success = True
    #         else:
    #             self.database_timer.destroy()
    #             self.get_logger().info('Datas com Stopped sending')
    #             response.success = True
    #     except:
    #         self.get_logger().error('Couldn\'t use Datas com')
    #         response.success=False
    #     return response
   
# async def main():
#     rclpy.init()
#     while rclpy.ok():
#         try:
#             database_node = Database_node()

#         except:
#             """
#             There has been an error with the program, so we will send the error log to the watchdog
#             """
#             x = rclpy.create_node('database_node')  # we state what node we are
#             publisher = x.create_publisher(Nodeupdate, '_internal_error', 10)  # we create the publisher
#             # we create the message
#             msg = Nodeupdate()
#             msg.node = "database_node"  # our identity
#             msg.message = traceback.format_exc()  # the error
#             # to be sure the message reaches, we must wait till wathdog is listening (publisher needs time to start up)
#             # TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
#             # este topic está oculto en echo al usar _
#             while publisher.get_subscription_count() == 0:  # while no one is listening
#                 sleep(0.01)  # we wait
#             publisher.publish(msg)  # we send the message
#             x.destroy_node()  # we destroy node and finish


# asyncio.run(main())
def main(args=None):
    rclpy.init(args=args)
    try:
        # start a class that servers the services
        database_node = Database_node()
        # loop the services
        rclpy.spin(database_node, executor=MultiThreadedExecutor())
        database_node.destroy_node()
    except:
        """
        There has been an error with the program, so we will send the error log to the watchdog
        """
        
        x = rclpy.create_node('database_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "database" #our identity
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