from rclpy.action import ActionClient #for defining actions
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from asv_interfaces.msg import Status, Nodeupdate, Location, String, Sensor, Sonar
from asv_interfaces.srv import ASVmode, CommandBool, Newpoint, LoadMission, SensorParams, PlannerParams, CommandStr, SonarService
from rcl_interfaces.msg import Log
from asv_interfaces.action import Goto
from action_msgs.msg import GoalStatus
from .submodulos.call_service import call_service
from .submodulos.terminal_handler import ping_google, check_ssh_tunelling, start_ssh_tunneling, kill_ssh_tunelling, restart_asv, check_internet
from .submodulos.MQTT import MQTT
import json, traceback
from datetime import datetime
import threading

class MQTT_node(Node):

    def parameters(self):
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value
        self.declare_parameter('internet_loss_timeout', 30)
        self.internet_loss_timeout = self.get_parameter('internet_loss_timeout').get_parameter_value().integer_value
        self.declare_parameter('mqtt_addr', "adress")
        self.mqtt_addr = self.get_parameter('mqtt_addr').get_parameter_value().string_value
        self.declare_parameter('mqtt_user', "user")
        self.mqtt_user = self.get_parameter('mqtt_user').get_parameter_value().string_value
        self.declare_parameter('mqtt_password', "password")
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value
        

    def declare_services(self):
        self.sendinfo = self.create_service(CommandBool, 'MQTT_send_info', self.sendinfo_callback)
        self.asv_mission_mode_client = self.create_client(ASVmode, 'change_mission_mode')
        self.new_samplepoint_client = self.create_client(Newpoint, 'new_samplepoint')
        self.load_mission_client = self.create_client(LoadMission, 'load_mission')
        self.cancel_movement_client = self.create_client(CommandBool, 'cancel_movement')
        self.enable_planning_client = self.create_client(CommandBool, 'enable_planning')
        self.sensor_parameters_client = self.create_client(SensorParams, 'Sensor_params')
        self.camera_recording_client = self.create_client(CommandBool, 'camera_recording')
        # self.sonar_client=self.create_client(SonarService, 'sonar_service')
        self.load_map_client = self.create_client(CommandStr, 'load_map')
        self.reset_home_client = self.create_client(CommandBool, 'reset_home')

        


    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.mission_mode_subscriber = self.create_subscription(String, 'mission_mode', self.mission_mode_suscriber_callback, 10)
        self.log_subscriber = self.create_subscription(Log, '/rosout',self.log_subscriber_callback, 10)
        self.destination_subscriber = self.create_subscription(Location, 'destination', self.destination_subscriber_callback, 10)
        self.sensors_subscriber = self.create_subscription(Sensor, 'sensors', self.sensors_subscriber_callback, 10)
        self.sonar_subscriber = self.create_subscription(Sonar, 'sonar', self.sonar_suscriber_callback, 10)
        self.waypoints_subscriber = self.create_subscription(Location, 'waypoint_mark',self.waypoint_mark_subscriber_callback, 10)

    #def declare_actions(self):

    def __init__(self):
        #start the node
        super().__init__('MQTT_node')

        #call the parameters
        self.parameters()

        #check if there is internet connection
        while not ping_google(): #ping google has an internal delay
            self.get_logger().error("There is no internet connection, retrying...") 

        #start MQTT Connection
        try:
            self.get_logger().info(f"MQTT connecting to {self.mqtt_addr}")
            self.mqtt = MQTT(str(self.vehicle_id), addr=self.mqtt_addr, topics2suscribe=[f"veh{self.vehicle_id}"], on_message=self.on_message, on_disconnect=self.on_disconnect,user=self.mqtt_user,password=self.mqtt_password)
        except ConnectionRefusedError:
            self.get_logger().error(f"Connection to MQTT server was refused")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
        except OSError:
            self.get_logger().error(f"MQTT server was not found")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
        except TimeoutError:
            self.get_logger().error(f"MQTT was busy, timeout error")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
        except:
            error = traceback.format_exc()
            self.get_logger().fatal(f"MQTT connection failed, unknown error:\n {error}")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
            #TODO raise error if MQTT fails so that main node knows about it to retry connection.

        #declare variables
        self.status=Status()
        self.mission_mode = ""
        self.processing = False
        self.call_msg=ASVmode.Request()
        self.mqtt_point= None
        self.load_mission = -1
        self.cancel_movement = -1
        self.shutdown = False
        self.map_name=None
        self.camera_handler=False
        self.camera_signal=CommandBool.Request()
        self.update_params=False
        self.read_params=False
        self.reset_home=None
        self.enable_planner=CommandBool.Request()
        self.sensor_params=SensorParams.Request()
        self.sonar_params=SonarService.Request()
        self.sonar=Sonar()
        self.enable_planner.value=True #prestart as no planner
        #call services
        self.declare_services()

        #call actions
        #self.declare_actions()
        sleep(1)
        message = json.dumps({
            "veh_num": self.vehicle_id,
            "mission_mode": "STARTUP",
            "asv_mode": "Trying to connect"
        })  # Must be a JSON format file.
        self.mqtt.send_new_msg(message)  # Send the MQTT message
        



        self.declare_topics()
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.processing == True: #we have something to do
                sleep(0.01) # wait in case something needs to be done in another thread
                if self.cancel_movement != -1: #we need to load a mission
                    call_service(self, self.cancel_movement_client, self.cancel_movement)
                    self.cancel_movement = -1
                if self.load_mission != -1: #we need to load a mission
                    call_service(self, self.load_mission_client, self.load_mission)
                    self.load_mission = -1
                if self.call_msg.asv_mode != -1: #we have to change mode
                    call_service(self, self.asv_mission_mode_client, self.call_msg)
                    self.call_msg.asv_mode = -1
                if self.mqtt_point is not None: #we need to add a new mqtt point
                        call_service(self, self.new_samplepoint_client, self.mqtt_point)
                        self.get_logger().info(f"point sent")
                        self.mqtt_point = None
                if self.shutdown:
                    self.get_logger().warning("Drone was asked to shutdown, changing into LAND mode")
                elif self.update_params:
                    #call_service(self, self.enable_planning_client, self.aux)
                    self.get_logger().info(f"params write")
                    call_service(self, self.enable_planning_client, self.enable_planner)
                    call_service(self, self.sensor_parameters_client, self.sensor_params)
                    
                    self.update_params=False
                elif self.read_params:
                    #call_service(self, self.enable_planning_client, self.aux)
                    self.get_logger().info("params read")
                    self.sensor_read=call_service(self, self.sensor_parameters_client, self.sensor_params)
                    self.params_read()
                    self.read_params=False
                elif self.camera_handler:
                    call_service(self, self.camera_recording_client, self.camera_signal)
                    self.camera_handler=False
                elif self.map_name != None:
                    call_service(self, self.load_map_client, self.map_name)
                    self.map_name=None
                elif self.reset_home!= None:
                    call_service(self, self.reset_home_client, self.reset_home)
                    self.reset_home=None                    
                self.processing = False
            sleep(0.1)

    def asv_send_info(self):
        """
             A function that sends the ASV information to the coordinator in the MQTT Broker every 0.5 seconds.
        """
        if self.status.asv_mode == "Offline":
            return #if there is no connection to drone, dont update status

        msg = json.dumps({
            "Latitude": self.status.lat,
            "Longitude": self.status.lon,
            "Sonar": self.sonar.sonar,
                "yaw": self.status.yaw,
            "veh_num": self.status.vehicle_id,
            "battery": self.status.battery,
            "armed": self.status.armed,
            "mission_mode": self.mission_mode,
            "asv_mode": self.status.asv_mode,
            "EKF": self.status.ekf_ok,
            "manual_mode": self.status.manual_mode
        })  # Must be a JSON format file.
        #TODO: transformar el topic con la informacion a formato JSON
        self.mqtt.send_new_msg(msg)  # Send the MQTT message
        self.mqtt.send_new_msg(self.sonar.sonar, topic="sonar")

    def status_suscriber_callback(self, msg):
        self.status = msg

    def sonar_suscriber_callback(self, msg):
        self.sonar.sonar = msg.sonar

    def on_message(self, _client, _, msg):
        """
        Asyncronous handler of a MQTT message. Ir receives a message from the broker. Depending on the fields of the input
        message, change the mode consequently.`
        Args:
            _client: Client object
            msg: MQTT message object.
        """
        #TODO: if message are spammed it may crash
        if self.processing==True:
            sleep(0.2) #wait for 0.2 seconds
            self.processing=False #free processing as ROS2 may have crashed and unable to attend instruction
            return
        if msg.topic == f"veh{self.vehicle_id}":
            self.processing=True
            message = json.loads(msg.payload.decode('utf-8'))  # Decode the msg into UTF-8
            self.get_logger().info(f"Received {message} on topic {msg.topic}")
            if "mission_type" in message:
                if message["mission_type"] == "LAND":  # Change the asv mission mode flag
                    self.call_msg.asv_mode = 0
                elif message["mission_type"] == "STANDBY":
                    self.call_msg.asv_mode = 1
                elif message["mission_type"] == "PREMISSION":
                    self.call_msg.asv_mode = 2
                elif message["mission_type"] == "MANUAL":
                    self.call_msg.asv_mode = 3
                elif message["mission_type"] == "SIMPLEPOINT":
                    self.call_msg.asv_mode = 4
                    self.mqtt_point = Newpoint.Request()
                    self.mqtt_point.new_point.lat = message["lat"]
                    self.mqtt_point.new_point.lon = message["lon"]
                elif message["mission_type"] == "RTL":
                    self.call_msg.asv_mode = 5
                elif message["mission_type"] == "SHUTDOWN":
                    #rclpy.shutdown()
                    #TODO: rclpy.shutdown() is still developing and is not able to close nodes or processes with multithread
                    # so we will forcekill the processes for the time being
                    self.shutdown = True #tell ros2 we are shutting down
                    self.shutdown_thread = threading.Thread(target=self.shutdown_asv) #we will manage shutdown in a new thread
                    #so that MQTT can finish things while we shut down
                    self.shutdown_thread.start()
                else:
                    mission_type=message["mission_type"]
                    self.get_logger().warning(f"unknown mission_type call: {mission_type}")
            if "load_mission" in message:
                self.load_mission = LoadMission.Request()
                self.load_mission.file_name = str(message["load_mission"])

            if "cancel_movement" in message:
                self.cancel_movement = CommandBool.Request()
                self.cancel_movement.value = True

            if "update_parameters" in message:
                try:
                    self.enable_planner.value=bool(message["enable_planner"])
                    self.sensor_params.read_only=False
                    self.sensor_params.number_of_samples = int(message["num_samples"])
                    self.sensor_params.pump_channel = int(message["pump_channel"])
                    self.sensor_params.use_pump = bool(message["enable_pump"])
                    self.sensor_params.time_between_samples = float(message["time_between_samples"])
                    self.update_params=True
                except:
                    self.update_params=False #params arrived in a bad way
               
            if "read_params" in message:
                self.read_params=True
                self.sensor_params.read_only=True

            if "start_recording" in message:
                self.camera_handler=True
                self.camera_signal.value=True

            if "stop_recording" in message:
                self.camera_handler=True
                self.camera_signal.value=False
            
            if "load_map" in message:
                self.map_name=CommandStr.Request()
                self.map_name.string=message["load_map"]

            if "reset_home" in message:
                self.reset_home=CommandBool.Request()

    def on_disconnect(self,  client,  __, _):
        sleep(1)
        self.get_logger().error("connection to server was lost")
        if not ping_google():
            time_without_internet=0
            while not ping_google():
                self.get_logger().error("no internet connection, waiting for internet",once=True)
                sleep(1)
                time_without_internet+=1
                if time_without_internet >= self.internet_loss_timeout:
                    self.processing == True
                    self.call_msg.asv_mode = 3 #if we waited for too long, change into manual mode

            self.get_logger().info("Internet connection regained")
        else:
            self.get_logger().error("there is internet, unknown reason, retrying to connect to MQTT")


    def sendinfo_callback(self, request, response):
        try:
            if request.value:
                self.get_logger().info(f"Requested to send MQTT")
                # create a timer
                timer_period = 0.5  # seconds
                self.mqtt_timer = self.create_timer(timer_period, self.asv_send_info)
                self.get_logger().info('MQTT start sending')
                response.success = True
            else:
                self.mqtt_timer.destroy()
                self.get_logger().info('MQTT com Stopped sending')
                response.success = True
        except:
            self.get_logger().error('Couldn\'t use MQTT com')
            response.success=False
        return response

    def mission_mode_suscriber_callback(self, msg):
        self.mission_mode=msg.string

    def log_subscriber_callback(self, msg):
        timestamp= msg.stamp.sec #time of the message
        name=msg.name #name of the node that sent the message
        log=msg.msg #log
        file=msg.file #file the message comes from
        function=msg.function #function the message comes from
        line=msg.line #line the message comes from
        time= datetime.utcfromtimestamp(timestamp+3600).strftime('%Y-%m-%d %H:%M:%S') #add +3600 for Spains Clocktime

        message = json.dumps({
            "veh_num": self.vehicle_id,
            "origin node": name,
            "time": time,
            "msg": log
        })  # Must be a JSON format file.
        self.mqtt.send_new_msg(message, topic="log")  # Send the MQTT message

    def destination_subscriber_callback(self, msg):
        message = json.dumps({
            "veh_num": self.vehicle_id,
            "Latitude": msg.lat,
            "Longitude": msg.lon,
        })  # Must be a JSON format file.
        self.mqtt.send_new_msg(message, topic="destination")  # Send the MQTT message

    def waypoint_mark_subscriber_callback(self, msg):
        message = json.dumps({
            "veh_num": self.vehicle_id,
            "Latitude": msg.lat,
            "Longitude": msg.lon,
        })  # Must be a JSON format file.
        self.mqtt.send_new_msg(message, topic="waypoint")  # Send the MQTT message

    # def sonar_susbcriber_callback(self,msg):
    #     z = { "veh_num": self.vehicle_id,
    #         "date": msg.date,
    #         "Sonar":msg.sonar,
    #         "Latitude": self.status.lat,
    #         "Longitude": self.status.lon,
    #     }  # Must be a JSON format file.
    #     message = json.dumps(z)
    #     self.mqtt.send_new_msg(message, topic="sonar")  # Send the MQTT message
    #     self.get_logger().info(f'sonar data sent to database{message}')


    def sensors_subscriber_callback(self, msg):

        z = { "veh_num": self.vehicle_id,
            "date": msg.date,
            "sonar":self.sonar.sonar,
            "Latitude": self.status.lat,
            "Longitude": self.status.lon,
        }  # Must be a JSON format file.
        if msg.ph != -1:
            y= {"ph" : msg.ph}
            z.update(y)
        if msg.smart_water_battery != -1:
            y= {"smart_water_battery" : msg.smart_water_battery}
            z.update(y)
        if msg.o2 != -1:
            y = {"Disolved_Oxygen": msg.o2}
            z.update(y)
        if msg.temperature != -1:
            y = {"temperature": msg.temperature}
            z.update(y)
        if msg.conductivity != -1:
            y = {"conductivity": msg.conductivity}
            z.update(y)
        if msg.oxidation_reduction_potential != -1:
            y = {"oxidation_reduction_potential": msg.oxidation_reduction_potential}
            z.update(y)
        message = json.dumps(z)
        self.mqtt.send_new_msg(message, topic="database")  # Send the MQTT message
        self.get_logger().info(f'sensor data sent to database{message}')

    def shutdown_asv(self):
        try:
            self.mqtt_timer.destroy() #stop updating drone status
        except:
            pass
        self.call_msg.asv_mode = 0 #change to land mode to put vehicle in a safe spot        
        for i in range(4):
            msg = json.dumps({
                "veh_num": self.vehicle_id,
                "armed": self.status.armed,
                "mission_mode": "Shutting Down ROS2",
                "asv_mode": self.status.asv_mode
            })  # Must be a JSON format file.
            self.mqtt.send_new_msg(msg)  # Send the MQTT message
            sleep(0.5) #wait for things to finish
        message = json.dumps({
            "veh_num": self.vehicle_id,
            "origin node": "MQTT_node",
            "time": str(datetime.now()),
            "msg": "executing Restart"
        })  # Must be a JSON format file.
        self.mqtt.send_new_msg(message, topic="log")  # Send the MQTT message
        restart_asv() #restart ASV


    def params_read(self):
        msg = json.dumps({
                "veh_num": 3,
                "planner_status": self.enable_planner.value,
                "pump_status": self.sensor_read.use_pump,
                "number_of_samples": self.sensor_read.number_of_samples,
                "pump_channel": self.sensor_read.pump_channel,
                "time_between_samples": self.sensor_read.time_between_samples,
            })  # Must be a JSON format file.
        self.get_logger().info("parameters sent")
        self.mqtt.send_new_msg(msg, topic="parameters")  # Send the MQTT message


def main():
    rclpy.init()
    while rclpy.ok():
        try:
            mqtt_node = MQTT_node()
        except:
            """
            There has been an error with the program, so we will send the error log to the watchdog
            """
            x = rclpy.create_node('mqtt_node') #we state what node we are
            publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
            #we create the message
            msg = Nodeupdate()
            msg.node = "mqtt_node" #our identity
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