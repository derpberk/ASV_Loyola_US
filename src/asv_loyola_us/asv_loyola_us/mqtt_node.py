from rclpy.action import ActionClient #for defining actions
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from asv_interfaces.msg import Status, Nodeupdate, Location, String
from asv_interfaces.srv import ASVmode, CommandBool, Newpoint
from rcl_interfaces.msg import Log
from asv_interfaces.action import Goto
from action_msgs.msg import GoalStatus
from .submodulos.call_service import call_service
from .submodulos.MQTT import MQTT
import json, traceback
from datetime import datetime

class MQTT_node(Node):

    def parameters(self):
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value
        self.declare_parameter('mqtt_addr', "20.126.131.210")
        self.mqtt_addr = self.get_parameter('mqtt_addr').get_parameter_value().string_value

    def declare_services(self):
        self.sendinfo = self.create_service(CommandBool, 'MQTT_send_info', self.sendinfo_callback)
        self.asv_mission_mode_client = self.create_client(ASVmode, 'change_mission_mode')
        self.new_samplepoint_client = self.create_client(Newpoint, 'new_samplepoint')
    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.mission_mode_subscriber = self.create_subscription(String, 'mission_mode', self.mission_mode_suscriber_callback, 10)
        self.log_subscriber = self.create_subscription(Log, '/rosout',self.log_subscriber_callback, 10)

    #def declare_actions(self):

    def __init__(self):
        #start the node
        super().__init__('MQTT_node')

        #call the parameters
        self.parameters()

        #start MQTT Connection
        self.mqtt = MQTT(str(self.vehicle_id), addr=self.mqtt_addr, topics2suscribe=[f"veh{self.vehicle_id}"], on_message=self.on_message)
        #TODO raise error if MQTT fails so that main node knows about it to retry connection.

        #declare variables
        self.status=Status()
        self.mission_mode = ""
        self.processing = False
        self.call_msg=ASVmode.Request()
        self.mqtt_point= None
        #call services
        self.declare_services()

        #call actions
        #self.declare_actions()

        self.declare_topics()
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.call_msg.asv_mode != -1:
                call_service(self, self.asv_mission_mode_client, self.call_msg)
                if self.mqtt_point is not None:
                    call_service(self, self.new_samplepoint_client, self.mqtt_point)
                    self.get_logger().info(f"point sent")
                self.call_msg.asv_mode = -1
                self.processing = False
            sleep(0.1)


    def asv_send_info(self):
        """
             A function that sends the ASV information to the coordinator in the MQTT Broker every 0.5 seconds.
        """

        msg = json.dumps({
            "Latitude": self.status.lat,
            "Longitude": self.status.lon,
                "yaw": self.status.yaw,
            "veh_num": self.status.vehicle_id,
            "battery": self.status.battery,
            "armed": self.status.armed,
            "mission_mode": self.mission_mode,
            "asv_mode": self.status.asv_mode
        })  # Must be a JSON format file.
        #TODO: transformar el topic con la informacion a formato JSON
        self.mqtt.send_new_msg(msg)  # Send the MQTT message
    def status_suscriber_callback(self, msg):
        self.status = msg


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
            return
        if msg.topic == f"veh{self.status.vehicle_id}":
            self.processing=True
            message = json.loads(msg.payload.decode('utf-8'))  # Decode the msg into UTF-8
            self.get_logger().info(f"Received {message} on topic {msg.topic}")
            if message["mission_type"] == "REST":  # Change the asv mission mode flag
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
        time= datetime.utcfromtimestamp(timestamp+3600).strftime('%Y-%m-%d %H:%M:%S') #add +86400 for Spains Clocktime

        message = json.dumps({
            "veh_num": self.status.vehicle_id,
            "origin node": name,
            "time": time,
            "msg": log
        })  # Must be a JSON format file.
        self.mqtt.send_new_msg(message, topic="log")  # Send the MQTT message


def main():
    rclpy.init()
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