from .submodulos.MQTT import MQTT
import json
from rclpy.node import Node
import rclpy
from mavros_msgs.srv import CommandBool
from asv_interfaces.msg import Status


class MQTT_node(Node):

    def parameters(self):
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value
        self.declare_parameter('mqtt_addr', "20.126.131.210")
        self.mqtt_addr = self.get_parameter('mqtt_addr').get_parameter_value().string_value

    def declare_services(self):
        self.sendinfo = self.create_service(CommandBool, 'MQTT_send_info', self.sendinfo_callback)

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'topic', self.status_suscriber_callback, 10)

    def __init__(self):
        #start the node
        super().__init__('MQTT_node')

        #call the parameters
        self.parameters()

        #start MQTT Connection
        self.mqtt = MQTT(self.vehicle_id, addr=self.mqtt_addr, topics2suscribe=[f"veh{self.vehicle_id}"], on_message=self.on_message)
        #TODO raise error if MQTT fails so that main node knows about it to retry connection.

        #declare variables
        self.status=Status()
        #call services
        self.declare_services()

        self.declare_topics()

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
            "armed": self.status.armed
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

        global asv_mode, received_mqtt_wp
        if msg.topic == f"veh{self.status.vehicle_id}":
            message = json.loads(msg.payload.decode('utf-8'))  # Decode the msg into UTF-8

            #TODO: Topics to pub this info
            """if verbose > 0:
                self.get_logger().info(f"Received {message} on topic {msg.topic}")
            if message["mission_type"] == "STANDBY":  # Change the asv mission mode flag
                #asv_mode = 0
            elif message["mission_type"] == "GUIDED":
                #asv_mode = 1
            elif message["mission_type"] == "MANUAL":
                #asv_mode = 2
            elif message["mission_type"] == "SIMPLE":
                #asv_mode = 3
                #received_mqtt_wp = [message["lon"], message["lat"], 0]
            elif message["mission_type"] == "RTL":
                #asv_mode = 4"""


    def sendinfo_callback(self, request, response):
        try:
            if request.value:
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





def main():
    rclpy.init()
    mqtt_node = MQTT_node()
    rclpy.spin(mqtt_node)

if __name__ == '__main__':
    main()