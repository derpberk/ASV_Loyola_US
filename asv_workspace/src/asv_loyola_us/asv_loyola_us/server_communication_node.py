#ros libraries
import rclpy #main librarie
from rclpy.node import Node #for defining a node

from .submodulos.MQTT import MQTT
from .submodulos.terminal_handler import ping_google
from .submodulos.get_asv_identity import get_asv_identity

from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from asv_interfaces.msg import SensorMsg
from asv_interfaces.msg import SonarMsg

# Import function to transform quaternion to euler
from .submodulos.quaternion_to_euler import quaternion_to_euler

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import json 
import rclpy
from time import sleep 
import traceback


class ServerCommunicationNode(Node):

    def initialize_parameters(self):

        # Declare some parameters #
        self.declare_parameter('internet_loss_timeout', 30)
        self.internet_loss_timeout = self.get_parameter('internet_loss_timeout').get_parameter_value().integer_value
        self.declare_parameter('mqtt_addr', "adress")
        self.mqtt_addr = self.get_parameter('mqtt_addr').get_parameter_value().string_value
        self.declare_parameter('mqtt_user', "user")
        self.vehicle_id = get_asv_identity()
        self.mqtt_user = 'asv' + str(get_asv_identity())
        self.declare_parameter('mqtt_password', "password")
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value

    def declare_topics(self):

        # This node connects to the following topics
        # 1) The state of the vehicle /mavros/state
        # 2) The battery /mavros/battery
        # 3) The start_asv topic /start_asv
        # 4) The wp_target topic /wp_target
        # 5) The position of the ASV /asv_position
        # 6) The wp_clear topic /wp_clear

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_BEF = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.asv_state_subscription = self.create_subscription(State, '/mavros/state', self.asv_state_callback, qos_profile)
        self.asv_battery_subscription = self.create_subscription(BatteryState, '/mavros/battery', self.asv_battery_callback, qos_profile_BEF)
        self.asv_position_subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)
        self.asv_orientation_subscription = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.asv_orientation_callback, qos_profile_BEF)

        self.wqp_sensor_subscription = self.create_subscription(SensorMsg, '/wqp_measurements', self.wqp_sensor_callback, qos_profile_BEF)
        self.sonar_sensor_subscription = self.create_subscription(SonarMsg, '/sonar_measurements', self.sonar_sensor_callback, qos_profile_BEF)

        # Publications
        self.start_asv_publisher = self.create_publisher(Bool, '/start_asv', qos_profile)
        self.wp_target_publisher = self.create_publisher(GlobalPositionTarget, '/wp_target', qos_profile)
        self.wp_clear_publisher = self.create_publisher(Bool, '/wp_clear', qos_profile)

    def declare_services(self):
        """ Services that this node offers and subscribes to."""

        # Service to change the mode of the ASV
        self.set_mode_srv = self.create_client(SetMode, '/mavros/set_mode')


    def __init__(self):
        super().__init__('communication_node')

        # Get parameters
        self.initialize_parameters()
        # Declare subscribers
        self.declare_topics()
        # Declare services
        self.declare_services()

        self.topic_identity = 'asv/asv' + str(self.vehicle_id)

        self.asv_mode = "MANUAL"
        self.battery = -1
        self.asv_position = {'latitude': 0, 'longitude': 0, 'heading': 0}


        # Declare MQTT topics
        topics = [self.topic_identity + '/start_asv', 
                self.topic_identity + '/wp_clear', 
                self.topic_identity + '/wp_target',
                self.topic_identity + '/asv_mode']
        
        for topic in topics:
            self.get_logger().info(f"Subscribing to {topic}")

        try:
            self.mqttConnection = MQTT(name=self.topic_identity, 
                                        addr=self.mqtt_addr, 
                                        user=self.mqtt_user, 
                                        password=self.mqtt_password, 
                                        on_message=self.on_message, 
                                        on_disconnect=self.on_disconnect,
                                        topics2suscribe=topics
                                        )
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

        # Declare timer for publishing data
        self.publishing_timer = self.create_timer(0.5, self.pub_timer_callback)

        # Spin forever
        while rclpy.ok():
            rclpy.spin_once(self)
            sleep(0.1)


    def pub_timer_callback(self):
        # When the timer is triggered, publish the data

        # Publish the state of the ASV
        self.mqttConnection.send_new_msg(self.asv_mode, topic = self.topic_identity + '/asv_state')

        # Publish the battery of the ASV
        self.mqttConnection.send_new_msg(self.battery, topic = self.topic_identity + '/asv_battery')

        # Publish the position of the ASV
        position_json = json.dumps(self.asv_position)
        self.mqttConnection.send_new_msg(position_json, topic = self.topic_identity + '/asv_position')


    def on_disconnect(self,  client,  __, _):

        sleep(1)
        self.get_logger().error("connection to server was lost")

        if not ping_google():
            time_without_internet = 0

            while not ping_google():

                self.get_logger().error("no internet connection, waiting for internet",once=True)
                sleep(1)
                time_without_internet += 1

                if time_without_internet >= self.internet_loss_timeout:
                    self.processing == True
                    self.call_msg.asv_mode = 3 #if we waited for too long, change into manual mode

            self.get_logger().info("Internet connection regained")
        else:
            self.get_logger().error("There is internet, unknown reason, retrying to connect to MQTT")

    def on_message(self, _client, _, msg):
        # This function is called when a message is received

        # Get the topic
        topic = msg.topic
        # Get the payload
        payload = msg.payload.decode("utf-8")

        # Check the topic
        if topic == self.topic_identity + '/start_asv':
            # If the topic is /asv_start, send the start command trough the topic
            msg = Bool()
            msg.data = payload == 1
            self.start_asv_publisher.publish(msg)
            self.get_logger().info("Start ASV command received")

        elif topic == self.topic_identity + '/wp_clear':
            # If the topic is /wp_clear, send the clear command trough the topic
            msg = Bool()
            msg.data = payload == 1
            self.wp_clear_publisher.publish(msg)
            self.get_logger().info("Clean WPs command received")

        elif topic == self.topic_identity + '/wp_target':
            # If the topic is /wp_target, send the WP command trough the topic
            # Format the payload to a dict (From json)
            payload = json.loads(payload)
            self.get_logger().info("New WP received: " + str(payload))
            msg = GlobalPositionTarget()
            try:
                msg.latitude = payload['latitude']
                msg.longitude = payload['longitude']
                # Publish the WP
                self.wp_target_publisher.publish(msg)
            except KeyError:
                self.get_logger().error("The payload of the requested WP is not correct")

        elif topic == self.topic_identity + '/asv_mode':
            # Call the service to change the mode of the ASV
            # Check if the mode is correct
            # Create the request
            request = SetMode.Request()
            request.custom_mode = payload
            # Call the service
            self.set_mode_srv.call_async(request)
            self.get_logger().info("Change mode command received: " + payload)
            
        else:
            self.get_logger().error("The topic " + topic + " is not recognized")
            self.get_logger().error("The payload is " + str(payload))

    def asv_state_callback(self, msg):
        # This function is called when the state topic is updated
        self.asv_mode = msg.mode
    
    def asv_battery_callback(self, msg):
        # This function is called when the battery topic is updated
        self.battery = msg.voltage

    def asv_position_callback(self, msg):

        self.asv_position['latitude'] = msg.latitude
        self.asv_position['longitude'] =  msg.longitude

    def asv_orientation_callback(self, msg):
         
         euler = quaternion_to_euler([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
         self.asv_position['heading'] = euler[2]

    def wqp_sensor_callback(self, msg):
        # This function is called when the wqp_sensor topic is updated
        # Check if the message is correct
        if msg.success:
            # If the message is correct, send the message to the MQTT server
            self.mqttConnection.send_new_msg(msg, topic = '/database/wqp')
        else:
            self.get_logger().error("The message received from the WQP sensor is not correct")

    def sonar_sensor_callback(self, msg):
        # This function is called when the sonar_sensor topic is updated
        if msg.success:
            self.mqttConnection.send_new_msg(msg, topic = '/database/sonar')
        else:
            self.get_logger().error("The message received from the sonar sensor is not correct")


def main(args=None):
    #init ROS2
    rclpy.init(args=args)

    #start a class that servers the services
    server_comms_node = ServerCommunicationNode()
    server_comms_node.destroy_node()

    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
