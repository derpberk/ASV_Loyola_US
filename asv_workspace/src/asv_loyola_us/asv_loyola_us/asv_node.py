#ros libraries
import rclpy #main librarie
from rclpy.node import Node #for defining a node

from mavros_msgs.msg import State, WaypointReached, GlobalPositionTarget
from sensor_msgs.msg import BatteryState
from mavros_msgs.srv import CommandBool, SetMode

from std_msgs.msg import Bool

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


import os 
from time import sleep #delay

import queue # for queueing the WP


class ASV_node(Node):

    def initialize_parameters(self):

        # Get parameters from ROSPARAM
        #self.declare_parameter('debug', True)
        self.debug = True #self.get_parameter('debug').get_parameter_value().bool_value
        #self.get_logger().info(f"Debug mode: {self.debug}")

    def declare_services(self):

        # This node connects to the following services:
        # 1 - /mavros/set_mode mavros_msgs/srv/SetMode
        self.set_mode_point_client = self.create_client(SetMode, '/mavros/set_mode')

    def declare_topics(self):

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
        # Subscribe to the state of the vehicle /mavros/state
        self.state_subscriber = self.create_subscription(State, '/mavros/state', self.state_topic_callback, qos_profile)
        # Subscribe to the WP reached topic /mavros/mission/reached
        self.wp_reached_subscriber = self.create_subscription(WaypointReached, '/mavros/mission/reached', self.wp_reached_topic_callback, qos_profile)
        # Subscribe to the battery topic /mavros/battery
        self.battery_subscriber = self.create_subscription(BatteryState, '/mavros/battery', self.battery_topic_callback, qos_profile_BEF)
        # Subscribe to the start_asv topic /start_asv
        self.start_asv_subscriber = self.create_subscription(Bool, '/start_asv', self.start_asv_callback, qos_profile)
        # Subscribe to the wp_target topic /wp_target (FROM MQTT)
        self.wp_target_subscriber = self.create_subscription(GlobalPositionTarget, '/wp_target', self.wp_target_callback, qos_profile)
        # Subscribe to the wp_clear topic /wp_clear (FROM MQTT)
        self.wp_clear_subscriber = self.create_subscription(Bool, '/wp_clear', self.wp_clear_callback, qos_profile)
        
        # Topic to send the WP to ardupilot /mavros/setpoint_raw/global
        self.wp_publisher = self.create_publisher(GlobalPositionTarget, '/mavros/setpoint_raw/global',  qos_profile_BEF)



    def state_topic_callback(self, msg):
        # This function is called when the state topic is updated
        self.get_logger().info(f"State MODE: {msg.mode}")
        self.asv_mode = msg.mode

    def wp_reached_topic_callback(self, msg):
        # This function is called when the WP is reached
        self.get_logger().info(f"WP reached: {msg.wp_seq}")
        self.wp_reached = True

    def battery_topic_callback(self, msg):
        # This function is called when the battery topic is updated
        self.battery = msg.voltage

    def start_asv_callback(self, msg):
        # This function is called when the start_asv topic is updated
        self.get_logger().info(f"ASV has been called to start!")
        self.start_asv = True
    
    def wp_target_callback(self, msg):
        # This function is called when the wp_target topic is updated
        self.get_logger().info(f"New WP received: {msg.latitude}, {msg.longitude}")
        self.wp_queue.put(msg, block=False) # Put the WP in the queue without blocking the main thread

    def wp_clear_callback(self, msg):
        # Just put the flag to msg #
        self.get_logger().info(f"WP cleared!")
        self.wp_cleared = True


    def __init__(self):
        super().__init__('asv_node')
        
        # Declare some parameters #
        self.initialize_parameters()

        # Initialise the mission state
        self.wp_reached = False
        self.wp_waiting = False
        self.wp_cleared = False
        self.wp_received = None
        self.start_asv = False
        # The WP Queue
        self.wp_queue = queue.Queue()
        self.FSM_STATE = "IDLE"
        self.asv_mode = "MANUAL"

        # Declare Services, Action and Subscribers
        self.declare_services()
        self.declare_topics()
        
        while rclpy.ok():
            # Main loop #
            self.main_loop()
            rclpy.spin_once(self)
            sleep(0.1)

    def main_loop(self):
        # This is the main loop of the ASV node # 
        # It is basically a FMS

        # If the mode is in manual, to IDLE
        if self.asv_mode != 'GUIDED':
            self.get_logger().info(f"We are in {self.asv_mode} mode, going to IDLE")
            self.FSM_STATE = 'IDLE'
        
        # THE FSM STATES #

        if self.FSM_STATE == 'IDLE':
            # Wait for the start command #
            if self.start_asv:
                self.get_logger().info(f"FSM to WAIT_WP state")
                self.FSM_STATE = 'WAIT_WP'
                self.set_mode('GUIDED')  # Change the mode to GUIDED 
                self.start_asv = False  # Deactivate the flag
            
        elif self.FSM_STATE == 'WAIT_WP':
            # Wait for the WP #
            if self.wp_queue.empty():
                # If the queue is empty, wait #
                self.FSM_STATE = 'WAIT_WP'
            
            else:
                # If the queue is not empty, get the WP #
                self.wp_received = self.wp_queue.get()
                self.FSM_STATE = 'GO_TO_WP'
            
        elif self.FSM_STATE == 'GO_TO_WP':
            # Go to the WP #
            self.go_to_point(self.wp_received)
            self.FSM_STATE = 'WAIT_TO_REACH'

        elif self.FSM_STATE == 'WAIT_TO_REACH':
            # Wait until the WP is reached or until the point is cleared #

            if self.wp_cleared:
                self.FSM_STATE = 'WAIT_WP'
                self.wp_cleared = False
                # Empty the queue #
                self.queue = queue.Queue()

            if self.wp_reached:
                self.FSM_STATE = 'WAIT_WP'
                self.wp_reached = False
            else:
                self.FSM_STATE = 'WAIT_TO_REACH'
            
        # self.get_logger().info(f"ASV is in {self.FSM_STATE} state")

    def go_to_point(self, wp):

        # This function sends the vehicle to a point by publishing the WP to the topic #

        # Create the request #
        msg = GlobalPositionTarget()
        msg.latitude = wp.latitude
        msg.longitude = wp.longitude

        # Send the request #
        self.wp_publisher.publish(msg)


    def set_mode(self, mode):
        # This function changes the mode of the vehicle #

        # Create the request #
        request = SetMode.Request()
        request.custom_mode = mode

        # Send the request #
        future = self.set_mode_point_client.call_async(request)

        return future



def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    
    #start a class that servers the services
    asv_node = ASV_node()
    asv_node.destroy_node()

    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
