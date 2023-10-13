#ros libraries
import rclpy #main librarie
from rclpy.node import Node #for defining a node

from mavros_msgs.msg import State, WaypointReached, GlobalPositionTarget, Waypoint
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.srv import CommandBool, SetMode, WaypointClear, WaypointPush

from asv_interfaces.srv import PathPlanner

from std_msgs.msg import Bool

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


import os 
from time import sleep #delay

import queue # for queueing the WP


class ASV_node(Node):

    def initialize_parameters(self):

        # Get parameters from ROSPARAM
        self.declare_parameter('debug', True)
        self.debug = True #self.get_parameter('debug').get_parameter_value().bool_value
        #self.get_logger().info(f"Debug mode: {self.debug}")
        self.declare_parameter('use_path_planner', True)
        self.use_path_planner = self.get_parameter('use_path_planner').get_parameter_value().bool_value

    def declare_services(self):

        # This node connects to the following services:
        # 1 - /mavros/set_mode mavros_msgs/srv/SetMode
        self.set_mode_point_client = self.create_client(SetMode, '/mavros/set_mode')
        # 2 - /path_planner_service asv_interfaces/srv/PathPlanner
        self.path_planner_client = self.create_client(PathPlanner, '/ASV/path_planner_service')
        # 3 - /mavros/mission/clear mavros_msgs/srv/WaypointClear
        self.wp_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        # 4 - /mavros/mission/push mavros_msgs/srv/WaypointPush
        self.wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')

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
        # Subscribe to the asv position topic /mavros/global_position/global
        self.asv_position_subscriber = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)
  

    def asv_position_callback(self, msg):
        # This function is called when the asv position topic is updated
        #self.get_logger().info(f"ASV position: {msg.latitude}, {msg.longitude}")
        self.asv_position = msg

    def state_topic_callback(self, msg):
        # This function is called when the state topic is updated
        # self.get_logger().info(f"State MODE: {msg.mode}")
        self.asv_mode = msg.mode
        self.asv_armed = msg.armed

    def wp_reached_topic_callback(self, msg):
        # This function is called when the WP is reached
        self.get_logger().info(f"WP reached: {msg.wp_seq}")

        if msg.wp_seq == self.mission_length - 1:
            self.wp_reached = True
            self.get_logger().info(f"Last WP reached!")

    def battery_topic_callback(self, msg):
        # This function is called when the battery topic is updated
        self.battery = msg.voltage

    def start_asv_callback(self, msg):
        # This function is called when the start_asv topic is updated
        self.get_logger().info(f"ASV has been called to start!")
        self.start_asv = True
    
    def wp_target_callback(self, msg):
        # This function is called when the wp_target topic is updated
        self.get_logger().info(f"New target WP received: {msg.latitude}, {msg.longitude}")

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
        self.wp_cleared = False
        self.start_asv = False
        
        # The WP Queue
        self.wp_queue = queue.Queue() # To store the WPs
        self.FSM_STATE = "IDLE"
        self.asv_mode = "MANUAL"
        self.asv_armed = False

        # Declare Services, Action and Subscribers
        self.declare_services()
        self.declare_topics()
        
        sleep(5)

        while rclpy.ok():
            # Main loop #
            self.main_loop()
            rclpy.spin_once(self)
            sleep(0.1)

    def main_loop(self):
        # This is the main loop of the ASV node # 
        # It is basically a FMS

        # If the mode is in manual, to IDLE
        if not self.asv_mode in ['GUIDED','AUTO'] or not self.asv_armed:
            self.get_logger().info(f"We are in {self.asv_mode} mode, and ASV is {'armed' if self.asv_armed else 'not armed'}. Going to IDLE.")
            self.FSM_STATE = 'IDLE'
            self.mission_length = 0 # Reset the mission length

        # If the WP is cleared, go to IDLE #
        if self.wp_cleared:
            
            self.FSM_STATE = 'WAIT_WP'
            self.wp_cleared = False
            self.wp_queue = queue.Queue() # Clear the queue
            self.mission_length = 0 # Reset the mission length

            # Call the clear service - Sincronously#
            future = self.wp_clear_client.call_async(WaypointClear.Request())
            # Wait for the response #
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.get_logger().info(f"WP cleared!")

            self.set_mode('GUIDED') # Change the mode to GUIDED
        
        # THE FSM STATES #

        if self.FSM_STATE == 'IDLE':
            # Wait HERE until the ASV is called to start #
            if self.start_asv:
                self.get_logger().info(f"FSM to WAIT_WP state")
                self.FSM_STATE = 'WAIT_WP'
                self.start_asv = False  # Deactivate the flag

        elif self.FSM_STATE == 'WAIT_WP':
            # Wait HERe until a WP is received #

            # If the path planner is used, unpack the path into the WP queue #
    
            if self.wp_queue.qsize() > 0:

                # If the path queue is not empty, get the objective #
                wp_objective = self.wp_queue.get()

                # 1) Configure the request #

                if self.use_path_planner:
                    new_objective = PathPlanner.Request()
                    new_objective.origin.lat = self.asv_position.latitude
                    new_objective.origin.lon = self.asv_position.longitude
                    new_objective.destination.lat = wp_objective.latitude
                    new_objective.destination.lon = wp_objective.longitude

                    # 2) Call the path planner #
                    future = self.path_planner_client.call_async(new_objective)

                    # Wait for the response #
                    rclpy.spin_until_future_complete(self, future)
                    self.get_logger().info(f"Path planner answered")

                    # Get the response #
                    response = future.result()

                    if not response.success:
                        self.get_logger().info(f"Path planner was not successful.")
                        return
                
                    final_path = response.path.path
             
                else:
                    # If the path planner is not used, just put the WP as the only WP in the list#
                    final_path = [wp_objective]

                # Conform a list of WPs #
                wp_list = []
                mission_request = WaypointPush.Request()
                for point in final_path:
                    wp = Waypoint()
                    wp.x_lat = point.lat
                    wp.y_long = point.lon
                    wp.autocontinue = True
                    wp.command = 16
                    wp.frame = 0 # Global frame
                    wp_list.append(wp)
                
                mission_request.waypoints = wp_list
                mission_request.start_index = 0

                # Push the list of WPs to the vehicle #
                future = self.wp_push_client.call_async(mission_request)

                # spin until results
                rclpy.spin_until_future_complete(self, future)

                if future.result().success:
                    self.get_logger().info(f"New path injected!")
                else:
                    self.get_logger().info(f"Could not inject the mission.")
                    return
                
                # Get the length of the mission #
                self.mission_length = len(wp_list)
                
                # Change the state #
                self.FSM_STATE = 'GO_TO_WP'

            else:
                self.FSM_STATE = 'WAIT_WP'

        elif self.FSM_STATE == 'GO_TO_WP':
            # Initialize the mission #
            self.set_mode('AUTO')  # Set the mode to AUTO
            self.get_logger().info(f"Starting mission!")

            # Change the state #
            self.FSM_STATE = 'WAIT_TO_REACH'

        elif self.FSM_STATE == 'WAIT_TO_REACH':
            # Wait until the WP is reached or until the point is cleared #

            if self.wp_reached:

                self.FSM_STATE = 'WAIT_WP'
                self.wp_reached = False
                self.mission_length = 0 # Reset the mission length
                # Clear the mission #
                future = self.wp_clear_client.call_async(WaypointClear.Request())
                # Wait for the response #
                rclpy.spin_until_future_complete(self, future)
                # Change the mode to GUIDED #
                self.set_mode('GUIDED')

            else:
                self.FSM_STATE = 'WAIT_TO_REACH'
            
        # self.get_logger().info(f"ASV is in {self.FSM_STATE} state")


    def set_mode(self, mode):
        # This function changes the mode of the vehicle #

        # Create the request #
        request = SetMode.Request()
        request.custom_mode = mode

        # Send the request #
        future = self.set_mode_point_client.call_async(request)

        # Wait for the response #
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(f"Mode changed to {mode}")

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
