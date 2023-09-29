import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from dronekit import connect, VehicleMode, LocationGlobal
import numpy as np
import pymavlink
import traceback
from math import atan2
import time
from pymavlink.dialects.v20 import ardupilotmega as mavlink2 #for obstacle distance information
from numpy import uint
from .submodulos.asv_identity import get_asv_identity

from asv_interfaces.msg import Status, Nodeupdate, Camera, Obstacles, Location
from asv_interfaces.srv import CommandBool, ASVmode, Newpoint, Takesample
from asv_interfaces.action import Goto


import os
os.environ['MAVLINK20'] = '1'


ardupilotModeDictionary = {

    "ASV_MODE": {
        "0":  "MANUAL",
        "1":  "CIRCLE",
        "2":  "STABILIZE",
        "3":  "TRAINING",
        "4":  "ACRO",
        "5":  "FBWA",
        "6":  "FBWB",
        "7":  "CRUISE",
        "8":  "AUTOTUNE",
        "10": "AUTO",
        "11": "RTL",
        "12": "LOITER",
        "13": "TAKEOFF",
        "14": "AVOID_ADSB",
        "15": "GUIDED",
        "16": "INITIALISING",
        "17": "QSTABILIZE",
        "18": "QHOVER",
        "19": "QLOITER",
        "20": "QLAND",
        "21": "QRTL",
        "22": "QAUTOTUNE",
        "23": "QACRO",
        "24": "THERMAL"
        },
    "MISSION_MODE": {
        "0": "STANDBY",
        "1": "GUIDED",
        "2": "MANUAL",
        "3": "SIMPLE",
        "4": "RTL"
        }

    }


class Dronekit_node(Node):

    """ This node is in charge of the communication with the drone using dronekit interface.
    It implements the following services/actions """

    # Services:
    #   - CommandBool: to arm/disarm the drone
    #   - ASVmode: to change the mode of the drone
    #   - Resethome: to reset the home location of the drone

    # Actions:
    #   - Goto: to send the drone to a given location/s


    def parameters(self):

        # Declare the parameters of the node

        # Parameter to enable/disable the debug mode
        self.declare_parameter('debug', True)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

        # Parameter to set the vehicle IP
        self.declare_parameter('vehicle_ip', 'tcp:navio.local:5678')
        self.declare_parameter('debug_vehicle_ip', 'tcp:127.0.0.1:5760')
        # If debug mode is enabled, the vehicle IP is set to the debug_vehicle_ip parameter
        if not self.DEBUG:
            self.vehicle_ip = self.get_parameter('vehicle_ip').get_parameter_value().string_value
        else:
            self.vehicle_ip = self.get_parameter('debug_vehicle_ip').get_parameter_value().string_value

        self.vehicle_id = get_asv_identity()

        # Parameter to set the connection timeout
        self.declare_parameter('connect_timeout', 15)
        self.connect_timeout = self.get_parameter('connect_timeout').get_parameter_value().integer_value
        # Parameter to set the maximum distance of a Waypoint
        self.declare_parameter('max_distance', 5000)
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().integer_value
        # The vehicle arm timeout in seconds 
        self.declare_parameter('arm_timeout', 15)
        self.arm_timeout = self.get_parameter('arm_timeout').get_parameter_value().integer_value
        # The vehicle travellin timeout in seconds
        self.declare_parameter('travelling_timeout', 60)
        self.travelling_timeout = self.get_parameter('travelling_timeout').get_parameter_value().integer_value
        # WP_RADIUS
        self.declare_parameter('wp_radius', 0.5)
        self.wp_radius = self.get_parameter('wp_radius').get_parameter_value().double_value

    def declare_services(self):
        """ Declare the services of the node: i) the host services, ii) the client services """

        # i) Host services
        self.arm_vehicle_service = self.create_service(CommandBool, 'arm_vehicle', self.arm_vehicle_callback)
        self.mode_vehicle_service = self.create_service(ASVmode, 'change_asv_mode', self.change_asv_mode_callback)
        self.reset_home_service = self.create_service(CommandBool, 'reset_home', self.reset_home_callback)

        # ii) Client services
        self.calculate_path_client = self.create_client(Newpoint, 'calculate_path')

    def declare_actions(self):
        """ Declare the actions of the node """

        # GoTo action #
        self.go_to_server = ActionServer(self, Goto, 'goto', execute_callback=self.goto_execute_callback,
                                         goal_callback=self.goto_check_goal_callback,
                                         cancel_callback=self.goto_cancel_callback,
                                         callback_group=ReentrantCallbackGroup())


    def declare_topics(self):
        """ Declare the topics of the node """

        timer_period = 0.1  # seconds
        self.status_publisher = self.create_publisher(Status, 'status', 10)
        self.status_publisher_timer = self.create_timer(timer_period, self.status_publish)
        self.waypoints_publisher = self.create_publisher(Location, 'waypoint_mark', 10)

    
    def __init__(self):
        """ Constructor """

        super().__init__('dronekit_node')

        # Declare the parameters of the node
        self.parameters()
        # Initialize the status
        self.status = Status()
        # connect to vehicle

        if self.DEBUG:
            self.get_logger().warning("Debug mode enabled")



        self.get_logger().info(f"Connecting to vehicle in {self.vehicle_ip}")

        try:
            # Connect to the vehicle using the vehicle IP
            self.vehicle = connect(self.vehicle_ip, timeout=self.connect_timeout, source_system=1, source_component=93)
            self.get_logger().info(f"Connected to vehicle in {self.vehicle_ip}")
            # Create the timer to read the RC channels
            self.rc_read_timer=self.create_timer(0.1, self.rc_read_callback)
            # Declare the services and other stuff
            self.declare_topics()
            self.declare_services()
            self.declare_actions()
            # Initialize the vehicle
            self.vehicle.add_message_listener('SYS_STATUS', self.vehicle_status_callback)
            self.cmds = self.vehicle.commands
            self.get_logger().info(f"Waiting for initialization")
            self.cmds.download()
            self.cmds.wait_ready()
            self.home = self.vehicle.home_location
            self.get_logger().info(f"Vehicle home location is {self.home}")

        except ConnectionRefusedError:
            self.get_logger().error(f"Connection to navio2 was refused")
            self.get_logger().fatal("Drone module is dead")
            self.destroy_node()

        except OSError:
            self.get_logger().error(f"Navio2 was not found in the same network")
            self.get_logger().fatal("Drone module is dead")
            self.destroy_node()

        except TimeoutError:
            self.get_logger().error(f"Navio2 port was busy, timeout error")
            self.get_logger().fatal("Drone module is dead")
            self.destroy_node()

        except:
            error = traceback.format_exc()
            self.get_logger().error(f"Connection to navio2 could not be made, unknown error:\n {error}")
            self.get_logger().fatal("Drone module is dead")
            self.destroy_node()

    def vehicle_status_callback(self, vehicle, name, msg):
        pass

    def reset_home_callback(self, request, response):
        self.vehicle.home_location = self.vehicle.location.global_frame

    def arm_vehicle_callback(self, request, response):
        """ Vehicle arm service callback:

        request.value: False to disarm, True to arm

        """

        # First, read the current status of the vehicle
        if self.status.manual_mode:
            # In manual mode, the vehicle cannot be armed because 
            # it is not safe to override the RC
            self.get_logger().error("RC IN MANUAL -> Vehicle cannot be armed/disarmed in manual mode")
            response.success = False
            return response
        else:

            try:
                if request.value:
                    self.vehicle.arm()
                    self.get_logger().info("Vehicle armed")
                else:
                    self.vehicle.disarm()
                    self.get_logger().info("Vehicle disarmed")
                # Transmit the new status
                while self.vehicle.armed != request.value:
                    self.get_logger().info("Waiting for vehicle to arm/disarm")
                    time.sleep(0.1)

            except:
                self.get_logger().error("Vehicle could not be armed/disarmed")
                response.success = False
        
        return response

    def rc_read_callback(self):
        if self.DEBUG==False:
            #if there is no RC return home
            try:
                #manage no RC detected
                if all([self.vehicle.channels['5'] == 0, self.vehicle.channels['6'] == 0]):
                    if self.vehicle.mode != VehicleMode("RTL"):
                        self.get_logger().info("seems there is no RC connected", once=True)
                        self.status.manual_mode = False #override RC if there is no connection
                        
                #manage RC switch between auto and manual mode
                elif self.status.manual_mode!=(self.vehicle.channels['6']>1200):
                    self.get_logger().info("manual interruption" if self.vehicle.channels['6']>1200 else "automatic control regained")
                    self.status.manual_mode=bool(self.vehicle.channels['6']>1200)

                #manage arm when RC in manual
                elif self.status.manual_mode:                
                    if self.vehicle.mode != VehicleMode("MANUAL") : #vehicle is not in desired mode
                        self.get_logger().info("manual switching vehicle mode to manual")
                        self.vehicle.mode = VehicleMode("MANUAL")
                    if self.vehicle.armed!=(self.vehicle.channels['5']>1200):
                        if self.vehicle.channels['5']>1200:
                            self.vehicle.arm()
                            self.get_logger().info("manual arm")
                        else:
                            self.vehicle.disarm()
                            self.get_logger().info("manual disarm")
            except:
                pass

    def status_publish(self):

        try:
            # TODO: es necesario que dronekit te devuelva valores no corruptos, crash otherwise
            self.status.lat = self.vehicle.location.global_relative_frame.lat
            self.status.lon = self.vehicle.location.global_relative_frame.lon
            self.status.yaw = self.vehicle.attitude.yaw
        except:
            pass

        #Treat batery apart, as it uses to fail
        try:
            self.status.battery = float(self.vehicle.battery.voltage)
        except:
            pass

        #finally if one of these values fails, dont send the message, this values are always reported
        try:
            self.status.armed = self.vehicle.armed
            self.status.vehicle_id = self.vehicle_id
            self.status.asv_mode=str(self.vehicle.mode.name)
            self.status.ekf_ok = bool(self.vehicle.ekf_ok)
            self.status_publisher.publish(self.status)
        except:
            pass

    def get_bearing(self, location1, location2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.
        This method is an approximation, and may not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py`
        Args:
            location1: Actual position (`dronekit.LocationGlobal`).
            location2: Reference position (`dronekit.LocationGlobal).
        Returns:
            The angle difference from `location1 to `location2
        """
        off_x = location2.lon - location1.lon
        off_y = location2.lat - location1.lat
        bearing = 90.00 + atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing

    def call_service(self, client,  msg):
        # TODO: raise error to avoid infinite wait if service is not up, after all means a module is not active $$ watchdog will be in charge
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f'service {client.srv_name} not available', once=True)
            return False
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    return response
                break


    def calculate_distance(self, goal_loc):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        Args:
            goal_loc: Reference position (dronekit.LocationGlobal).
        Returns:
            distance from the ASV to the goal_loc in meters
        """

        # Convert to radians #
        lat1 = np.radians(self.vehicle.location.global_relative_frame.lat)
        lat2 = np.radians(goal_loc.lat)
        lon1 = np.radians(self.vehicle.location.global_relative_frame.lon)
        lon2 = np.radians(goal_loc.lon)

        # Obtains the latitude/longitude differences #
        d_lat = lat2 - lat1
        d_lon = lon2 - lon1

        # Returns True if the waypoint is within 1.5 meters the ASV position
        a = np.sin(0.5 * d_lat) ** 2 + np.sin(0.5 * d_lon) ** 2 * np.cos(lat1) * np.cos(lat2)
        c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
        return 6378100.0 * c

    def change_asv_mode_callback(self, request, response):
        """ Change the ASV mode """

        if self.status.manual_mode:
            self.get_logger().info(f"RC in manual, cannot change mode")
            return response

        # Check if the mode is valid
        if request.asv_mode_str in ardupilotModeDictionary["ASV_MODE"].values():
            mode = request.asv_mode_str
        else:
            self.get_logger().info(f"{request.asv_mode_str} is not a valid mode")
            response.success = False
            return response

        # Change the mode
        self.vehicle.mode = VehicleMode(mode)
        response.success = True
        return response
    
    def goto_check_goal_callback(self, goal_request):
        """ This callback is called when a new goal is received. 
        It checks if the goal is valid """

        self.get_logger().info("Received new point to go to")

        # Check if the goal is valid
        if not self.vehicle.armed or self.vehicle.mode != VehicleMode("LOITER"):
            self.get_logger().error(f'Error: vehicle should be armed and in loiter mode\nbut arming is {self.vehicle.armed} and vehicle is in {self.vehicle.mode}.')
            if not self.vehicle.ekf_ok:
                self.get_logger().error(f"EKF seems to be the main issue, System Status: mode {self.vehicle.mode.name}, GPS_status: {self.vehicle.gps_0}, System status: {self.vehicle.system_status.state}, System able to arm {self.vehicle.is_armable}")
            return GoalResponse.REJECT #To move we must be armed and in loiter

        if self.status.manual_mode:
            self.get_logger().error(f'RC in manual mode, action rejected')
            return GoalResponse.REJECT

        if self.calculate_distance(goal_request.samplepoint)> self.max_distance:
            self.get_logger().error(f'we are too far away from the destination point {self.calculate_distance(goal_request.samplepoint)}m, cancelling action')
            return GoalResponse.REJECT

        self.get_logger().info(f'Action accepted')
        return GoalResponse.ACCEPT

    def goto_cancel_callback(self,goal_handle):
        """ This callback is called when the action is cancelled """

        self.get_logger().info("Action cancelled")
        return CancelResponse.ACCEPT

    def goto_execute_callback(self, goal_handle): 
        """ This callback is called when a new goal is received.

        It sends the vehicle to the given location.

        """
        # Feedback message is the remaining distance to the goal
        feedback_msg = Goto.Feedback()
        # Result message is a boolen indicating if the goal was reached
        result = Goto.Result()

        # Compute the path #
        destination_point = Newpoint.Request()
        destination_point.new_point = goal_handle.request.samplepoint

        # Compute the free path to the goal
        path = self.call_service(self.calculate_path_client, destination_point)

        # Check if the path is valid
        if path == False or path.success == False:
            self.get_logger().error("The planner could not find a valid path")
            self.get_logger().error("Going to the goal directly without checking for obstacles")
            # If the path is not valid, go directly to the goal
            path = [goal_handle.request.samplepoint]
        else:
            # If the path is valid, go to the first waypoint
            path = path.point_list

        # Set the vehicle to GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")

        # Go to iteratively to each waypoint
        arm_counter = 0

        while rclpy.ok() and len(path) > 0:

            # Get the next waypoint
            next_waypoint = path.pop(0)
            self.get_logger().info(f"Going to {next_waypoint}")

            # Send the vehicle to the next waypoint
            self.vehicle.simple_goto(LocationGlobal(next_waypoint.lat, next_waypoint.lon, 0.0))
            self.publish_waypoint(next_waypoint)

            feedback_count = 0
            
            # Wait until the vehicle is close to the waypoint
            while rclpy.ok() and not self.reached_position(next_waypoint):

                time.sleep(0.1)
                # Publish the remaining distance to the goal
                feedback_count  += 1
                if feedback_count == 50:
                    feedback_msg.distance = self.calculate_distance(next_waypoint)
                    goal_handle.publish_feedback(feedback_msg)
                    feedback_count = 0

                # Check the abort conditions #
                # 1) The action is cancelled -> To loiter around actual position #
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.vehicle.mode = VehicleMode("LOITER")
                    self.get_logger().info("GOTO Action cancelled")
                    result.finish_flag = "Action cancelled"
                    return result

                # 2) The vehicle is in manual mode -> To loiter around actual position #
                elif self.status.manual_mode:
                        self.get_logger().warning("Manual interruption during mission")
                        result.finish_flag = "Manual interruption"
                        goal_handle.abort()
                        return result

                # 3) EKF is not ok 
                elif not self.vehicle.ekf_ok: #if ekf is not ok, stop

                    self.get_logger().warning('EKF FAILED, sensor read not good enough, waiting for better signal, switching to manual and disarming')
                    self.vehicle.mode = VehicleMode("MANUAL")
                    self.vehicle.disarm()
                    result.finish_flag = "ekf timeout"
                    self.get_logger().info('EKF timeout, exiting action')
                    goal_handle.abort()
                    return result

                # 4) The vehicle is not in GUIDED mode -> vehicle is not in desired mode
                elif self.vehicle.mode != VehicleMode("GUIDED"): #vehicle is not in desired mode
                    self.get_logger().warning("asv_mode was changed externally")
                    self.vehicle.mode = VehicleMode("GUIDED") # restore mode
                    self.vehicle.simple_goto(LocationGlobal(goal_handle.request.samplepoint.lat, goal_handle.request.samplepoint.lon, 0.0))

                # 5) The vehicle is not armed -> vehicle is not armed
                elif not self.status.armed: #event vehicle disarmed

                    if not self.vehicle.is_armable:
                        self.get_logger().fatal('vehicle is not armable, inconsistent state')
                    
                    else:
                        if arm_counter % 10 == 0:
                            self.get_logger().info('received external disarm in auto mode, waiting for RC')
                        
                        # Wait for RC to arm
                        if self.vehicle.channels['5'] > 1200:
                            self.vehicle.arm()
                            self.get_logger().info('Vehicle armed!')
                            arm_counter=0
                        else:
                            arm_counter+=1

                        # If the vehicle is not armed after arm_timeout seconds, abort the action
                        if arm_counter > self.arm_timeout:
                            self.get_logger().info('Waited for RC for too long, exiting action')
                            result.finish_flag = "timeout vehicle disarmed"
                            goal_handle.abort()
                            return result

            # Goal reached
            self.get_logger().info(f"Reached {next_waypoint}!")

        # If the vehicle is in GUIDED mode, set it to LOITER
        if self.vehicle.mode == VehicleMode("GUIDED"):
            self.vehicle.mode = VehicleMode("LOITER")
        goal_handle.succeed()
        self.get_logger().info("Final goal reached!")

        result.success = True
        result.finish_flag = "Normal execution"
        return result

    
    def reached_position(self, goal_loc):
        """
        Args:
            goal_loc: Reference position (dronekit.LocationGlobal).
        Returns:
            'True' if the ASV distance respecto to the target Waypoint is less than 0.5 meters.
        """

        return self.calculate_distance(goal_loc) < self.wp_radius

    def publish_waypoint(self, waypoint):
        """ Publish the waypoint to the waypoint_mark topic """

        waypoint_msg = Location()
        waypoint_msg.lat = waypoint.lat
        waypoint_msg.lon = waypoint.lon
        self.waypoints_publisher.publish(waypoint_msg)


def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    try:
        #start a class that servers the services
        dronekit_node = Dronekit_node()
        #loop the node
        rclpy.spin(dronekit_node, executor=MultiThreadedExecutor())

        dronekit_node.destroy_node()

    except:
        """
        There has been an error with the program, so we will send the error log to the watchdog
        """
        x = rclpy.create_node('dronekit_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "dronekit_node" #our identity
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
