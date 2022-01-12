import rclpy
from rclpy.node import Node
#TODO: deprecate dronekit, use pymavlink
from dronekit import connect, VehicleMode, LocationGlobal
import numpy as np
import pymavlink
import traceback
from math import atan2
from .submodulos.dictionary import dictionary
import time
#import intefaces
from asv_interfaces.msg import Status
from asv_interfaces.srv import CommandBool, ASVmode, Newpoint


modes_str = []

# This node generates the necessary services for  comunication towards the drone
#parameters are only read at start

class Dronekit_node(Node):

    #his functions defines and assigns value to the parameters
    def parameters(self):
        self.declare_parameter('vehicle_ip', 'tcp:127.0.0.1:5762')
        self.vehicle_ip = self.get_parameter('vehicle_ip').get_parameter_value().string_value
        self.declare_parameter('timeout', 15)
        self.timout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id=self.get_parameter('vehicle_id').get_parameter_value().integer_value


    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        #host
        self.arm_vehicle_service = self.create_service(CommandBool, 'arm_vehicle', self.arm_vehicle_callback)
        self.go_to_point_service = self.create_service(Newpoint, 'go_to_point_command', self.go_to_point_callback)
        self.change_ASV_mode_service = self.create_service(ASVmode, 'change_asv_mode', self.change_asv_mode_callback)
        #client
        self.asv_mission_mode_client = self.create_client(ASVmode, 'change_mission_mode')



    def declare_topics(self):
        timer_period = 0.5  # seconds
        self.status_publisher = self.create_publisher(Status, 'status', 10)
        self.status_publisher_timer = self.create_timer(timer_period, self.status_publish)

    def __init__(self):
        # start the node
        super().__init__('Dronekit_node')

        # declare parameters
        self.parameters()
        self.status = Status()

        # connect to vehicle
        self.vehicle = connect(self.vehicle_ip, timeout=self.timout)
            #TODO raise error if there has been a timeout,
            #we can try to restart the dronekit node

        """        except ConnectionRefusedError:
            keep_going = False
            if verbose > 0:
                self.get_logger().fatal("Connection to navio2 could not be made")
        """
        self.get_logger().info(f"Connecting to vehicle in {self.vehicle_ip}")

        self.dictionary()

        # declare the services
        #self.declare_services()
        # start to pubblish

        self.declare_topics()
        self.declare_services()

    def arm_vehicle_callback(self, request, response):
        """
        Arming vehicle function. To arm, the vehicle must be in GUIDED mode. If so, the _vehicle.armed flag
        can be activated. The function waits for the vehicle to be armed.
        Args:
             _vehicle: `dronekit.connection object.
        """
        # Copter should arm in GUIDED mode

        try:
            if request.value:
                self.vehicle.mode = VehicleMode("GUIDED")
                self.vehicle.arm()
                self.get_logger().info('Vehicle armed')
                response.success=True
            else:
                self.vehicle.disarm()
                self.get_logger().info('Vehicle disarmed')
                response.success=True
        except:
            self.get_logger().error('Couldn\'t arm vehicle')
            response.success=False
        return response

    def status_publish(self):
        self.status.lat = self.vehicle.location.global_relative_frame.lat
        self.status.lon = self.vehicle.location.global_relative_frame.lon
        self.status.yaw = self.vehicle.attitude.yaw
        self.status.vehicle_id = self.vehicle_id
        self.status.battery = float(self.vehicle.battery.level)
        self.status.armed = self.vehicle.armed

        self.status_publisher.publish(self.status)


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


    def condition_yaw(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.
        By default the yaw of the vehicle will follow the direction of travel. After setting
        the yaw using this function there is no way to return to the default yaw "follow direction
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
        For more information see:
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            pymavlink.mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def reached_position(señf, current_loc, goal_loc):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        Args:
            current_loc: Actual position (dronekit.LocationGlobal).
            goal_loc: Reference position (dronekit.LocationGlobal).
        Returns:
            'True' if the ASV distance respecto to the target Waypoint is less than 1.5 meters.
        """

        # Convert to radians #
        lat1 = np.radians(current_loc.lat)
        lat2 = np.radians(goal_loc.lat)
        lon1 = np.radians(current_loc.lon)
        lon2 = np.radians(goal_loc.lon)

        # Obtains the latitude/longitude differences #
        d_lat = lat2 - lat1
        d_lon = lon2 - lon1

        # Returns True if the waypoint is within 1.5 meters the ASV position
        a = np.sin(0.5 * d_lat) ** 2 + np.sin(0.5 * d_lon) ** 2 * np.cos(lat1) * np.cos(lat2)
        c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
        return 6378100.0 * c < 0.5

    def change_asv_mode_callback(self, request, response):
        #string takes preference before int
        if len(request.asv_mode_str) != 0:
            if request.asv_mode_str in self.mode_type.values():
                mode=request.asv_mode_str
            else:
                self.get_logger().info(f"{request.asv_mode_str} is not a valid mode")
                response.success=False
                return response
        else:
            if request.asv_mode in self.mode_type:
                mode=self.mode_type[str(request.asv_mode)]
            else:
                self.get_logger().info(f"{request.asv_mode} is not a valid mode")
                response.success = False
                return response
        self.vehicle.mode = VehicleMode(mode)
        response.success=True
        return response
        #TODO: add other responses

    def go_to_point_callback(self, request, response):
        if self.vehicle.armed and self.vehicle.mode != VehicleMode("GUIDED"):
            self.get_logger().error(f'Error: vehicle should be armed and in guided mode\nbut arming is {vehicle.armed} and in {vehicle.mode}. \nSetting mode to Stand-by')
            msg=ASVmode.Request()
            msg.asv_mode=0
            self.call_service(self.asv_mission_mode_client, msg)
            response.success=False
            return response

        self.get_logger().info(f"Turning to : {self.get_bearing(self.vehicle.location.global_relative_frame, request.new_point)} N")
        self.condition_yaw(self.get_bearing(self.vehicle.location.global_relative_frame, request.new_point))
        time.sleep(2)
        self.vehicle.simple_goto(LocationGlobal(request.new_point.lat,request.new_point.lon,0.0))
        timer=0
        # Waits until the position has been reached.
        while not self.reached_position(self.vehicle.location.global_relative_frame, request.new_point) and timer<15:
            time.sleep(1)
            timer=timer+1
            #self.condition_yaw(self.get_bearing(self.vehicle.location.global_relative_frame, request.new_point))
            #TODO: fix infinite loop

        self.get_logger().info(f"Position Reached: {self.vehicle.location.global_relative_frame.lat}, {self.vehicle.location.global_relative_frame.lat} N")
        response.success=True
        return response

    def call_service(self, client,  msg):
        # TODO: raise error to avoid infinite wait if service is not up, after all means a module is not active $$ watchdog will be in charge
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {client.srv_name} not available, waiting again...', once=True)
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


    def dictionary(self):
        self.mode_type=dictionary("ASV_MODE")

def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    try:
        #start a class that servers the services
        dronekit_node = Dronekit_node()
        #loop the node
        rclpy.spin(dronekit_node)
    except:
        """
        There has been an error with the program, so we will send the error log
        """
        x = rclpy.create_node('dronekit_node') #we state what node we are
        #we need to make sure we are subscribed to logs, so we create a dummy publisher as subscriptions happen syncronously
        publisher = x.create_publisher(Log, 'rosout', 10) #we create the publisher
        while publisher.get_subscription_count() == 0: #while rosout is not up
            sleep(0.01) #we wait
        #we publish the error
        x.get_logger().fatal(traceback.format_exc())
    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()