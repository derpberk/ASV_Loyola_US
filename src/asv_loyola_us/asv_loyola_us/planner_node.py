import rclpy
from rclpy.node import Node
import traceback
from asv_interfaces.srv import Newwaypoint, ASVmode, CommandBool
from asv_interfaces.msg import Status, Nodeupdate
from asv_interfaces.action import Samplepoint
#TODO: Everything

class Planner_node(Node):

    # his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('mission_filename', '1')
        self.mission_filename = self.get_parameter('mission_filename').get_parameter_value().string_value
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

    # this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        # host
        self.samplepoint_service = self.create_service(Newwaypoint, 'new_samplepoint', self.new_samplepoint_callback)
        self.mission_mode_service = self.create_service(ASVmode, 'change_mission_mode', self.new_mission_mode)
        self.close_asv_service = self.create_service(CommandBool, 'close_asv', self.close_asv_callback)
        # client
        self.mqtt_send_info = self.create_client(CommandBool, 'MQTT_send_info')
        self.arm_vehicle_client = self.create_client(CommandBool, 'arm_vehicle')
        self.collect_sample_client = self.create_client(CommandBool, 'get_water_module_sample')
        self.change_asv_mode_client = self.create_client(ASVmode, 'change_asv_mode')

    def declare_actions(self):
        self.go_to_server = rclpy.action.ActionServer(self, Samplepoint, 'fibonacci', self.go_to_callback)

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        # TODO: Topic que publique el estado de la mision para lectura de datos

    def __init__(self):
        # start the node
        super().__init__('mission_node')

        # declare parameter of drone IP
        #self.parameters()


        # declare the services
        #self.declare_services()

        # declare actions
        self.declare_actions()


    def go_to_callback(self, goal_handle):
        feedback_msg = Samplepoint.Feedback()
        result = Samplepoint.Result()
        self.get_logger().info('Creating new path')
        path = self.get_path(goal_handle.request.samplepoint)

        #TODO
        # while not reached position()
        # llamar action control
        # cerrar action si cambia el path debido a deteccion de obstaculos

        #goal_handle.publish_feedback(feedback_msg)


        #TODO: after reaching
        goal_handle.succeed()
        result.success = True
        return result


    def get_path(self, goal):
        #TODO: everything
        return goal

    def reached_position(current_loc, goal_loc):
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

    def get_bearing(location1, location2):
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

    def move2wp():
        """
        Function for moving to the next wp. This function should only be called in mode 1 or 3 (Preloaded mission / simplegoto)
        because it uses `get_next_wp function.
        Returns:
            True when finished, False when it is not possible to move or change the mode.
        """
        global asv_mode

        # Arm the vehicle if needed #
        if not vehicle.armed:
            arm(vehicle)
        # If the vehicle cannot be armed or the autopilot mode is not GUIDED, raise a Warning and returns False.
        # The mode must be guided always to move to the next waypoint.
        if not vehicle.armed or vehicle.mode != VehicleMode("GUIDED"):
            if verbose > 0:
                print("Error: vehicle should be armed and in guided mode")
                print(f"but arming is {vehicle.armed} and in {vehicle.mode}.")
                print("Setting mode to Stand-by")
            asv_mode = 0
            return False

        # When the pre-requisites of armability and the correct mode are setted, obtain the next waypoint.
        # Depending on the mode, obtained from the preloaded mission or from the MQTT broker.
        point2go = get_next_wp(vehicle)

        # Throw some information if specified the verbose condition
        if verbose > 0:
            print("Turning to : ", get_bearing(vehicle.location.global_relative_frame, point2go), "N")
        condition_yaw(get_bearing(vehicle.location.global_relative_frame, point2go))
        time.sleep(2)

        # MOVE!
        vehicle.simple_goto(point2go)

        # Waits until the position has been reached.
        while not reached_position(vehicle.location.global_relative_frame, point2go):
            time.sleep(1)
            continue

        # Once the position has been reached, change the autopilot mode to LOITER to maintain actual position (disturbance
        # rejection)
        vehicle.mode = VehicleMode("LOITER")

        # Throw some information about the sampling
        if verbose > 0:
            if current_asv_mode == 1:
                print("TOMANDO MUESTRAS, quedan: ", len(waypoints), "waypoints")
            else:
                print("TOMANDO MUESTRAS")

        if DEBUG:
            time.sleep(3)
        else:
            # If not in Debugging, take a sample using the Sensor Module#
            position = vehicle.location.global_relative_frame

            if SENSOR:
                reads = modulo_de_sensores.take_a_sample(position=[position.lat, position.lon], num_of_samples=3)
                for read in reads:
                    mqtt.send_new_msg(json.dumps(read), "database")  # Send the MQTT message
                    time.sleep(0.1)
            else:
                time.sleep(1.5)  # Sleep for a second

        vehicle.mode = VehicleMode("GUIDED")  # Return to GUIDED to pursue the next waypoint

        time.sleep(1)  # Wait a second to be sure the vehicle mode is changed.

        return True


    def condition_yaw(heading, relative=False):
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
        msg = vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        vehicle.send_mavlink(msg)

    def status_suscriber_callback(self, msg):
        self.status = msg


def main(args=None):
    # init ROS2
    rclpy.init(args=args)
    try:
        # start a class that servers the services
        planner_node = Planner_node()
        # loop the services
        rclpy.spin(planner_node)
    except:
        """
        There has been an error with the program, so we will send the error log to the watchdog
        """
        x = rclpy.create_node('planner_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, 'internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "planner_node" #our identity
        msg.message = traceback.format_exc() #the error
        #to be sure the message reaches, we must wait till wathdog is listening (publisher needs time to start up)
        #TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
        while publisher.get_subscription_count() == 0: #while no one is listening
            sleep(0.01) #we wait
        publisher.publish(msg) #we send the message
        x.destroy_node() #we destroy node and finish
    # after close connection shut down ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
