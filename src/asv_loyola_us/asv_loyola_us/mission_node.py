#ros libraries
import rclpy #main librarie
from rclpy.node import Node #for defining a node
from rclpy.action import ActionClient #for defining actions


import os #to read paths
from .submodulos.KMLMissionGeneration import KMLMissionGenerator #for reading missions
from .submodulos.call_service import call_service #to call services
import traceback #for code errors
from time import sleep #delay

#custom interfaces
from asv_interfaces.srv import Newpoint, ASVmode, CommandBool
from asv_interfaces.msg import Status, Nodeupdate, String
from asv_interfaces.action import Goto

from action_msgs.msg import GoalStatus


class Mission_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('mission_filepath', "MisionesLoyola_dron_2.kml")
        path="~/ASV_Loyola_US/"+self.get_parameter('mission_filepath').get_parameter_value().string_value
        self.mission_filepath = os.path.expanduser(path)
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value


    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        #host
        self.samplepoint_service = self.create_service(Newpoint, 'new_samplepoint', self.new_samplepoint_callback)
        self.mission_mode_service = self.create_service(ASVmode, 'change_mission_mode', self.new_mission_mode)
        self.close_asv_service = self.create_service(CommandBool, 'close_asv', self.close_asv_callback)
        self.close_asv_service = self.create_service(CommandBool, 'load_mission', self.load_mission_callback)
        #client
        self.mqtt_send_info = self.create_client(CommandBool, 'MQTT_send_info')
        self.arm_vehicle_client = self.create_client(CommandBool, 'arm_vehicle')
        self.collect_sample_client = self.create_client(CommandBool, 'get_water_module_sample')
        self.change_asv_mode_client = self.create_client(ASVmode, 'change_asv_mode')
        self.go_to_point_client = self.create_client(Newpoint, 'go_to_point_command')

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.mission_mode_publisher = self.create_publisher(String, 'mission_mode', 10)
        self.mission_mode_publisher_timer = self.create_timer(1, self.mission_mode_publish)

    def declare_actions(self):
        self.goto_action_client = ActionClient(self, Goto, 'goto')

    def __init__(self):
        #start the node
        super().__init__('mission_node')

        #declare parameter of drone IP
        self.parameters()

        #declarations
        self.samplepoints=[] #list of waypoints to follow

        #declare the services
        self.declare_services()

        #declare topics
        self.declare_topics()

        self.declare_actions()

        #spin once so that declared things initialize
        rclpy.spin_once(self)

        #start the drone
        self.startup()
        self.get_logger().info('All systems operative')

        #loop main
        while rclpy.ok():
            rclpy.spin_once(self) #check if a topic has been published or if a timer aired
            self.main()
            sleep(0.1)  #we will run main each 1 second

    def startup(self):
        self.mission_mode = 0  # el modo del ASV deseado
        self.current_mission_mode = -1  # el modo del ASV actual

        self.mission_mode_strs = ["REST", "STANDBY", "PRELOADED_MISSION", "MANUAL", "RTL"]  # Strings para modos

        self.mqtt_waypoint = [] #store waypoint from Server
        self.status = Status() #Status of the robot

        self.get_logger().debug("Starting startup")
        self.get_logger().info(f"Starting mode:{self.mission_mode} {self.mission_mode_strs[self.mission_mode]}.")
        if self.DEBUG:
            self.get_logger().warning("Debug mode enabled")

        #TODO: Fix error due to path not existing
        self.mg = KMLMissionGenerator(self.mission_filepath)
        self.samplepoints = self.mg.get_samplepoints()

        #this is a workaround, if it returns false means vehicle is not on due to code flow
        self.get_logger().debug("Connecting to Vehicle")
        if not self.arm_vehicle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().fatal('vehicle node is not answering')
            #TODO raise error if there has been a timeout,

        #TODO: wait for all systems operative from vehicle
        #     This includes:
        #           Sensor module
        if not self.mqtt_send_info.wait_for_service(timeout_sec=1.0):
            self.get_logger().fatal('mqtt node is not answering')
        msg=CommandBool.Request()
        msg.value=True
        call_service(self, self.mqtt_send_info, msg)



    """
    This function automatically runs in loop at 1 Hz
    we MUST avoid spin_until_future_complete
    """
    def main(self):
        if self.mission_mode == 0:  # Rest
            if self.change_current_mission_mode(self.mission_mode):
                self.get_logger().info("vehicle resting.")
            if self.status.armed:
                self.arm_vehicle(False)
                self.get_logger().info("Vehicle was armed! Stoping the vehicle.")

        elif self.mission_mode == 1:  # Stand_By
            if self.change_current_mission_mode(self.mission_mode):
                self.arm_vehicle(True)
                self.change_ASV_mode("LOITER")
                self.get_logger().info("vehicle armed, indicate point to go in \"/go_to_command\" service")
                self.get_logger().info("vehicle in \'STANDBY\' mode")

        elif self.mission_mode == 2:  # Pre-loaded Mission
            #check if we come from other mode
            if self.change_current_mission_mode(self.mission_mode):
                #set ASV mode to Loiter
                self.change_ASV_mode("LOITER")
                #check if we have a mission to follow, go to rest if not
                if len(self.samplepoints) == 0:
                    self.get_logger().info("no preloaded mission, call \"/load_mission\" service")
                    self.mission_mode = 0 #return vehicle to
                else:
                    self.arm_vehicle(True)
                    self.get_logger().info(f"Starting Pre-loaded Mission {self.mission_filepath}")
                    self.waiting_for_action = False
            else: #we have a mission to follow, so we enter this part of the code
                if len(self.samplepoints) == 0: #first check if mission is finished
                    self.get_logger().info(f"Finished preloaded mission.\nSetting mode to Stand-by.")
                    self.mission_mode = 0
                elif self.waiting_for_action: #check if we are waiting to reach a new point
                    pass
                else: #go to the next point
                    self.go_to(self.get_next_wp())

        elif self.mission_mode == 3:  # Manual Mode
            if self.change_current_mission_mode(self.mission_mode):
                self.arm_vehicle(True)
                self.change_ASV_mode("MANUAL")
                self.get_logger().info(f"vehicle in \'MANUAL\' mode")

        elif self.mission_mode == 4:  # RTL
            if self.change_current_mission_mode(self.mission_mode):
                self.arm_vehicle(True)
                self.change_ASV_mode("RTL")
                self.get_logger().info("vehicle in \'RTL\' mode")
        else:
            #raise an error, inconsistent mode
            self.get_logger().fatal("we reached an inconsistent mode")

        #TODO: en misiones mas grandes la mision debe finalizar si vehicle.battery.level < 0.6
        # if self.status.battery <0.6:
        #    self.main_loop.destroy()
        #TODO: Es necesario implementar una subruntina de acción para volver a home




    def new_samplepoint_callback(self, request, response):
        #TODO: Testear funcionamiento

        """
        this function starts when a new samplepoint is received from MQTT or elsewhere
        it appends a new samplepoint to the samplepoint list
        it sends back the samplepoints list
        """
        self.get_logger().info('new waypoint received: ', request.new_point)
        if self.current_mission_mode == 1:
            self.samplepoints.append(request.new_point)
            response.point_list = self.samplepoints
        elif self.current_mission_mode == 3:
            self.mqtt_waypoint=request.new_point
            response.point_list = request.new_point
        return response

    def new_mission_mode(self, request, response):
        #TODO: Testear funcionamiento

        """
        this function changes the ASV Mode if the value is between limits
        """
        #check if mode is between limits
        if request.asv_mode > 4 or request.asv_mode < 0:
            response.success = False
        else:
            response.previous_mode = self.current_mission_mode
            self.mission_mode = request.asv_mode
            response.success = True
        return response

    def get_next_wp(self):
        #TODO: Implementar, esto va a estar mezclado con planner y MQTT, pero es necesario saber que se va a enviar
        #TODO: Esta funcion se va a eliminar
        """
        Receives the vehicle object and pop the next waypoint. If `current_mission_mode is 1 (preloaded mission), pops the next
        mission waypoint. If `current_mission_mode is 3 (simple go-to), pop the position of the received mqtt wwaypoint.
        Args:
            _vehicle: The connection vehicle object from `dronekit.
        Returns:
            Returns the next waypoint as a `dronekit.LocationGlobal object.
        """

        # global received_mqtt_wp  solo en escritura es útil poner una variable como global, para lectura no es necesario
        # https://stackoverflow.com/questions/423379/using-global-variables-in-a-function

        if self.current_mission_mode == 1:  # Preloaded
            nextwp = self.received_mqtt_wp

        elif self.current_mission_mode == 2:
            nextwp = self.samplepoints.pop(0)
        else:
            self.get_logger().fatal(f"Current ASV Mode should be 1: {self.mission_mode_strs[1]} or 2: {self.mission_mode_strs[2]} but it is {self.current_mission_mode}: {self.mission_mode_strs[self.current_mission_mode]}")
            raise ValueError(f"Current ASV Mode should be 1: {self.mission_mode_strs[1]} or 2: {self.mission_mode_strs[2]} but it is {self.current_mission_mode}: {self.mission_mode_strs[self.current_mission_mode]}")
        self.get_logger().info(f"Next waypoint is {nextwp}" )
        return nextwp



            
    def change_current_mission_mode(self, desired_mode):
        """
        Changes the ASV mode safelly
        Args:
            desired_mode: int` with the desired mode to change.
        Returns:
            Returns a boolean flag to indicate whether the mode has changed to the desired one or not.
        """
        #TODO: Arreglar esta funcion para que se hagan referencia unos a otros

        if self.current_mission_mode != desired_mode:
            self.current_mission_mode = desired_mode
            self.get_logger().info(f"Changed current ASV mode to {self.mission_mode_strs[self.current_mission_mode]}.")
            return True
        return False

    def status_suscriber_callback(self, msg):
        self.status = msg

    def close_asv_callback(self, request, response):
        self.get_logger().info("FINISHING MISSION")
        #disarm vehicle
        #TODO: check if arming failed
        self.arm_vehicle(False)
        #TODO: vehicle.close()
        return True


    def arm_vehicle(self, value):
        aux=CommandBool.Request()
        aux.value=value
        self.get_logger().debug("asked for ASV arm." if value else "asked for ASV disarm")
        call_service(self, self.arm_vehicle_client, aux)
        return True


    def load_mission_callback(self, request, response):
        if len(request.file_name == 0): #if string is empty load mission
            self.samplepoints = self.mg.get_samplepoints()
            response.success = True
        else: #load indicated mission
            try:
                self.mission_filepath = os.path.expanduser("~/ASV_Loyola_US/"+request.file_name)
                self.mg = KMLMissionGenerator(self.mission_filepath)
                self.samplepoints = self.mg.get_samplepoints()
                response.success = True
            except:
                response.success = False
        return response

    def change_ASV_mode(self, mode):
        #TODO: Establecer diferenciacion con los modos de ardupilot, se recomienda usar string
        #TODO: protección si se indica un modo fuera de rango
        aux = ASVmode.Request()
        if type(mode)=='int':
            aux.asv_mode = mode
            self.get_logger().debug(f"asked to change asv to mode: {[mode]}")
            call_service(self, self.change_asv_mode_client, aux)
        else:
            aux.asv_mode_str = mode
            self.get_logger().debug(f"asked to change asv to mode: {mode}")
            call_service(self, self.change_asv_mode_client, aux)

    def go_to(self, location):
        self.get_logger().info(f"going to {location}")
        self.goto_action_client.wait_for_server()
        self.waiting_for_action=True
        goal_msg = Goto.Goal()
        goal_msg.samplepoint = location
        self.get_logger().debug('Sending goal request...')
        self._send_goal_future = self.goto_action_client.send_goal_async(goal_msg, feedback_callback=self.goto_feedback_callback)
        self._send_goal_future.add_done_callback(self.go_to_response)

    def goto_feedback_callback(self, feedback):
        distance= feedback.feedback.distance #TODO: unused
        self.get_logger().info(f"distance={distance}")

    def go_to_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.waiting_for_action=False
            #TODO: decide what to do if goal is rejected, in other words, action busy
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.go_to_finished)

    def go_to_finished(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.success))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
        self.waiting_for_action=False

    #TODO: add a cancel to the action function if ever necessary

    def mission_mode_publish(self):
        msg=String()
        try:
            msg.string=self.mission_mode_strs[self.mission_mode]
            self.mission_mode_publisher.publish(msg)
        except:
            pass


def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    #start a class that servers the services
    try:
        mission_node = Mission_node()
    except:
        """
        There has been an error with the program, so we will send the error log to the watchdog
        """
        x = rclpy.create_node('mission_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "mission_node" #our identity
        msg.message = traceback.format_exc() #the error
        #to be sure the message reaches, we must wait till wathdog is listening (publisher needs time to start up)
        #TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
        # este topic está oculto en echo al usar _
        while publisher.get_subscription_count() == 0: #while no one is listening
            sleep(0.01) #we wait
        publisher.publish(msg) #we send the message
        x.destroy_node() #we destroy node and finish

    #after close connection shut down ROS2
    rclpy.shutdown()




if __name__ == '__main__':
    main()
