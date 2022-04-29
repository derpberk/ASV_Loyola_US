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
from asv_interfaces.srv import Newpoint, ASVmode, CommandBool, LoadMission
from asv_interfaces.msg import Status, Nodeupdate, String, Location
from asv_interfaces.action import Goto

from action_msgs.msg import GoalStatus


class Mission_node(Node):

    #this functions defines and assigns value to the variables defined in /config/config.yaml
    def parameters(self):
        self.declare_parameter('mission_filepath', "MisionesLoyola_dron_2.kml")
        path="~/ASV_Loyola_US/"+self.get_parameter('mission_filepath').get_parameter_value().string_value
        self.mission_filepath = os.path.expanduser(path)
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value
        self.declare_parameter('mqtt_point_timeout', 15)
        self.mqtt_point_timeout = self.get_parameter('mqtt_point_timeout').get_parameter_value().integer_value


    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        #host services
        self.samplepoint_service = self.create_service(Newpoint, 'new_samplepoint', self.new_samplepoint_callback)
        self.mission_mode_service = self.create_service(ASVmode, 'change_mission_mode', self.new_mission_mode)
        self.close_asv_service = self.create_service(CommandBool, 'close_asv', self.close_asv_callback)
        self.load_mission_service = self.create_service(LoadMission, 'load_mission', self.load_mission_callback)
        self.cancel_movement_service = self.create_service(CommandBool, 'cancel_movement', self.cancel_movement_callback)

        #client services
        self.mqtt_send_info = self.create_client(CommandBool, 'MQTT_send_info')
        self.arm_vehicle_client = self.create_client(CommandBool, 'arm_vehicle')
        self.collect_sample_client = self.create_client(CommandBool, 'get_water_module_sample')
        self.change_asv_mode_client = self.create_client(ASVmode, 'change_asv_mode')
        self.go_to_point_client = self.create_client(Newpoint, 'go_to_point_command')

    #this function delcares the topics
    def declare_topics(self):
        #subscriptions
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.mission_mode_publisher = self.create_publisher(String, 'mission_mode', 10)
        
        #publishers
        self.mission_mode_publisher_timer = self.create_timer(1, self.mission_mode_publish) #timer 1 Hz
        self.destination_publisher = self.create_publisher(Location, 'destination', 10)

    #this function declares the actions
    def declare_actions(self):
        self.goto_action_client = ActionClient(self, Goto, 'goto') #action client
        self.waiting_for_action = False                            #variable to indicate the action is executing (True) or not (False)

    #this function will be the first one executed
    def __init__(self):
        #start the node with name 'mission_node'
        super().__init__('mission_node')

        #declare parameter of drone IP
        self.parameters()

        #declare the services
        self.declare_services()

        #declare topics
        self.declare_topics()

        #declare the actions
        self.declare_actions()

        #spin once so that declared things initialize
        rclpy.spin_once(self)

        #start the drone
        self.startup()
        self.get_logger().info('All systems operative')
        self.get_logger().info(f"Drone running version {self.version()}")

        #ask mqtt to start sending once everything is operative
        call_service(self, self.mqtt_send_info, CommandBool.Request())

        #loop main
        while rclpy.ok():
            rclpy.spin_once(self) #check if a topic has been published or if a timer aired
            self.main()
            sleep(0.1)  #we will run main each 1 second

    """This function is in charge of the initialization fo the drone, it must take into account
    - Initialization of variables

    
    
    """
    def startup(self):

        self.mission_mode = 0  # desired ASV mission mode
        self.current_mission_mode = -1  # actual ASV mission mode
        self.samplepoints=[] #list of points where we want to take a sample
        self.mission_mode_strs = ["LAND", "STANDBY", "PRELOADED_MISSION", "MANUAL", "SIMPLE POINT", "RTL", "MANUAL_INTERRUPTION"]  # Strings for mission mode names

        self.mqtt_waypoint = None #store samplepoint from Server
        self.status = Status() #Status of the robot, this includes: (arm_status, location, asv_mode, vehicle_id), will be updated in self.status_suscriber_callback each 0.5 seconds
        
        self.get_logger().debug("Starting startup")
        self.get_logger().info(f"Starting mode:{self.mission_mode} {self.mission_mode_strs[self.mission_mode]}.")

        if self.DEBUG:
            self.get_logger().warning("Debug mode enabled")

        #Preload mission specified in config.yaml
        try:
            self.mg = KMLMissionGenerator(self.mission_filepath)
            self.samplepoints = self.mg.get_samplepoints()
        except:
            self.get_logger().error("Preloaded mission doesnt exist, no mission will be preloaded")


        #this is a workaround, if it returns false means vehicle is not on due to code flow
        self.get_logger().debug("Connecting to Vehicle")
        if not self.arm_vehicle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().fatal('vehicle node is not answering', once=True)
            #INFO: we could use a while instead of an if to wait till it is on, but it may become an infinite loop

        #same workaround for MQTT
        while not self.mqtt_send_info.wait_for_service(timeout_sec=10.0):
            self.get_logger().fatal('mqtt node is not answering')


    """
    This function automatically runs in loop at 1 Hz
    we MUST avoid spin_until_future_complete
    This function contains all the modes
    """
    def main(self):
        #LAND mode
        #This mode disarms the robot, and throws a warning if it has been armed externally
        if self.mission_mode == 0:
            if self.change_current_mission_mode(self.mission_mode): #the contents of this if will only be executed once
                self.get_logger().info("vehicle resting.")
                self.change_ASV_mode("MANUAL")
                self.arm_vehicle(False)
            if self.status.manual_mode:
                self.mission_mode=6
            if self.status.armed:
                self.get_logger().warning("Vehicle was armed externally! .", once=True)
        

        #TODO: first objective was to disarm the vehicle for safety if it is armed, however we cannot arm the vehicle in this mode, and that
        # made people freak, so no safety mode that forces vehicle off, just a mode that warns


        # STANDBY mode
        # This mode puts ASV in LOITER mode and arms it, vehicle will remain in the same position
        elif self.mission_mode == 1:  
            if self.change_current_mission_mode(self.mission_mode): #the contents of this if will only be executed once
                if not self.status.ekf_ok: #if vehicle has EKF problems go back to manual and do not enter the new mode
                        self.get_logger().info("The vehicle is not able to go into automatic mode")
                        self.mission_mode=0
                else:
                    self.change_ASV_mode("LOITER")
                    self.arm_vehicle(True)
                    self.get_logger().info("vehicle in \'STANDBY\' mode")
            if self.status.manual_mode:
                self.mission_mode=6
            


        # Pre-loaded Mission mode
        # This mode will start a mission following the points in the list self.samplepoints, once that list is empty it will go back to standby
        elif self.mission_mode == 2:  
            if self.change_current_mission_mode(self.mission_mode): #the contents of this if will only be executed once
                #set ASV mode to Loiter
                if not self.status.ekf_ok: #if vehicle has EKF problems go back to manual and do not enter the new mode
                    self.get_logger().info("The vehicle is not able to go into automatic mode")
                    self.mission_mode=0
                #check if we have a mission to follow, go to STANDBY if not
                elif len(self.samplepoints) == 0:
                    self.get_logger().info("no preloaded mission, call \"/load_mission\" service")
                    self.mission_mode = 1 #return vehicle to
                else:
                    self.change_ASV_mode("LOITER")
                    self.arm_vehicle(True)
                    self.get_logger().info(f"Starting Pre-loaded Mission {self.mission_filepath}")
            else: #we arrive here, so we have a mission to follow
                if self.waiting_for_action: #check if we are waiting to reach a new point
                    pass
                elif len(self.samplepoints) == 0: #check if mission is finished, as this is a loop, change to STANDBY mode if thats the case
                    self.get_logger().info(f"Finished preloaded mission.\nSetting mode to Stand-by.")
                    self.mission_mode = 1
                else: #go to the next point
                    self.go_to(self.get_next_wp())

        # Manual Mode
        #This mode changes ASV mode to MANUAL and arms the vehicle
        elif self.mission_mode == 3:  
            if self.change_current_mission_mode(self.mission_mode):#the contents of this if will only be executed once
                self.change_ASV_mode("MANUAL")
                self.arm_vehicle(True)
                self.get_logger().info(f"vehicle in \'MANUAL\' mode")

        # Simple GO-TO
        #This mode is basically STANDBY
        # but it administrates a buffer of 1 MQTT message due to code flow
        elif self.mission_mode == 4: 
            if self.change_current_mission_mode(self.mission_mode):#the contents of this if will only be executed once
                if not self.status.ekf_ok: #if vehicle has EKF problems go back to manual and do not enter the new mode
                    self.get_logger().info("The vehicle is not able to go into automatic mode")
                    self.mission_mode=0
                else:
                    timeout_counter=0
                    self.change_ASV_mode("LOITER")
                    self.arm_vehicle(True)
                    self.get_logger().info("vehicle in \'SIMPLE POINT\' mode")
            if self.mqtt_waypoint is not None and not self.waiting_for_action: #if we have a point and we are not busy
                self.go_to(self.mqtt_waypoint) #go to the point
                self.mqtt_waypoint = None #discard the point
                #TODO: may be, implement a higher buffer for points
            if self.waiting_for_action == False and self.mqtt_waypoint == None:
                timeout_counter += 1
                if timeout_counter > self.mqtt_point_timeout*10:
                    self.mission_mode = 1
                    self.get_logger().info(f"No point in { self.mqtt_point_timeout} seconds, going into standby mode")
            else:
                timeout_counter=0





        # RTL mode
        #This mode will arm the vehicle and set RTL mode
        #TODO: this mode is supposed to make ASV return to home, but we have not yet checked if it works
        elif self.mission_mode == 5: 
            if not self.status.ekf_ok: #if vehicle has EKF problems go back to manual
                    self.get_logger().info("The vehicle lost GPS reference")
                    self.mission_mode=0
            if self.status.manual_mode: #manual takes preference over anything else
                self.mission_mode=6
            elif self.change_current_mission_mode(self.mission_mode):#the contents of this if will only be executed once
                self.arm_vehicle(True)
                self.change_ASV_mode("RTL")
                self.get_logger().info("vehicle in \'RTL\' mode")

        # Manual interruption
        #This mode will log and wait for RC to resume previous mode
        #TODO: this mode is supposed to make ASV return to home, but we have not yet checked if it works
        elif self.mission_mode == 6: 
            #store mission mode
            if self.current_mission_mode!=self.mission_mode:
                self.last_mode=self.current_mission_mode

            if not self.status.ekf_ok: #if vehicle has EKF log
                self.get_logger().info("The vehicle lost EKF", once=True)

            if self.change_current_mission_mode(self.mission_mode):#the contents of this if will only be executed once
                self.get_logger().info("vehicle has been manually interrupted by RC, switching to Manual mode")

            elif not self.status.manual_mode:
                self.get_logger().info("vehicle in auto mode, recovering last mode")
                self.mission_mode=self.last_mode
        
        #TODO: raise an error, inconsistent mode
        else:
            self.get_logger().fatal("we reached an inconsistent mode")
            self.mission_mode=0

        #TODO: en misiones mas grandes la mision debe finalizar si vehicle.battery.level < 0.6
        # if self.status.battery <0.6:



    """
    Callback from topic /new_samplepoint
    This function recieves a new samplepoint and stores it if
    - mission_mode="PRELOADED MISSION"    --> appends it to the list of points
    - mission_mode="SIMPLEPOINT"          --> overrides mqtt_waypoint value

    @parameters
        -newpoint: Location: struct with lon and lat values (float64)
    @returns
        -point_list: Location[]: List of points the submarine has as destinations 
    """
    #TODO: this will work better if we can somehow save missions, as it is used mainly for MQTT

    def new_samplepoint_callback(self, request, response):
        self.get_logger().info(f'new waypoint received: {request.new_point}')
        if self.current_mission_mode == 2: #'PRELOADED MISSION' mode
            self.samplepoints.append(request.new_point)
            response.point_list = self.samplepoints
        elif self.current_mission_mode == 4: #'SIMPLE POINT' mode
            self.mqtt_waypoint = request.new_point
            response.point_list = [self.mqtt_waypoint]
        return response


    """
    Callback from topic /change_mission_mode
    This function changes current mission mode to the one requested if it is between limits
    @parameters
        -asv_mode: (int): desired mode
    @returns
        -success: (Bool): True if success, false otherwise 
    """
    def new_mission_mode(self, request, response):

        if len(request.asv_mode_str) != 0:
            if request.asv_mode_str in self.mission_mode_strs:
                self.mission_mode=self.mission_mode_strs.index(request.asv_mode_str)
                if self.waiting_for_action: #if the action was active
                    try:
                        self.get_logger().info("the asv was moving, canceling movement")
                        self.goal_handle.cancel_goal_async()
                        if  self.current_mission_mode == 2: #if mission, restore the point
                            self.samplepoints.insert(0,self.point_backup)
                    except:
                        pass
                return response
            else:
                self.get_logger().info(f"{request.asv_mode_str} is not a valid mode")
                response.success=False
                return response

        #check if mode is between limits
        if request.asv_mode > len(self.mission_mode_strs) or request.asv_mode < 0:
            response.success = False
        else:
            response.previous_mode = self.current_mission_mode
            self.mission_mode = request.asv_mode
            response.success = True
            if self.waiting_for_action: #if the action was active
                try:
                    self.get_logger().info("the asv was moving, canceling movement")
                    self.goal_handle.cancel_goal_async()
                    if  self.current_mission_mode == 2: #if mission, restore the point
                        self.samplepoints.insert(0,self.point_backup)
                except:
                    pass
        return response


    """
    This function pops the next waypoint, only used in premission, it should be deprecated as it always enters the first if
    @returns
        -nextwp: Location: next point in the mission list
    """
    #TODO: check if list is empty
    def get_next_wp(self):

        if self.current_mission_mode == 2:
            nextwp = self.samplepoints.pop(0)
        else:
            self.get_logger().fatal(f"Current ASV Mode should be 2: {self.mission_mode_strs[2]} but it is {self.current_mission_mode}: {self.mission_mode_strs[self.current_mission_mode]}")
            raise ValueError(f"Current ASV Mode should be 2: {self.mission_mode_strs[2]} but it is {self.current_mission_mode}: {self.mission_mode_strs[self.current_mission_mode]}")
        self.get_logger().debug(f"Next samplepoint is {nextwp}" )
        return nextwp


    """
        Changes the ASV mission mode safelly
        Args:
            desired_mode: int` with the desired mode to change.
        Returns:
            Returns a boolean flag to indicate whether the mode has changed to the desired one or not.
    """        
    def change_current_mission_mode(self, desired_mode):
        
        #TODO: Arreglar esta funcion para que se hagan referencia unos a otros

        if self.current_mission_mode != desired_mode:
            self.current_mission_mode = desired_mode
            self.get_logger().info(f"Changed current ASV mode to {self.mission_mode_strs[self.current_mission_mode]}.")
            return True
        return False


    
    """
    Callback from topic /status
    This function stores the status of the robot in the status variable
    """

    def status_suscriber_callback(self, msg):
        self.status = msg



    """
        This function is a callback from service /close_asv
        this function is not used, it should be better developed
        This functions turns off the ASV safelly, killing the node
        Args:
            bool always true to turn off the vehicle
        Returns:
            Bool indicating success of the action
    """       


    def close_asv_callback(self, request, response):
        self.get_logger().info("FINISHING MISSION")
        #disarm vehicle
        #TODO: check if arming failed
        self.arm_vehicle(False)
        #TODO: vehicle.close()
        return True

    """
    This function calls service /arm_vehicle arming or disarming the vehicle
    @parameters
        -value: (bool):True for arming, False for disarming
    @returns
        - always true
    """

    def arm_vehicle(self, value):
        aux=CommandBool.Request()
        aux.value=value
        self.get_logger().debug("asked for ASV arm." if value else "asked for ASV disarm")
        call_service(self, self.arm_vehicle_client, aux)
        return True


    """
    Callback from service /load_mission
    This function loads the mission from a file if it exists
    @parameters
        -file_name: (str): name of the file (it must be a number)
    @returns
        -success: (Bool): True if success, false otherwise 
    """

    def load_mission_callback(self, request, response):
        self.get_logger().info(f"asked to load mission {request.file_name}")
        if request.file_name == "": #if string is empty load last mission
            self.samplepoints = self.mg.get_samplepoints()
            response.success = True
        else: #load indicated mission
            try:
                self.mission_filepath = os.path.expanduser("~/ASV_Loyola_US/MisionesLoyola_dron_"+request.file_name+".kml")
                self.mg = KMLMissionGenerator(self.mission_filepath)
                self.samplepoints = self.mg.get_samplepoints()
                response.success = True
                self.get_logger().info(f"mission: MisionesLoyola_dron_"+request.file_name+".kml"+" was loaded successfully")
            except:
                response.success = False
                self.get_logger().info(f"mission: MisionesLoyola_dron_{request.file_name}.kml doesn't exist in database")
        return response

    """
        Changes the ASV mode safelly
        Args:
            mode: int or string with the desired mode to change. You can check int values in /submodulos/dictionary.py
    """     

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


    """
        This function starts the call to the action /goto
    """     
    def go_to(self, location):
        self.get_logger().info(f"going to [{location.lat},{location.lon}]")
        self.destination_publisher.publish(location)
        self.goto_action_client.wait_for_server()
        self.waiting_for_action=True
        self.point_backup=location
        goal_msg = Goto.Goal()
        goal_msg.samplepoint = location
        self.get_logger().debug('Sending goal request...')
        self._send_goal_future = self.goto_action_client.send_goal_async(goal_msg, feedback_callback=self.goto_feedback_callback)
        self._send_goal_future.add_done_callback(self.go_to_response)

    """
        This function prints the feedback from the goto action
    """     
    def goto_feedback_callback(self, feedback):
        distance= feedback.feedback.distance #TODO: unused
        self.get_logger().info(f"distance={distance}")

    """
        This function is the first answer from the action service, indicates whether the request is accepted or rejected
    """   
    def go_to_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected :(, something is not working')
            if self.status.manual_mode:
                self.get_logger().info("Manual interruption, killing mission handler")
                #restore the point #TODO: check the point comes from mission and not simplepoint
                self.mission_mode=6 #go to manual
            elif self.mission_mode == 2 or self.mission_mode == 4:#mission was rejected, but we want to do a mission, probably drone started in a wrong state, or it is busy
                if self.status.armed:
                    self.get_logger().info('Vehicle in wrong state, trying to recover')
                    self.change_ASV_mode("LOITER")
                    self.get_logger().info(f'asking again to go to {self.point_backup}')
                    sleep(2) #sleep to avoid spamming points
                    goal_msg = Goto.Goal()
                    goal_msg.samplepoint = self.point_backup
                    self._send_goal_future = self.goto_action_client.send_goal_async(goal_msg, feedback_callback=self.goto_feedback_callback)
                    self._send_goal_future.add_done_callback(self.go_to_response)
                else:
                    self.mission_mode=0
                    self.get_logger().error('Vehicle disarmed, going to REST mode')
                    #TODO: add timeout in case ekf fails for too long
            else:
                self.get_logger().info(f'Goal was rejected, unknown state {self.mission_mode}')
            self.waiting_for_action=False
            if self.current_mission_mode == 2:
                self.get_logger().info(f'restoring point')
                self.samplepoints.insert(0,self.point_backup)
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.go_to_finished)


    """
        This function prints the result once the action finishes
    """   
    def go_to_finished(self, future):
        result = future.result().result #this variable is the result specified in the action
        status = future.result().status # ABORT = 4     CANCELED = 5    CANCEL_GOAL = 2     EXECUTE = 1     SUCCEED = 3     NOT_SPECIFIED = 6
        status_string= ["0 IS NOT AN STATE", "EXECUTE", "CANCEL_GOAL", "SUCCEEDED", "ABORT", "CANCELED", "NOT SPECIFIED"]
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.success))
        else:
            self.get_logger().info(f'Goal failed with status: {status_string[int(status)]}')
            self.get_logger().info(f'reason : {result.finish_flag}')
            if self.mission_mode == 2: # restore the point
                self.samplepoints.insert(0,self.point_backup)
                self.get_logger().info(f'point restored')
            if result.finish_flag == "Manual interruption":
                self.mission_mode = 6
                self.get_logger().info("Manual interruption, going into MANUAL mode")
            else:
                self.mission_mode = 1
                self.get_logger().info("going into STANDBY mode")

            
            
        self.waiting_for_action=False #we indicate the state machine that action finished

    """
        This function publishes the mission mode in the /mission_mode topic
        it is executed each second
    """   
    def mission_mode_publish(self):
        msg=String()
        try:
            msg.string=self.mission_mode_strs[self.mission_mode]
            self.mission_mode_publisher.publish(msg)
        except:
            pass

    """
        This function is a callback from the service /cancel_movement
        This function forces the stop of the drone and deletes the destination point, returning the drone to standby mode
    """   
    def cancel_movement_callback(self, request, response):
        self.get_logger().info("asked to stop the vehicle")
        if self.waiting_for_action: #if the action was active
            self.get_logger().info("the asv was moving, canceling movement")
            self.goal_handle.cancel_goal_async()
        else:
            self.get_logger().info("vehicle was already stoped, nothing to do")
        response.success=True
        return response
            

    def version(self):
        version="unknown"
        try:
            with open('/home/xavier/ASV_Loyola_US/version.txt', 'r') as f:
                for line in f:
                    line=line.strip().replace(" ", "").split(":")
                    version = str(line[1])
                    break
        except:
            pass
        return version

def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    #start a class that servers the services
    try:
        mission_node = Mission_node()
        mission_node.destroy_node()
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
