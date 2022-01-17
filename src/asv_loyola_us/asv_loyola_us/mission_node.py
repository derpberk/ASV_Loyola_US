import rclpy
from rclpy.node import Node
from .submodulos.KMLMissionGeneration import KMLMissionGenerator
import traceback
from time import sleep
from asv_interfaces.srv import Newpoint, ASVmode, CommandBool
from asv_interfaces.msg import Status, Nodeupdate
import os
import time
from .submodulos.call_service import call_service

class Mission_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('mission_filepath', os.path.expanduser("~/ASV_Loyola_US/MisionesLoyola_dron_2.kml"))
        self.mission_filepath = self.get_parameter('mission_filepath').get_parameter_value().string_value
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        #host
        self.samplepoint_service = self.create_service(Newpoint, 'new_samplepoint', self.new_samplepoint_callback)
        self.mission_mode_service = self.create_service(ASVmode, 'change_mission_mode', self.new_mission_mode)
        self.close_asv_service = self.create_service(CommandBool, 'close_asv', self.close_asv_callback)
        #client
        self.mqtt_send_info = self.create_client(CommandBool, 'MQTT_send_info')
        self.arm_vehicle_client = self.create_client(CommandBool, 'arm_vehicle')
        self.collect_sample_client = self.create_client(CommandBool, 'get_water_module_sample')
        self.change_asv_mode_client = self.create_client(ASVmode, 'change_asv_mode')
        self.go_to_point_client = self.create_client(Newpoint, 'go_to_point_command')

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        #TODO: Topic que publique el estado de la mision para lectura de datos

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

        #spin once so that declared things initialize
        rclpy.spin_once(self)

        #start the drone
        self.startup()
        self.get_logger().info('All systems operative')

        #loop main
        while rclpy.ok():
            rclpy.spin_once(self) #check if a topic has been published or if a timer aired
            self.main()
            time.sleep(1)  #we will run main each 1 second

    def startup(self):
        self.mission_mode = 0  # el modo del ASV deseado
        self.current_mission_mode = -1  # el modo del ASV actual

        self.mission_mode_strs = ["STANDBY", "GUIDED", "MANUAL", "SIMPLE", "RTL"]  # Strings para modos

        self.mqtt_waypoint = [] #store waypoint from Server
        self.status = Status() #Status of the robot

        self.get_logger().debug("Starting startup")
        self.get_logger().info(f"Starting mode:{self.mission_mode} {self.mission_mode_strs[self.mission_mode]}.")
        if self.DEBUG:
            self.get_logger().warning("Debug mode enabled")

        #TODO: deprecate
        #start MQTT periodic send
        #keepgoing=CommandBool.Request()
        #keepgoing.value=True


        #TODO integrar filosofía signal.signal(signal.SIGTERM, manejador_de_senal)


        #INFO: The load of a mission can be implemented externally
        #TODO: Fix error due to path not existing
        self.mg = KMLMissionGenerator(self.mission_filepath)
        self.samplepoints = self.mg.get_samplepoints()

        #TODO: this is a workaround, if it returns false means vehicle is not on due to code flow, an action would be fine implementation
        self.get_logger().debug("Connecting to Vehicle")
        if not self.arm_vehicle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().fatal('vehicle node is not answering')
            #TODO raise error if there has been a timeout,
            #we can try to restart the dronekit node

        #TODO: wait for all systems operative from vehicle
        #     This includes:
        #           Sensor module
        #
        """        except ConnectionRefusedError:
            keep_going = False
            if verbose > 0:
                print("Connection to navio2 could not be made")
        """



    """
    This function automatically runs in loop at 1 Hz
    """
    def main(self):
        if self.mission_mode == 0:  # Stand By
            if self.change_current_mission_mode(self.mission_mode):
                if self.status.armed:
                    self.arm_vehicle(False)
                    self.get_logger().info("Standing By.")
                else:
                    self.get_logger().info("normal start, system disarmed")
        elif self.mission_mode == 1:  # Pre-loaded Mission
            if self.change_current_mission_mode(self.mission_mode):
                self.arm_vehicle(True)
                self.get_logger().info("Starting Pre-loaded Mission.")
                self.samplepoints = self.mg.get_samplepoints()

            if len(self.samplepoints) == 0:
                self.get_logger().info(f"Finished preloaded mission.\nSetting mode to Stand-by.")
                #TODO: reset mission list?
                # self.get_logger().debug("Resetting mission list.")
                self.mission_mode = 0
            else:
                #TODO: action that calls planner to go to waypoint
                # move2wp()
                msg=Newpoint.Request()
                msg.new_point=self.get_next_wp()
                call_service(self, self.go_to_point_client, msg)
        elif self.mission_mode == 2:  # Manual Mode
            if self.change_current_mission_mode(self.mission_mode):
                if self.status.mode != "MANUAL":
                    #TODO: actualizar self.status.mode de otra forma
                    self.status.mode = self.change_ASV_mode("MANUAL")
                    self.get_logger().info(f"Vehicle is now in {self.status.mode}")


        elif self.mission_mode == 3:  # Simple Go-To
            if self.change_current_mission_mode(self.mission_mode):
                if move2wp():
                    self.get_logger().info("Finished simple goto.")
                    self.get_logger().info("Setting mode to Stand-by.")
                    self.mission_mode = 0

        elif self.mission_mode == 4:  # RTL
            if self.change_current_mission_mode(self.mission_mode):
                if vehicle.mode != VehicleMode("RTL"):
                    vehicle.mode = VehicleMode("RTL")
                if verbose > 0:
                    print(f"Vehicle is now in {vehicle.mode}")
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
            nextwp = self.samplepoints.pop(0)
        elif self.current_mission_mode == 3:
            nextwp = self.received_mqtt_wp
        else:
            raise ValueError(f"Current ASV Mode should be 1: {self.mission_mode_strs[1]} or 3: {self.mission_mode_strs[3]}.")
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
        self.get_logger().info("asked for ASV arm." if value else "asked for ASV disarm")
        if call_service(self, self.arm_vehicle_client, aux):
            self.get_logger().info("arm competed" if value else "ASV disarm")
            return True
        else:
            self.get_logger().error("ASV arm failed" if value else "ASV disarm failed")
            return False

    def change_ASV_mode(self, mode):
        #TODO: Establecer diferenciacion con los modos de ardupilot, se recomienda usar string
        #TODO: protección si se indica un modo fuera de rango
        ASVMODES=["STANDBY", "GUIDED", "MANUAL", "SIMPLE", "RTL"]
        aux = ASVmode.Request()
        if type(mode)=='int':
            aux.asv_mode=mode
            self.get_logger().debug(f"asked to change asv to mode: {ASVMODES[mode]}")
            if call_service(self, self.change_asv_mode_client, aux):
                return True
            else:
                # TODO: decide if error here or in dronekit
                return False
        else:
            aux.asv_mode_str=mode
            self.get_logger().debug(f"asked to change asv to mode: {mode}")
            if call_service(self, self.change_asv_mode_client, aux):
                return True
            else:
                #TODO: decide if error here or in dronekit
                return False


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
        publisher = x.create_publisher(Nodeupdate, 'internal_error', 10) #we create the publisher
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
