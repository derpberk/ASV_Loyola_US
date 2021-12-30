import rclpy
from rclpy.node import Node
from .submodulos.KMLMissionGeneration import KMLMissionGenerator

from geometry_msgs.msg import Point
from asv_interfaces.srv import Newwaypoint
from mavros_msgs.srv import CommandBool

from geometry_msgs.msg import Point
class Mission_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('mission_filename', '1')
        self.mission_filename = self.get_parameter('mission_filename').get_parameter_value().string_value
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        self.samplepoint_service = self.create_service(Newwaypoint, 'new_samplepoint', self.new_samplepoint_callback)
        self.mqtt_send_info = self.create_client(CommandBool, 'MQTT_send_info')
        self.arm_vehicle = self.create_client(CommandBool, 'arm_vehicle')
        self.collect_sample = self.create_client(CommandBool, 'get_water_module_sample')

    def declare_topics(self):
        dummy=0


    def __init__(self):
        #start the node
        super().__init__('Dronekit_node')

        #declare parameter of drone IP
        self.parameters()

        #declarations
        self.samplepoints=[] #list of waypoints to follow
        #declare the services
        self.declare_services()

    def call_service(self, client,  msg):
        # TODO: raise error to avoid infinite wait if service is not up, after all means a module is not active
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
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




    def new_samplepoint_callback(self, request, response):
        #TODO: Implementar

        """
        this function starts when a new samplepoint is received from MQTT or elsewhere
        it appends a new samplepoint to the samplepoint list
        it sends back the samplepoints list
        """

        self.samplepoints.append(request.new_point)
        self.get_logger().info('new waypoint received: ', request.new_point)
        response.point_list = self.samplepoints
        return response


    def get_next_wp(_vehicle):
        #TODO: Implementar
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

        if current_mission_mode == 1:  # Preloaded
            nextwp = waypoints.pop(0)
        elif current_mission_mode == 3:
            nextwp = received_mqtt_wp
        else:
            raise ValueError(f"Current ASV Mode should be 1: {mission_mode_strs[1]} or 3: {mission_mode_strs[3]}.")
        self.get_logger().info("Next waypoint is", nextwp)
        return LocationGlobal(nextwp[1], nextwp[0], nextwp[2])



    def main(self):
        self.mission_mode = 0  # el modo del ASV deseado
        self.current_mission_mode = -1  # el modo del ASV actual

        self.mission_mode_strs = ["STANDBY", "GUIDED", "MANUAL", "SIMPLE", "RTL"]  # Strings para modos

        self.get_logger().info("ASV ready to connect.")
        self.get_logger().info(f"Starting mode:{self.mission_mode} {self.mission_mode_strs[mission_mode]}.")
        if self.DEBUG:
            self.get_logger().info("Debug mode enabled")

        #start MQTT periodic send
        keepgoing=CommandBool.Request()
        keepgoing.value=True


        #TODO integrar filosofía signal.signal(signal.SIGTERM, manejador_de_senal)


        #INFO: The load of a mission can be implemented externally
        mg = KMLMissionGenerator(self.mission_filename)
        self.samplepoints = mg.get_samplepoints()

        #this is a workaround, if it returns false means vehicle is not on due to code flow
        if not self.arm_vehicle.wait_for_service(timeout_sec=1.0):
            dummy=0
            #TODO raise error if there has been a timeout,
            #we can try to restart the dronekit node

        """        except ConnectionRefusedError:
            keep_going = False
            if verbose > 0:
                print("Connection to navio2 could not be made")
        """

        #this is a workaround, if it returns false means sensor is not on (either in debug or real sensor)
        if not self.collect_sample.wait_for_service(timeout_sec=1.0):
            dummy = 0
            # TODO raise error if there has been a timeout,
            # we can try to restart sensor mode, send MQTT sensor is not connected or raise an error

        # creamos el hilo que continuamente envia datos de posicion al servidor
        self.call_service(self.mqtt_send_info, keepgoing)  # this funcion returns whether MQTT start has been a success


        # p misiones mas grandes keep_going debe bajar a false si vehicle.battery.level < 0.6

        #TODO: en lugar de utilizar keep going nodo coordinador debe ser capaz de gestionar nodos y eliminarlos en caso de error lanzando nodos alternativos
        while keep_going:
            #TODO: Pasar a ROS TODO ESTO
            try:
                if self.mission_mode == 0:  # Stand By
                    if change_current_mission_mode(self.mission_mode):
                        if vehicle.armed:
                            vehicle.disarm()
                        if verbose > 0:
                            print("ASV is armed." if vehicle.armed else "ASV is disarmed. Standing By.")
                    time.sleep(1)
                elif self.mission_mode == 1:  # Pre-loaded Mission
                    if verbose > 0 and self.change_current_mission_mode(self.mission_mode):
                        print("Starting Pre-loaded Mission.")

                    if len(waypoints) == 0:
                        if verbose > 0:
                            print(f"Finished preloaded mission.")
                            print("Setting mode to Stand-by.")
                            print("Resetting mission list.")
                        self.mission_mode = 0
                        waypoints = mg.get_mission_list()[0]
                        continue

                    if move2wp():
                        time.sleep(1)

                elif self.mission_mode == 2:  # Manual Mode
                    if self.change_current_mission_mode(self.mission_mode):
                        if vehicle.mode != VehicleMode("MANUAL"):
                            vehicle.mode = VehicleMode("MANUAL")
                        if verbose > 0:
                            print(f"Vehicle is now in {vehicle.mode}")
                    else:
                        time.sleep(1)

                elif self.mission_mode == 3:  # Simple Go-To
                    if self.change_current_mission_mode(self.mission_mode):
                        if move2wp():
                            if verbose > 0:
                                print("Finished simple goto.")
                                print("Setting mode to Stand-by.")
                            mission_mode = 0

                elif self.mission_mode == 4:  # RTL
                    if self.change_current_mission_mode(self.mission_mode):
                        if vehicle.mode != VehicleMode("RTL"):
                            vehicle.mode = VehicleMode("RTL")
                        if verbose > 0:
                            print(f"Vehicle is now in {vehicle.mode}")
                    else:
                        time.sleep(1)
            except Exception as e:
                mission_mode = 0
                time.sleep(1)
                print(str(e))

        # Cerramos la conexion con el navio2
        #TODO:servicio que cierre Navio2
        self.get_logger().info("FINISHING MISSION")
        try:
            vehicle.disarm()
            vehicle.close()
        except NameError:
            pass

        # Cerramos la conexion con los sensores si existen#
        if not DEBUG and SENSOR:
            modulo_de_sensores.close()
            
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

def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    #start a class that servers the services
    mission_node = Mission_node()
    #loop the services
    rclpy.spin(mission_node)
    #after close connection shut down ROS2
    rclpy.shutdown()




if __name__ == '__main__':
    main()
