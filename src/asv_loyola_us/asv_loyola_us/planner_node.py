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
        self.samplepoint_service = self.create_service(Newpoint, 'new_samplepoint', self.new_samplepoint_callback)
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

        if not self.status.armed:
            self.get_logger().info('vehicle was not armed, Arming Vehicle...')



        while len(path) != 0:
            next_waypoint=path.pop()

            #TODO:
            # llamar action control
            # cerrar action si cambia el path debido a deteccion de obstaculos
            # cerrar si el dron presenta problemas (ej: no esta armado) es decir, devuelve false

            #TODO:Once the position has been reached, change the autopilot mode to LOITER to maintain actual position (disturbance
            # rejection)
            # vehicle.mode = VehicleMode("LOITER")

            # Throw some information about the sampling
            if verbose > 0:
                if current_asv_mode == 1:
                    print("TOMANDO MUESTRAS, quedan: ", len(waypoints), "waypoints")
                else:
                    print("TOMANDO MUESTRAS")
            vehicle.mode = VehicleMode("GUIDED")  # Return to GUIDED to pursue the next waypoint
        #goal_handle.publish_feedback(feedback_msg)


        #TODO: after reaching
        goal_handle.succeed()
        result.success = True
        return result


    def get_path(self, goal):
        #TODO: everything
        return goal


    def move2wp(self):
        """
        Function for moving to the next wp. This function should only be called in mode 1 or 3 (Preloaded mission / simplegoto)
        because it uses `get_next_wp function.
        Returns:
            True when finished, False when it is not possible to move or change the mode.
        """

        # If the vehicle cannot be armed or the autopilot mode is not GUIDED, raise a Warning and returns False.
        # The mode must be guided always to move to the next waypoint.


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
