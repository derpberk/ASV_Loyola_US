import rclpy
from rclpy.node import Node
import traceback
import numpy as np
from geopy.distance import lonlat, VincentyDistance
from time import sleep

from asv_interfaces.srv import Newwaypoint, ASVmode, CommandBool, LoadMap
from asv_interfaces.msg import Status, Nodeupdate, Location
from asv_interfaces.action import Samplepoint
#TODO: Everything

class Planner_node(Node):

    # his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('map_filename', 'Loyola.csv')
        path = "~/ASV_Loyola_US/" + self.get_parameter('map_filename').get_parameter_value().string_value
        self.map_filepath = os.path.expanduser(path)
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

    # this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        # host
        self.samplepoint_service = self.create_service(LoadMap, 'load_map', self.load_map_callback)

    def declare_actions(self):
        self.go_to_server = rclpy.action.ActionServer(self, Samplepoint, 'fibonacci', self.go_to_callback)

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        # TODO: Topic que publique el estado de la mision para lectura de datos

    def __init__(self):
        # start the node
        super().__init__('mission_node')

        # declare parameter of drone IP
        self.parameters()


        # declare the services
        self.declare_services()

        # declare actions
        self.declare_actions()

        if self.preload_map():
            self.allow_planning=True
        else:
            self.allow_planning=False


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

    def A_star_get_path(self, start_cell, goal_cell):

        # declarations of list
        self.openlist = []  # here we will store all the cells we have yet to visit
        self.closedlist = []  # here we will store all the cells we have already visited

        # start point append to the cells to visit the one in which we start, the list gotta start somewhere

        self.openlist.append(Cell([start_cell[0], start_cell[1]], [start_cell[0], start_cell[1]], 0, sqrt(
            pow(start_cell[0] - goal_cell[0], 2) + pow(start_cell[1] - goal_cell[1], 2))))

        while len(self.openlist) != 0:  # while we have cells to visit

            candidate = self.openlist.pop(0)  # take out one cell we want to visit
            self.closedlist.append(candidate)  # put it inside the list of cells we have already visited

            if candidate.position == goal_cell:  # if we have reached our goal we exit the main loop
                break

            # ok, so we have not reached the goal, lets use our 4 neighbourghs (right, left, up and down) as candidates of cells we may want to visit

            neighbour = [[candidate.position[0] + 1, candidate.position[1]],
                         [candidate.position[0], candidate.position[1] + 1],
                         [candidate.position[0] - 1, candidate.position[1]],
                         [candidate.position[0], candidate.position[1] - 1]]

            for x in range(4):  # for each neighbour
                already_visited = False  # we will firstly say that we have yet to visit that cell

                if self.map[neighbour[x][0], neighbour[x][
                    1]] == 0:  # ok, first, are we in a wall, 0 means no wall, so we can continue

                    for i in self.openlist:
                        if ([neighbour[x][0], neighbour[x][
                            1]] == i.position):  # have we seen this cell before but we have yet to visit it?
                            already_visited = True  # we will say that we have already visited it
                            if (
                                    i.g < candidate.g - 1):  # is it easier to go to our actual cell from where we were before? (the cost is lower)
                                candidate = self.openlist.pop()  # the cost is lower, so we update the path by saying
                                candidate.father = i.position  # we come from the cell whose cost is lower
                                candidate.g = i.g + 1  # we update the travelled distance
                                self.openlist.append(candidate)  # we store the update

                    for i in self.closedlist:
                        if ([neighbour[x][0],
                             neighbour[x][1]] == i.position):  ##have we seen this cell before and visited it?
                            already_visited = True  # we will say that we have already visited it
                            if (
                                    i.g < candidate.g - 1):  # is it easier to go to our actual cell from where we were before? (the cost is lower)
                                candidate = self.closedlist.pop()  # the cost is lower, so we update the path by saying
                                candidate.father = i.position  # we come from the cell whose cost is lower
                                candidate.g = i.g + 1  # we update the travelled distance
                                self.closedlist.append(candidate)  # we store the update

                    if already_visited == False:  # seems we have yet to see this cell so lets add it to the list of cells we have yet to visit

                        # this if decides how we will measure the cost of the cell
                        if distance_mode == 1:
                            self.openlist.append(
                                Cell([neighbour[x][0], neighbour[x][1]], [candidate.position[0], candidate.position[1]],
                                     candidate.g + 1,
                                     pow(neighbour[x][0] - goal_cell[0], 2) + pow(neighbour[x][1] - goal_cell[1], 2)))
                        else:
                            self.openlist.append(
                                Cell([neighbour[x][0], neighbour[x][1]], [candidate.position[0], candidate.position[1]],
                                     candidate.g + 1,
                                     abs(neighbour[x][0] - goal_cell[0]) + abs(neighbour[x][1] - goal_cell[1])))

            self.openlist.sort(key=lambda Cell: Cell.f)  # we sort the list by cost to restart the loop

        path = []  # here we will store the optimal path to the goal form the start pose
        if len(self.openlist) == 0 and self.closedlist[0].position != goal_cell:  # if we have tried all the paths but we didnt reach the goal position
            self.get_logger.error("we werent able to find a path")
            return []


        else:  # lets compute the path!
            parent = goal_cell  # we will start with the goal cell
            while parent != start_cell:  # until we get to the start point
                position = self.closedlist.pop()  # we take out a cell from the list of points
                if position.position == parent:  # we look for the cell whose coordinates we want
                    path.append(position.father)  # we add that cell to the path
                    parent = position.father  # we update the cell we are looking for

        path.reverse()  # we revert the list, as we want to go from start to finish
        path.append(goal_cell)  # we add the last point as it doesnt appear on the list

        return path





    def load_map_callback(self, request, response):
        if len(request.file_name)>0:
            try:
                self.map_filepath = os.path.expanduser( "~/ASV_Loyola_US/" + request.file_name)
            except:
                response.success=False
                return response
            if self.preload_map():
                response.success = True
                return response
            response.success = False
            return response

        else:
            self.origin = request.origin
            self.resolution = request.resolution
            if self.calculate_map(request.data, request.width, request.height):
                response.success=True
                return response
            response.success=False
            return response




    def calculate_map(self, data, width, height):
        if len(data) != width*height:
            return False

        down = VincentyDistance(height * self.resolution).destination([self.origin.lon, self.origin.lat], bearing=270)
        right= VincentyDistance(width * self.resolution).destination([self.origin.lon, self.origin.lat], bearing=0)

        self.geo_range = [[self.origin.lon, right],
                          [self.origin.lat, down]]

        self.map = np.zeros(height, width)
        counter = 0
        counter2 = 0
        for i in data.data:
            if i != 0:
                self.map[counter2][counter] = 1  # obstacle
            counter += 1
            if counter == self.width:
                counter = 0
                counter2 += 1

    def preload_map(self):

        data=np.genfromtxt(self.map_filepath, sep=',')
        height = self.map.shape[0]
        width = self.map.shape[1]
        self.origin=Location()
        #TODO: read origin an resolution from file
        self.origin.lon=0
        self.origin.lat=0
        self.resolution=1
        if self.calculate_map(data, width, height):
            return True
        return False

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
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
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
