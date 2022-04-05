import rclpy
from rclpy.node import Node
import traceback
import numpy as np
from time import sleep
import os

from asv_interfaces.srv import Newpoint, ASVmode, CommandBool, CommandStr
from asv_interfaces.msg import Status, Nodeupdate, Location
from asv_interfaces.action import Samplepoint
from .submodulos.A_star import A_star
#TODO: Everything

class Planner_node(Node):

    # his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('map_filename', 'Loyola121x239')
        path = "~/ASV_Loyola_US/mapas/" + self.get_parameter('map_filename').get_parameter_value().string_value
        self.map_filepath = os.path.expanduser(path)
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

    # this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        # host
        self.load_map_service = self.create_service(CommandStr, 'load_map', self.load_map_callback)
        self.enable_planning_service = self.create_service(CommandBool, 'enable_planning', self.enable_planning_callback)
        self.calculate_path_service = self.create_service(Newpoint, 'calculate_path', self.calculate_path_callback)

    #def declare_actions(self):
        #self.go_to_server = rclpy.action.ActionServer(self, Samplepoint, 'fibonacci', self.go_to_callback)

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        # TODO: Topic que publique el estado de la mision para lectura de datos

    def __init__(self):
        # start the node
        super().__init__('planner_node')

        # declare parameter of drone IP
        self.parameters()


        # declare the services
        self.declare_services()

        # declare actions
        #self.declare_actions()
        self.use_planner=False
        self.status=Status()
        try:
            self.planner=A_star(self.map_filepath, 1)
            self.get_logger().info("Planner node initialized")
        except:
            error = traceback.format_exc()
            self.get_logger().error(f"Planner unknown error:\n {error}")
            self.get_logger().fatal("Planner module is dead")
            self.destroy_node()



    def load_map_callback(self, request, response):
        if len(request.file_name)>0:
            try:
                self.map_filepath = os.path.expanduser("~/ASV_Loyola_US/mapas/" + request.file_name)
                self.planner=A_star(self.map_filepath, 1)
            except:
                response.success=False
        return response

    def enable_planning_callback(self, request, response):
        if self.request.value:
            self.use_planner=True
            self.get_logger().info("planner enabled")
        else:
            self.use_planner=False
            self.get_logger().info("planner disabled")
        return response

    def calculate_path_callback(self, request, response):
        #check if we have information about the drone
        if self.status.asv_mode == "Offline":
            response.success=False
            self.get_logger().error("Tried to get path but drone is not online")
            return response
        #check if we want to use the planner
        if not self.use_planner:
            response.point_list=[request.new_point]
            return response

        #check if drone is inside a wall (non reachable point)
        aux=self.planner.calculate_cell([self.status.lat,self.status.lon])
        if self.planner.map[aux[0]][aux[1]]:
            self.get_logger().error("Drone is inside a wall, cannot calculate path")
            return response
        #check if destination is insida a wall
        aux=self.planner.calculate_cell([request.new_point.lat,request.new_point.lon])
        if self.planner.map[aux[0]][aux[1]]:
            self.get_logger().error("Drone is inside a wall, cannot calculate path")
            return response
        
        #save starttime
        starttime=self.get_clock().now().seconds_nanoseconds()
        #calculate path
        self.planner.compute_gps_path([self.status.lat,self.status.lon], [request.new_point.lat,request.new_point.lon])
        #save stoptime and print it
        stoptime=self.get_clock().now().seconds_nanoseconds()
        if stoptime[1]>starttime[1]:
            time=float(stoptime[0]-starttime[0]-1)+float(stoptime[1]-starttime[1] +10000000000)/10000000000
        else:
            time=float(stoptime[0]-starttime[0])+float(stoptime[1]-starttime[1])/10000000000
        time=round(time, 9)
        if self.planner.gps_path is not None:
            self.get_logger().info(f"path calculated in {time} seconds")
        else:
            self.get_logger().info(f"No path was found, time spent: {time} seconds")
            response.success=False
            return response
        
        #return gps points

        self.gps_path.pop(0) #we may want to pop the first point, as it is the position of the drone

        for i in self.planner.gps_path:
            aux=Location()
            aux.lat=i[0]
            aux.lon=i[1]
            response.point_list.append[i]

        return response

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
        planner_node.destroy_node()
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
