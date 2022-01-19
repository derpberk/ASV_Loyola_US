import rclpy
from rclpy.node import Node
import traceback
from time import sleep
from asv_interfaces.srv import Newpoint, ASVmode, CommandBool
from asv_interfaces.msg import Status, Nodeupdate
import os

class Test():
    def __init__(self, name):
        self.name=name
        self.value=0

class Handler_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('debug', False)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value

    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)

    def __init__(self):
        #start the node
        super().__init__('mission_node')

        #declare parameter of drone IP
        self.parameters()

        #declare topics
        self.declare_topics()

        self.active_drones = []


        #loop main
        while rclpy.ok():
            #rclpy.spin_once(self) #check if a topic has been published or if a timer aired
            self.main()
            sleep(1)  #we will run main each 1 second

    """
    This function automatically runs in loop at 1 Hz
    """
    def main(self):
        nodes=self.get_node_names_and_namespaces()
        for i in nodes:
            if len(i[1])>1:
                if i[1][1:] not in self.active_drones:
                    self.get_logger().info(f"New drone \"{i[1][1:]}\" appeared")
                    self.active_drones.append(i[1][1:])

        for drone in self.active_drones:
            aux = False
            for i in nodes:
                if i[1][1:] == drone:
                    aux=True
            if aux==False:
                self.get_logger().info(f"drone \"{drone}\" disappeared")
                self.active_drones.pop(self.active_drones.index(drone))


    def status_suscriber_callback(self,msg):
        self.status=msg




def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    #start a class that servers the services
    handler_node = Handler_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
