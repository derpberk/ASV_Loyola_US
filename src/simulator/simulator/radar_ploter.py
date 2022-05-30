#este nodo llama a 2 servicios de MavROS para poner el vehículo en modo GUIADO y tras 1 segundo armar el vehículo


import sys

import rclpy
from rclpy.node import Node
import time
from asv_interfaces.msg import Obstacles
import numpy as np

class radar_ploter(Node):

    def __init__(self):
        #we create the test node
        super().__init__('radar_ploter_node')
        self.obstacles_subscriber = self.create_subscription(Obstacles, 'camera_obstacles', self.camera_obstacles_callback, 10)
        self.get_logger().info("radar started")
        while rclpy.ok():
            rclpy.spin_once(self)
            time.sleep(0.1)

    def camera_obstacles_callback(self, msg):
        #we need to parse the data
        distance=[2000 for i in range(72)] #2000 or greater is no obstacle
        if len(msg.distance)>72: #max size of array is 72
            size=72
        else:
            size=len(msg.distance)
        for i in range(size):
            distance[i]=np.uint(msg.distance[i])

        s=""
        for i in distance:
            if i<2000:
                s+="|"
            else:
                s+="_"
        self.get_logger().info(s)
        #we start at -54º, with an increment of 1.5 degrees
        
def main():
    rclpy.init()
    radar = radar_ploter()
    radar.destroy_node()

if __name__ == '__main__':
    main()
