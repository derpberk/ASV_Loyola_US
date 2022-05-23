#este nodo llama a 2 servicios de MavROS para poner el vehículo en modo GUIADO y tras 1 segundo armar el vehículo


import sys

import rclpy
from rclpy.node import Node
import time
from pymavlink import mavutil
from numpy import uint
from dronekit import connect, VehicleMode, LocationGlobal

from pymavlink.dialects.v20 import ardupilotmega as mavlink2 #for obstacle distance information

import os
os.environ["MAVLINK20"] = "1"

class Test_node(Node):

    def __init__(self):
        #we create the test node
        super().__init__('test_node')
        tipo=0

        if tipo==1:
            conn = mavutil.mavlink_connection(
                "tcp:navio.local:5678",
                autoreconnect = True,
                source_system = 1,
                source_component = 93,
                baud=57600, #921600
                force_connected=True,
                dialect = "ardupilotmega",
            )
        else:
            vehicle=connect("tcp:navio.local:5678", timeout=15, source_system=1, source_component=93)


        while rclpy.ok():
            dist=[200+10*i for i in range(72)]
            if tipo==1:

                conn.mav.obstacle_distance_send(
                    0,    # us Timestamp (UNIX time or time since system boot)
                    0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
                    dist,          # distances,    uint16_t[72],   cm
                    0,                  # increment,    uint8_t,        deg
                    30,	    # min_distance, uint16_t,       cm
                    2000,       # max_distance, uint16_t,       cm
                    3,	    # increment_f,  float,          deg
                    0,       # angle_offset, float,          deg
                    12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
                )
                self.get_logger().info(f"msg:\n{dist}",once=True)
            else:
                msg=mavlink2.MAVLink_obstacle_distance_message(
                    0,    # us Timestamp (UNIX time or time since system boot)
                    3,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
                    dist,           # distances,    uint16_t[72],   cm
                    int(15),                  # increment,    uint8_t,        deg
                    int(30),	                # min_distance, uint16_t,       cm
                    int(2000),               # max_distance, uint16_t,       cm
                    float(10.0),	    # increment_f,  float,          deg #https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
                    float(-10.0),       # angle_offset, float,          deg
                    12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
                    )
                self.get_logger().info(f"msg:\n{msg}",once=True)

                vehicle.send_mavlink(msg)
            
            time.sleep(0.05)
        





def main(args=None):
    rclpy.init(args=args)
    test_node = Test_node()
    rclpy.spin(test_node)


    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
