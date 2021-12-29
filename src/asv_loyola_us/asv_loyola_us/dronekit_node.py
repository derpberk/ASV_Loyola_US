import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobal

#import intefaces
from mavros_msgs.srv import CommandBool


# This node generates the necessary services for  comunication towards the drone (Mavros is in charge of comm from the drone)
#parameters are only read at start

class Dronekit_node(Node):

    #his functions defines and assigns value to the
    def parameters(self):
        self.declare_parameter('vehicle_ip', 'tcp:127.0.0.1:5762')
        self.vehicle_ip = self.get_parameter('vehicle_ip').get_parameter_value().string_value
        self.declare_parameter('timeout', 15)
        self.timout=self.get_parameter('timeout').get_parameter_value().integer_value

    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        self.arm_vehicle = self.create_service(CommandBool, 'arm_vehicle', self.arm_vehicle_callback)

    def __init__(self):
        #start the node
        super().__init__('Dronekit_node')

        #declare parameter of drone IP
        self.parameters()

        #connect to vehicle
        self.vehicle= vehicle = connect(self.vehicle_ip, timeout=self.timout)

        #declare the services
        self.declare_services()

    def arm_vehicle_callback(self, request, response):
        """
        Arming vehicle function. To arm, the vehicle must be in GUIDED mode. If so, the _vehicle.armed flag
        can be activated. The function waits for the vehicle to be armed.
        Args:
             _vehicle: `dronekit.connection object.
        """
        # Copter should arm in GUIDED mode

        #TODO: Use request.value to arm or disarm the vehicle

        try:
            if request.value:
                self.vehicle.mode = VehicleMode("GUIDED")
                self.vehicle.arm()
                self.get_logger().info('Vehicle armed')
                response.success=True
            else:
                self.vehicle.disarm()
                self.get_logger().info('Vehicle disarmed')
                response.success=True
        except:
            self.get_logger().error('Couldn\'t arm vehicle')
            response.success=False
        return response


def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    #start a class that servers the services
    dronekit_node = Dronekit_node()
    #loop the services
    rclpy.spin(dronekit_node)
    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()