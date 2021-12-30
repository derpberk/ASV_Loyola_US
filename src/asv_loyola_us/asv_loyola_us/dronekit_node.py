import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobal

#import intefaces
from mavros_msgs.srv import CommandBool
from asv_interfaces.msg import Status


# This node generates the necessary services for  comunication towards the drone (Mavros is in charge of comm from the drone)
#parameters are only read at start

class Dronekit_node(Node):

    #his functions defines and assigns value to the parameters
    def parameters(self):
        self.declare_parameter('vehicle_ip', 'tcp:127.0.0.1:5762')
        self.vehicle_ip = self.get_parameter('vehicle_ip').get_parameter_value().string_value
        self.declare_parameter('timeout', 15)
        self.timout = self.get_parameter('timeout').get_parameter_value().integer_value

    #this function declares the services, its only purpose is to keep code clean
    def declare_services(self):
        self.arm_vehicle = self.create_service(CommandBool, 'arm_vehicle', self.arm_vehicle_callback)

    def declare_topics(self):
        timer_period = 0.5  # seconds
        self.status_publisher = self.create_publisher(Status, 'status', 10)
        self.status_publisher_timer = self.create_timer(timer_period, self.status_publish)

    def __init__(self):
        # start the node
        super().__init__('Dronekit_node')

        # declare parameters
        self.parameters()
        self.status = Status()

        # connect to vehicle
        self.vehicle= vehicle = connect(self.vehicle_ip, timeout=self.timout)
        self.get_logger().info(f"Connecting to vehicle in {self.vehicle_ip}")

        # declare the services
        self.declare_services()
        # start to pubblish
        self.declare_topics()

    def arm_vehicle_callback(self, request, response):
        """
        Arming vehicle function. To arm, the vehicle must be in GUIDED mode. If so, the _vehicle.armed flag
        can be activated. The function waits for the vehicle to be armed.
        Args:
             _vehicle: `dronekit.connection object.
        """
        # Copter should arm in GUIDED mode

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

    def status_publish(self):
        self.status.lat = self.vehicle.location.global_relative_frame.lat,
        self.status.lon = self.vehicle.location.global_relative_frame.lon,
        self.status.yaw = self.vehicle.attitude.yaw,
        self.status.vehicle_id = self.vehicle_id,
        self.status.battery = self.vehicle.battery.level,
        self.status.armed = self.vehicle.armed

        self.publisher_.publish(self.status)






def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    #start a class that servers the services
    dronekit_node = Dronekit_node()
    #loop the node
    rclpy.spin(dronekit_node)
    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()