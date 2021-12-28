import sys

from mavros_msgs.srv import CommandBool, CommandLong
import rclpy
from rclpy.node import Node
import time

def get_result(service, future):
    while rclpy.ok():
        rclpy.spin_once(service)
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                service.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                service.get_logger().info(
                    'Result = %d' %
                    (response.success))
            break

class Test_node(Node):

    def __init__(self):
        #we create the test node
        super().__init__('test_node')


        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming') #client to arm the robot
        self.mavlink_client = self.create_client(CommandLong, '/mavros/cmd/command') #client to send commands

        """optional wait for service in case they do not exist
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """


    def arm_robot(self,value):
        req = CommandBool.Request()
        req.value=value
        return self.arm_client.call_async(req)

    def setmode(self, value):
        req=CommandLong.Request()
        req.broadcast = False
        req.command = 176
        req.param1 = 157.0  # mode
        if value=='GUIDED' or 1:
            req.param2 = 15.0   #   Custom mode





        #req.param2 =    #   Custom mode
        #req.param3 =    #   Custom submode

        return self.mavlink_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    test_node = Test_node()
    get_result(test_node, test_node.setmode('GUIDED'))
    time.sleep(1)
    get_result(test_node, test_node.arm_robot(True))


    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
