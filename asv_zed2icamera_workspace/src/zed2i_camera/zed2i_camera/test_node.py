import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header, Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
from asv_interfaces.msg import TrashMsg

class Test_node(Node):
    def __init__(self):
        super().__init__('test_node')
        queue_size = 1
        qos_profile_BEF=rclpy.qos.QoSProfile(
            depth=queue_size,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
            )
        qos_profile_REL=rclpy.qos.QoSProfile(
            depth=queue_size,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
            )
        self.navsatfix_publisher = self.create_publisher(NavSatFix, '/mavros/global_position/global', qos_profile_BEF)
        self.compass_hdg_publisher = self.create_publisher(Float64, '/mavros/global_position/compass_hdg', qos_profile_BEF)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.navsatfix_compass_hdg_publisher_callback)

        self.trash_detections_subscription = self.create_subscription(TrashMsg, '/zed2i_trash_detections/trash_localization', self.trash_detections_callback, qos_profile_BEF)
    
    def navsatfix_compass_hdg_publisher_callback(self):
        #self.get_logger().info('Timer event')
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Position in degrees.
        msg.latitude = 37.4109078
        msg.longitude = -6.0027590

        # Altitude in metres.
        msg.altitude = 0.0

        msg.position_covariance[0] = 0
        msg.position_covariance[4] = 0
        msg.position_covariance[8] = 0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        msg_heading = Float64()
        msg_heading.data = 55.0
        self.navsatfix_publisher.publish(msg)
        self.compass_hdg_publisher.publish(msg_heading)

    def trash_detections_callback(self,msg):
        
        self.get_logger().info("The position of "+ msg.date + " is ({:.10f}), {:.10f}).".format(msg.lat,msg.lon))

def main(args=None):
    rclpy.init(args=args)

    test_node = Test_node()

    rclpy.spin(test_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
