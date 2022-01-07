import rclpy
from rclpy.node import Node
from datetime import datetime

from asv_interfaces.msg import Nodeupdate, String

# TODO: we must assure watchdog never crashes, so keep code simple

watchdog_timer=1
nodes=["mission_node","planner_node", "communication"]

class Watchdog_node(Node):

    def declare_topics(self):
        self.error_subscriber = self.create_subscription(Nodeupdate, 'internal_error', self.error_callback, 10)
        self.watchdog_subscriber = self.create_subscription(Nodeupdate, 'Watchdog', self.watchdog_callback, 10)
        self.error_log_publisher = self.create_publisher(String, 'error', 10)
        self.error_log_publisher_timer = self.create_timer(1, self.error_log_publish)


    def __init__(self):
        #start the node
        super().__init__('Dronekit_node')

        #declarations
        self.declare_topics()
        self.error_list=[]
        self.last_error_subscriber_number=0
        self.watchdog_timer = self.create_timer(watchdog_timer, self.watchdog_timer_function)

    def error_callback(self,msg):
        error = "there has been an error in node: \'%s\' at time %s\n\n%s" % (msg.node, datetime.utcfromtimestamp(self.get_clock().now().seconds_nanoseconds()[0]+3600).strftime('%Y-%m-%d %H:%M:%S'), self.parse_error(msg.message))
        # TODO: disarm vehicle if error detected
        self.get_logger().fatal(error)
        self.error_list.append(error)
        #in case we were subscribed we say that there has been an error
        if self.error_log_publisher.get_subscription_count() > 0:
            msg2=String()
            msg2.string=error
            self.error_log_publisher.publish(msg2)

        #TODO: ¿queremos tener el error en otro formato, y en un archivo a parte del que obtenemos de [FATAL] que se guarda en los logs de ROS?

    def parse_time(self,time):
        aux=time.seconds_nanoseconds()
        return datetime.utcfromtimestamp(aux[0]+86400).strftime('%Y-%m-%d %H:%M:%S') #add +86400 for Spains Clocktime

    def parse_error(self,error):
        return error
        #TODO: realizar un parse del error para hacerlo más corto y legible

    def watchdog_timer_function(self):
        dummy=0
        #TODO: incrementar un contador para cada nodo

        #TODO: Si el contador ha alcanzado un valor mayor a x, ejecutar una rutina de desbloqueo

    def watchdog_callback(self):
        dummy=0
        #TODO: Poner el contador de cada nodo a 0


    def error_log_publish(self):
        if self.error_log_publisher.get_subscription_count() != self.last_error_subscriber_number:
            self.last_error_subscriber_number=self.error_log_publisher.get_subscription_count()
            msg = String()
            if len(self.error_list) == 0:
                msg.string = "There has been no error so far"
                self.error_log_publisher.publish(msg)
            else:
                for i in self.error_list:
                    msg.string=str(i)
                    self.error_log_publisher.publish(msg)

def main(args=None):
    # init ROS2
    rclpy.init(args=args)
    # start a class that servers the services
    watchdog_node = Watchdog_node()
    # loop the node
    rclpy.spin(watchdog_node)
    # after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()