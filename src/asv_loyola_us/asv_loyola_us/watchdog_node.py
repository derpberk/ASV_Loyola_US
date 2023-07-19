import rclpy
from rclpy.node import Node
from datetime import datetime
import launch_ros.actions
import launch
import os
from ament_index_python.packages import get_package_share_directory
from asv_interfaces.msg import Nodeupdate, String, Status

# TODO: we must assure watchdog never crashes, so keep code simple

watchdog_timer=1
nodes=["mission_node","planner_node", "communication"]

class Watchdog_node(Node):

    def declare_topics(self):
        self.error_subscriber = self.create_subscription(Nodeupdate, '_internal_error', self.error_callback, 10)
        self.watchdog_subscriber = self.create_subscription(Nodeupdate, 'Watchdog', self.watchdog_callback, 10)
        self.error_log_publisher = self.create_publisher(String, 'error', 10)
        self.error_log_publisher_timer = self.create_timer(1, self.error_log_publish)
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)



    def __init__(self):
        #start the node
        super().__init__('Dronekit_node')

        #declarations
        self.declare_topics()
        self.error_list=[]
        self.last_error_subscriber_number=0
        self.watchdog_timer = self.create_timer(watchdog_timer, self.watchdog_timer_function)
        aux=datetime.today()
               
        

        # Ruta de la carpeta
        folder_path ="~/ASV_Loyola_US/logs/"

        # Verificar si la carpeta existe
        if not os.path.exists(folder_path):
            # Crear la carpeta si no existe
            os.makedirs(folder_path)

        # Nombre completo del archivo con la ruta
        file_name = os.path.join(folder_path, aux.strftime("%m.%d.%Y..%H.%M") + ".txt")

        # Abrir el archivo en modo escritura
        self.status_file = open(file_name, "w")





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

    #TODO: start the node from here, so that we can restart things from MQTT
    """def start_sensor_module_callback(self):
        config = os.path.join(
        get_package_share_directory('asv_loyola_us'),
        'config',
        'config.yaml')

        sensors = launch_ros.actions.Node(
        package='asv_loyola_us',
        executable='sensors',
        name='sensors_node',
        parameters = [config]
        launch.LaunchDescription([sensors])
    )"""

    def status_suscriber_callback(self,msg):
        self.status_file.write(f" {datetime.utcfromtimestamp(self.get_clock().now().seconds_nanoseconds()[0]+3600).strftime('%Y-%m-%d %H:%M:%S')} [{msg.lat},{msg.lon}] Battery:{msg.battery}\n")


def main(args=None):
    # init ROS2
    rclpy.init(args=args)
    # start a class that servers the services
    watchdog_node = Watchdog_node()
    # loop the node
    rclpy.spin(watchdog_node)
    # after close connection shut down ROS2
    watchdog_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()