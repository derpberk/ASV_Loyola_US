from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from asv_interfaces.msg import Status, Nodeupdate, Location, String, Sensor
from asv_interfaces.srv import ASVmode, CommandBool, Newpoint, LoadMission, SensorParams, PlannerParams
from rcl_interfaces.msg import Log
from action_msgs.msg import GoalStatus
from .submodulos.call_service import call_service
from .submodulos.terminal_handler import start_recording, singint_pid
from datetime import datetime
import threading
import traceback

class Camera_node(Node):

    def parameters(self):
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value

    def declare_services(self):
        self.sendinfo = self.create_service(CommandBool, 'camera_recording', self.camera_recording_callback)


    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.mission_mode_subscriber = self.create_subscription(String, 'mission_mode', self.mission_mode_suscriber_callback, 10)

    #def declare_actions(self):

    def __init__(self):
        #start the node
        super().__init__('camera_node')

        #call the parameters
        self.parameters()

        #declare variables
        self.status=Status()
        self.mission_mode = ""
        self.recording=False

        #call services
        self.declare_services()

        #call actions
        #self.declare_actions()

        #declare topics
        self.declare_topics()
        self.get_logger().info("camera node initialized")


    def status_suscriber_callback(self, msg):
        self.status = msg


    def mission_mode_suscriber_callback(self, msg):
        self.mission_mode=msg.string

    def camera_recording_callback(self, request, response):
        if request.value:
            if self.recording:
                self.get_logger().info("camera already recording")
                response.success=False
                return response #we are already recording
            self.get_logger().info("started recording")
            self.recording_instance = start_recording()
            self.recording=True
        else:
            if not self.recording: #we arent recording
                response.success=False
                self.get_logger().info("camera wasnt recording")
                return response
            self.get_logger().info("stop recording")
            singint_pid(self.recording_instance)
            self.recording=False
        return response
            

def main():
    rclpy.init()
    try:
        camera_node = Camera_node()
        rclpy.spin(camera_node)
    except:
        """
        There has been an error with the program, so we will send the error log to the watchdog
        """
        x = rclpy.create_node('camera_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "camera_node" #our identity
        msg.message = traceback.format_exc() #the error
        #to be sure the message reaches, we must wait till wathdog is listening (publisher needs time to start up)
        #TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
        # este topic está oculto en echo al usar _
        while publisher.get_subscription_count() == 0: #while no one is listening
            sleep(0.01) #we wait
        publisher.publish(msg) #we send the message
        x.destroy_node() #we destroy node and finish


if __name__ == '__main__':
    main()