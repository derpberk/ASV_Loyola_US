from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from asv_interfaces.msg import Status, Nodeupdate, Location, String
from asv_interfaces.srv import CommandBool
from rcl_interfaces.msg import Log
from action_msgs.msg import GoalStatus
from .submodulos.call_service import call_service
from .submodulos.terminal_handler import start_recording, singint_pid
from datetime import datetime
import traceback
import pyzed.sl as sl
from .submodulos.batch_system_handler import *
import numpy as np

class Camera_node(Node):

    def parameters(self):
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.enable_obstacle_avoidance = self.get_parameter('enable_obstacle_avoidance').get_parameter_value().bool_value

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
        
        ##connect to camera

        self.get_logger().info("Initializing camera")

        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.sdk_verbose = False #disable verbose

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger.error("camera couldn't be initialized (impossible to open camera), camera module is ded")
            self.destroy_node()
            return

        obj_param = sl.ObjectDetectionParameters()
        obj_param.enable_tracking=True # Objects will keep the same ID between frames
        obj_param.image_sync=True
        obj_param.enable_mask_output=True # Outputs 2D masks over detected objects
        #obj_param.detection_model=sl.DETECTION_MODEL.CUSTOM_BOX_OBJECTS  #to use our custom data set

        camera_infos = self.zed.get_camera_information()

        positional_tracking_param = sl.PositionalTrackingParameters() #load default parameters
        #positional_tracking_param.set_as_static = True
        positional_tracking_param.set_floor_as_origin = True
        self.zed.enable_positional_tracking(positional_tracking_param)

        self.get_logger()
        if self.enable_obstacle_avoidance:
            err = self.zed.enable_object_detection(obj_param)
            if err != sl.ERROR_CODE.SUCCESS :
                self.get_logger().error("obstacle detenction couldnt be initialized, closing camera")
                self.zed.close()
        else:
            self.get_logger().info("obstacle avoidance not enabled")
        
        self.objects = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.obj_runtime_param.detection_confidence_threshold = 40

        while self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            err = self.zed.retrieve_objects(self.objects, self.obj_runtime_param)
            if self.objects.is_new :
                obj_array = self.objects.object_list
                self.get_logger().info(f"{len(obj_array)} Object(s) detected")
                for objeto in obj_array:
                    self.get_logger().info(f"object {objeto.id} known as {objeto.label} detected at {objeto.position} with confidence {objeto.confidence} status {objeto.tracking_state}")
        # Close the camera
        self.zed.close()


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
        #There has been an error with the program, so we will send the error log to the watchdog
        x = rclpy.create_node('camera_node') #we state what node we are
        publisher = x.create_publisher(Nodeupdate, '_internal_error', 10) #we create the publisher
        #we create the message
        msg = Nodeupdate()
        msg.node = "camera_node" #our identity
        msg.message = traceback.format_exc() #the error
        x.get_logger().error(f"error:{msg.message}")
        #to be sure the message reaches, we must wait till wathdog is listening (publisher needs time to start up)
        #TODO: Vulnerable si alguien esta haciendo echo del topic, el unico subscriptor debe ser wathdog
        # este topic está oculto en echo al usar _
        while publisher.get_subscription_count() == 0: #while no one is listening
            sleep(0.01) #we wait
        publisher.publish(msg) #we send the message
        x.destroy_node() #we destroy node and finish


if __name__ == '__main__':
    main()