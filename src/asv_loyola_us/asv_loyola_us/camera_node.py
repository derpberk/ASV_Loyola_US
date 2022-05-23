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
import traceback
import pyzed.sl as sl
from .submodulos.batch_system_handler import *
import numpy as np

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
        
        ##connect to camera

        # Create a InitParameters object and set configuration parameters
        self.zed = sl.Camera()                              
        init_params = sl.InitParameters() #struct
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP  
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.depth_maximum_distance = 20
        self.get_logger().info("camera node initialized")
        self.camera_status = self.zed.open(init_params)

        if self.camera_status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Camera Init failed")
            self.destroy_node()
            return
        
        # Enable positional tracking module
        positional_tracking_parameters = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(positional_tracking_parameters)
        self.get_logger().info("tracking module")


        # Enable object detection module
        batch_parameters = sl.BatchParameters()
        batch_parameters.enable = True
        batch_parameters.latency = 2.0
        batch_handler = BatchSystemHandler(batch_parameters.latency*2)
        obj_param = sl.ObjectDetectionParameters(batch_trajectories_parameters=batch_parameters)
        obj_param.detection_model = sl.DETECTION_MODEL.MULTI_CLASS_BOX
        self.get_logger().info("detection module")
        # Defines if the object detection will track objects across images flow.
        obj_param.enable_tracking = True
        self.zed.enable_object_detection(obj_param)
        self.get_logger().info("object detection")

        camera_infos = self.zed.get_camera_information()
        point_cloud_res = sl.Resolution(min(camera_infos.camera_resolution.width, 720), min(camera_infos.camera_resolution.height, 404)) 
        # Configure object detection runtime parameters
        obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        obj_runtime_param.detection_confidence_threshold = 60
        # To select a set of specific object classes
        obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]# To select a set of specific object classes
        # To set a specific threshold   
        obj_runtime_param.object_class_detection_confidence_threshold = {sl.OBJECT_CLASS.PERSON: 60} # To set a specific threshold

        # Runtime parameters
        runtime_params = sl.RuntimeParameters()
        runtime_params.confidence_threshold = 50

        self.get_logger().info("runtime")

        # Create objects that will store SDK outputs
        point_cloud = sl.Mat(point_cloud_res.width, point_cloud_res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)
        objects = sl.Objects()
        image_left = sl.Mat()

        display_resolution = sl.Resolution(min(camera_infos.camera_resolution.width, 1280), min(camera_infos.camera_resolution.height, 720))
        self.get_logger().info("semi finish")

        self.zed.disable_object_detection()
        self.zed.disable_positional_tracking()
        sleep(2)
        print("closing")
        self.zed.close()
        
        return
        # Utilities for tracks view
        camera_config = self.zed.get_camera_information().camera_configuration
        tracks_resolution = sl.Resolution(400, display_resolution.height)
        image_track_ocv = np.zeros((tracks_resolution.height, tracks_resolution.width, 4), np.uint8)
        
        # Will store the 2D image and tracklet views
        global_image = np.full((display_resolution.height, display_resolution.width+tracks_resolution.width, 4), [245, 239, 239,255], np.uint8)

        point_cloud_render = sl.Mat()
        self.get_logger().info("start loop")

        # Camera pose
        cam_w_pose = sl.Pose()
        cam_c_pose = sl.Pose()

        while rclpy.ok():

            if self.zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve objects
                returned_state = self.zed.retrieve_objects(objects, obj_runtime_param)

                if (returned_state == sl.ERROR_CODE.SUCCESS and objects.is_new):
                    self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU, point_cloud_res)
                    point_cloud.copy_to(point_cloud_render)
                    self.zed.retrieve_image(image_left, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
                    image_render_left = image_left.get_data()
                    self.zed.get_position(cam_c_pose, sl.REFERENCE_FRAME.CAMERA)
                    objects_batch = []
                    self.zed.get_objects_batch(objects_batch)
                    batch_handler.push(cam_c_pose,cam_w_pose,image_left,point_cloud,objects_batch)
                    cam_c_pose, cam_w_pose, image_left, point_cloud_render, objects = batch_handler.pop(cam_c_pose,cam_w_pose,image_left,point_cloud,objects)
                    image_render_left = image_left.get_data()
                    new_objects = objects.is_new
                    np.copyto(image_left_ocv,image_render_left)
                    global_image = cv2.hconcat([image_left_ocv,image_track_ocv])
                    track_view_generator.generate_view(objects, cam_w_pose, image_track_ocv, objects.is_tracked)
                    self.get_logger().info(f" %%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%5\n\nobject:{objects} \n\npoint cloud:{point_cloud} \n\n")

        
        self.get_logger().info("loop finished, close")
        batch_handler.clear()
        self.zed.disable_object_detection()
        self.zed.disable_positional_tracking()
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
    #try:
    camera_node = Camera_node()
    rclpy.spin(camera_node)
    """except:
    """
    #There has been an error with the program, so we will send the error log to the watchdog
    """
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
    x.destroy_node() #we destroy node and finish"""


if __name__ == '__main__':
    main()