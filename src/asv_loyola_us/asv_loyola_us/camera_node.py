from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from asv_interfaces.msg import Status, Nodeupdate, Location, String, Camera, Obstacles
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
import threading
from math import atan2, degrees

class Camera_node(Node):

    def parameters(self):
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.enable_obstacle_avoidance = self.get_parameter('enable_obstacle_avoidance').get_parameter_value().bool_value

    def declare_services(self):
        self.sendinfo = self.create_service(CommandBool, 'camera_recording', self.camera_recording_callback)
        self.reset_home_service = self.create_service(CommandBool, 'enable_obstacle_avoidance', self.obstacle_avoidance_enable)



    def declare_topics(self):
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_suscriber_callback, 10)
        self.mission_mode_subscriber = self.create_subscription(String, 'mission_mode', self.mission_mode_suscriber_callback, 10)
        self.obstacles_publisher = self.create_publisher(Obstacles, 'camera_obstacles', 10)


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
        self.stop_camera_detection=False


        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        #init_params.camera_fps = 30  # Set fps at 30
        init_params.sdk_verbose = False #disable verbose
        

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("camera couldn't be initialized (impossible to open camera), camera module is ded")
            self.destroy_node()
            return

        self.objects = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.obj_runtime_param.detection_confidence_threshold = 70

        self.camera_detection_thread = threading.Thread(target=self.camera_perception)
        self.camera_recording_thread = threading.Thread(target=self.camera_recording)



        if self.enable_obstacle_avoidance:
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

            err = self.zed.enable_object_detection(obj_param)
            if err != sl.ERROR_CODE.SUCCESS :
                self.get_logger().error("obstacle detenction couldnt be initialized, closing camera")
                self.zed.close()
                return
            self.camera_detection_thread.start()
        else:
            self.get_logger().info("obstacle avoidance not enabled")


        
    def obstacle_avoidance_enable(self, request, response):
        if request.value:
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

            err = self.zed.enable_object_detection(obj_param)
            if err != sl.ERROR_CODE.SUCCESS :
                self.get_logger().error("obstacle detenction couldnt be initialized")
                response.success=False
                return response
            self.get_logger().info("obstacle avoidance enabled")
            self.camera_detection_thread.start()
        else:
            self.zed.disable_object_detection()
            self.zed.disable_positional_tracking()
            self.stop_camera_detection=True
            self.get_logger().info("obstacle avoidance disabled")
        return response


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
            self.recording=True
            self.camera_recording_thread.start()
        else:
            if not self.recording: #we arent recording
                response.success=False
                self.get_logger().info("camera wasnt recording")
                return response
            self.get_logger().info("stop recording")
            self.recording=False
        return response
            
    def camera_perception(self):
        self.get_logger().info("object avoidance enabled")
        while (self.zed.grab() == sl.ERROR_CODE.SUCCESS) and (self.stop_camera_detection == False):
            err = self.zed.retrieve_objects(self.objects, self.obj_runtime_param)
            if err != sl.ERROR_CODE.SUCCESS :
                self.get_logger().error("objects retrieval failed")
                continue

            if self.objects.is_new :
                obj_array = self.objects.object_list
                #self.get_logger().info(f"{len(obj_array)} Object(s) detected")
                obstacles=Obstacles()
                obstacles.angle_increment=1.5 # we will cover an area of 110º (camera aperture) starting from -54 to 54º with an increment of 1.5 degrees as we can send at most 72 values
                obstacles.distance=[2000 for i in range(72)] #2000 or greater is no obstacle
                for objeto in obj_array: #we will store everything of the object
                    obj = Camera()
                    obj.id = int(objeto.id) 
                    label=str(repr(objeto.label))
                    sublabel=str(repr(objeto.sublabel))
                    #self.get_logger().info(f"label: {label}, sublabel {sublabel}",once=True)
                    obj.label = label
                    obj.position = [objeto.position[0], objeto.position[1], objeto.position[2]]
                    #self.get_logger().info(f"pos: {[objeto.position[0], objeto.position[1], objeto.position[2]]}")
                    obj.confidence = objeto.confidence
                    #obj.tracking_state = objeto.tracking_state
                    #self.get_logger().info(f"track: {objeto.tracking_state}",once=True)
                    obj.dimensions = [objeto.dimensions[0], objeto.dimensions[1], objeto.dimensions[2]]
                    obj.velocity = [objeto.velocity[0], objeto.velocity[1], objeto.velocity[2]]
                    for i in range(4):
                        for j in range(2):
                            obj.bounding_box_2d.append(objeto.bounding_box_2d[i][j])
                    for i in range(8):
                        for j in range(3):
                            obj.bounding_box.append(objeto.bounding_box[i][j])
                    obstacles.objects.append(obj)

                    #for obstacle detection we apply pythagoras theorem

                    minangle=degrees(atan2(objeto.position[0]-objeto.dimensions[0]/2,objeto.position[2]))
                    maxangle=degrees(atan2(objeto.position[0]+objeto.dimensions[0]/2,objeto.position[2]))
                    #self.get_logger().info(f"{label}, {sublabel} , angles: {[minangle, maxangle]}")
                    if abs(minangle)>55 or maxangle>55:
                        self.get_logger().error("object trepassed camera limits")
                    else:
                        for i in range(int((53+minangle)/1.5),int((53+maxangle)/1.5)):
                            if obstacles.distance[i]>int(objeto.position[2]*100):
                                obstacles.distance[i]=int(objeto.position[2]*100)
                self.obstacles_publisher.publish(obstacles)

    def camera_recording(self):
        name=datetime.today()
        name=name.strftime('%Y.%m.%d..%H.%M')
        name=str("/home/xavier/zed_datasets/recording"+name+".svo")
        recording_param = sl.RecordingParameters(name, sl.SVO_COMPRESSION_MODE.H264)
        err = self.zed.enable_recording(recording_param)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().info("camera could not start recording")
            return #error
        self.get_logger().info("started recording")
        runtime = sl.RuntimeParameters()
        while True:
            if self.zed.grab(runtime):
                pass #we will enter this if each new frame
            if self.recording==False:
                self.zed.disable_recording()
                return
                


def main():
    rclpy.init()
    try:
        camera_node = Camera_node()
        rclpy.spin(camera_node)
        # After finish close the camera
        camera_node.get_logger().info("normal finish")
        camera_node.zed.close()
    except:
        #There has been an error with the program, so we will send the error log to the watchdog
        zed=sl.Camera()
        zed.close()
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