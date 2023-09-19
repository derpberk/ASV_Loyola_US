import os 
import sys
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from asv_interfaces.msg import Status, Nodeupdate, Location, String, Camera, Obstacles
from asv_interfaces.srv import CommandBool
from rcl_interfaces.msg import Log
from .submodulos.call_service import call_service
from datetime import datetime
import traceback
import pyzed.sl as sl
import cv2
import numpy as np
import threading
from math import atan2, degrees
import torch
import torch.backends.cudnn as cudnn
sys.path.insert(0, '/home/xavier/repositorios/yolov5')
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device
from utils.augmentations import letterbox

class Camera_node(Node):

    def parameters(self):
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value
        self.declare_parameter('img_size', 640)
        self.img_size = self.get_parameter('img_size').get_parameter_value().integer_value
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.enable_obstacle_avoidance = self.get_parameter('enable_obstacle_avoidance').get_parameter_value().bool_value
        self.declare_parameter('weights_filename', 'best.pt')
        path = "/home/ASV_Loyola_US/" + self.get_parameter('weights_filename').get_parameter_value().string_value
        self.weights_filepath = os.path.expanduser(path)
        self.declare_parameter('confidence', 0.4)
        self.confidence = self.get_parameter('confidence').get_parameter_value().double_value

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
        

        #define objects for threads
        self.lock = threading.Lock()
        self.run_signal = False
        self.stop_camera_detection=False

        # Create a InitParameters object and set configuration parameters
        #init_params = sl.InitParameters(svo_real_time_mode=True)
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        #init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.camera_fps = 30  # Set fps at 30
        init_params.sdk_verbose = True #disable verbose

        # Open the camera
        try:
            err = self.zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"camera couldn't be initialized:{err}")
                self.get_logger().fatal("Camera module is dead")
                self.destroy_node()
                return
        except:
            error = traceback.format_exc()
            self.get_logger().error(f"Connection to camera could not be made, unknown error:\n {error}")
            self.get_logger().fatal("Camera module is dead")
            self.destroy_node()
            

        #enable positional tracking
        positional_tracking_param = sl.PositionalTrackingParameters() #load default parameters
        #positional_tracking_param.set_as_static = True
        positional_tracking_param.set_floor_as_origin = True
        self.zed.enable_positional_tracking(positional_tracking_param)

        #enable object detection    
        obj_param = sl.ObjectDetectionParameters()
        obj_param.enable_tracking=True # Objects will keep the same ID between frames
        obj_param.detection_model = sl.DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.image_sync=True
        obj_param.enable_mask_output=True # Outputs 2D masks over detected objects

        err = self.zed.enable_object_detection(obj_param)
        if err != sl.ERROR_CODE.SUCCESS :
            self.get_logger().error("obstacle detenction couldnt be initialized, closing camera")
            self.zed.close()
            return

        #shared variables
        self.objects = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        runtime_params = sl.RuntimeParameters()
        #self.obj_runtime_param.detection_confidence_threshold = 70
        image_left_tmp = sl.Mat()

        #declare threads
        self.camera_detection_thread = threading.Thread(target=self.camera_perception, args=(self.weights_filepath,self.img_size, self.confidence,)) #weights, img_size, confidence
        self.camera_recording_thread = threading.Thread(target=self.camera_recording)
            
        #if defined at start, start obstacle avoidance thread   
        if self.enable_obstacle_avoidance:
            self.camera_detection_thread.start()
        else:
            self.get_logger().info("obstacle avoidance not enabled")

        while rclpy.ok():
            if self.zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                # -- Get the image
                self.lock.acquire()
                self.zed.retrieve_image(image_left_tmp, sl.VIEW.LEFT)
                self.image_net = image_left_tmp.get_data()
                self.lock.release()
                self.run_signal = True
                # -- Detection running on the other thread
                while self.run_signal:
                    sleep(0.001)
                # Wait for detections
                self.lock.acquire()
                # -- Ingest detections
                self.zed.ingest_custom_box_objects(self.detections)
                self.lock.release()
                self.zed.retrieve_objects(self.objects, self.obj_runtime_param)
                if self.objects.is_new :
                    obj_array = self.objects.object_list
                    #self.get_logger().info(f"{len(obj_array)} Object(s) detected")
                    obstacles=Obstacles()
                    obstacles.angle_increment=1.5 # we will cover an area of 110º (camera aperture) starting from -54 to 54º with an increment of 1.5 degrees as we can send at most 72 values
                    obstacles.distance=[2000 for i in range(72)] #2000 or greater is no obstacle
                    try:
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
                            try:
                                minangle=degrees(atan2(objeto.bounding_box_2d[0][0],objeto.position[2]))
                                maxangle=degrees(atan2(objeto.bounding_box_2d[1][0],objeto.position[2]))
                            except:
                                continue
                            self.get_logger().info(f"{objeto.bounding_box_2d}, {objeto.position} , angles: {[minangle, maxangle]}")

                            #self.get_logger().info(f"{label}, {sublabel} , angles: {[minangle, maxangle]}")
                            if abs(minangle)>55 or maxangle>55:
                                self.get_logger().error("object trepassed camera limits")
                            else:
                                for i in range(int((53+minangle)/1.5),int((53+maxangle)/1.5)):                        
                                    if obstacles.distance[i]>int(objeto.position[2]*100):
                                        obstacles.distance[i]=int(objeto.position[2]*100)
                        self.obstacles_publisher.publish(obstacles)
                    except:
                        pass
        self.zed.close()

        
    def obstacle_avoidance_enable(self, request, response):
        if request.value:
            self.get_logger().info("obstacle avoidance enabled")
            self.stop_camera_detection=False
            self.camera_detection_thread.start()
        else:
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
                
    def img_preprocess(self, img, device, half, net_size):
        net_image, ratio, pad = letterbox(img[:, :, :3], net_size, auto=False)
        net_image = net_image.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        net_image = np.ascontiguousarray(net_image)

        img = torch.from_numpy(net_image).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0

        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        return img, ratio, pad

    def xywh2abcd(self, xywh, im_shape):
        output = np.zeros((4, 2))

        # Center / Width / Height -> BBox corners coordinates
        x_min = (xywh[0] - 0.5*xywh[2]) * im_shape[1]
        x_max = (xywh[0] + 0.5*xywh[2]) * im_shape[1]
        y_min = (xywh[1] - 0.5*xywh[3]) * im_shape[0]
        y_max = (xywh[1] + 0.5*xywh[3]) * im_shape[0]

        # A ------ B
        # | Object |
        # D ------ C

        output[0][0] = x_min
        output[0][1] = y_min

        output[1][0] = x_max
        output[1][1] = y_min

        output[2][0] = x_min
        output[2][1] = y_max

        output[3][0] = x_max
        output[3][1] = y_max
        return output

    def detections_to_custom_box(self, detections, im, im0):
        output = []
        for i, det in enumerate(detections):
            if len(det):
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh

                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh

                    # Creating ingestable objects for the ZED SDK
                    obj = sl.CustomBoxObjectData()
                    obj.bounding_box_2d = self.xywh2abcd(xywh, im0.shape)
                    obj.label = cls
                    obj.probability = conf
                    obj.is_grounded = False
                    output.append(obj)
        return output

    def camera_perception(self, weights, img_size, conf_thres=0.2, iou_thres=0.45):

        self.get_logger().info("Intializing Network...")

        device = select_device()
        half = device.type != 'cpu'  # half precision only supported on CUDA
        imgsz = img_size

        # Load model
        model = attempt_load(weights, device=device)  # load FP32
        stride = int(model.stride.max())  # model stride
        imgsz = check_img_size(imgsz, s=stride)  # check img_size
        if half:
            model.half()  # to FP16
        cudnn.benchmark = True

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

        self.get_logger().info("Network initialized")

        while rclpy.ok() and self.stop_camera_detection==False:
            if self.run_signal:
                self.lock.acquire()
                img, ratio, pad = self.img_preprocess(self.image_net, device, half, imgsz)

                pred = model(img)[0]
                det = non_max_suppression(pred, conf_thres, iou_thres)

                # ZED CustomBox format (with inverse letterboxing tf applied)
                self.detections = self.detections_to_custom_box(det, img, self.image_net)
                self.lock.release()
                self.run_signal = False
            sleep(0.01)

def main():
    rclpy.init()
    try:
        camera_node = Camera_node()
        rclpy.spin(camera_node, executor=MultiThreadedExecutor())
        # After finish close the camera
        camera_node.get_logger().info("normal finish")
        camera_node.zed.close()
        camera_node.destroy_node()
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