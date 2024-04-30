import queue
from tabnanny import verbose
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
import message_filters
import numpy as np
#import pyzed.sl as sl
import tensorrt
import cv2
import torch
import quaternion
import os
import utm
import time
from datetime import datetime
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO
from geographiclib.geodesic import Geodesic
import pandas as pd
class Custom_object_detection(Node):

    def __init__(self):
        super().__init__('custom_object_detection')
        queue_size = 1
        qos_profile_BEF=rclpy.qos.QoSProfile(
            depth=queue_size,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
            )
        qos_profile_REL=rclpy.qos.QoSProfile(
            depth=queue_size,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
            )
        
        self.depth_sub = message_filters.Subscriber(self, Image, '/zed/zed_node/depth/depth_registered', qos_profile=qos_profile_REL)
        self.camera_image_sub = message_filters.Subscriber(self, Image, '/zed/zed_node/left/image_rect_color',qos_profile=qos_profile_REL)        
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/zed/zed_node/left/camera_info',qos_profile=qos_profile_REL)
        tss = message_filters.TimeSynchronizer([self.camera_info_sub, self.camera_image_sub , self.depth_sub], queue_size=queue_size)
        tss.registerCallback(self.image_callback)
        # Create a subscription to the position and the compass heading of the ASV
        self.asv_position_subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)

        self.compass_hdg_subscription = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.compass_hdg_callback, qos_profile_REL)

        # self.drone_coordinates = self.create_subscription()
        self.trash_detections_publisher = self.create_publisher(String, '/camera_node/trash_detections', qos_profile_REL)
        
        timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0      
        np.bool = np.bool_ # Quick fix for some deprecation problems 
        self.geod = Geodesic.WGS84
        
        self.current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the folder containing the script
        self.weights = 'best_v8n_fp16'
        #self.weights = 'yolov5su_notrash'
        self.weights_format = '.engine'
        yolo_directory = os.path.join(self.current_directory,'utils','weights', self.weights+self.weights_format)
        #yolo_directory = os.path.join(self.current_directory,'utils','yolov5su.engine')
        self.model = YOLO(yolo_directory, task='detect')
        np.bool = np.bool_ # Quick fix for some deprecation problems
        self.imgsz = 1280
        if 'yolov5su_notrash' in yolo_directory:
            self.imgsz = 640
        self.model(np.zeros((640,640,3)), imgsz=self.imgsz)
        self.get_logger().info(yolo_directory)
        self.bridge = CvBridge()
        self.distance=None
        self.rotMat = r = R.from_rotvec(np.pi/2*np.array([1,0,1])) # from Right handed, y-down (image - default) to Right handed, z-up, x-forward (ROS)
        self.cv_image=None
        self.point_cloud_data=None
        self.depth_image=None
        self.drone_position = np.nan*np.ones((2,))
        self.compass_hdg = None        
        self.start_time = time.time()
        self.display_every = 1 # displays the frame rate every 1 second 
        self.counter = 0
        self.fps = None


        date = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")

        self.df = pd.DataFrame(columns=["Datetime","Class","Distance (m)","Latitude","Longitude"])
        self.df_path = f"{yolo_directory}/detection_logs/Detections_{date}.log"
        self.out_video = cv2.VideoWriter(f"{self.current_directory}/utils/videos/{date}_{self.weights}.avi",cv2.VideoWriter_fourcc(*'XVID'),10.0,(1280,720))

    def asv_position_callback(self, msg):

        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.get_logger().info(f'Received latitude {self.drone_position[0]} and longitude {self.drone_position[1]}')

    def compass_hdg_callback(self, msg):
        self.get_logger().info(f'Received compass heading {msg.data}')
        self.compass_hdg = msg.data

    def camera_info_callback(self, msg):
        # Almacenar la información de la cámara
        self.camera_info = msg
    
    def image_callback(self, camera_info_msg, camera_image_msg, depth_msg):

        self.camera_info = camera_info_msg # To retrieve calibration data
        self.cv_image = self.bridge.imgmsg_to_cv2(camera_image_msg, desired_encoding='bgr8') # left camera color image to pass YOLO

        # Get a pointer to the depth values casting the data pointer to floating point
        self.depth_image = memoryview(depth_msg.data).cast('f')
        self.width=depth_msg.width
        
        # Infer with YOLO
        self.get_logger().info(f"------------------------------------------------------------------------------\n")
        results = self.model.predict(self.cv_image, imgsz=self.imgsz, stream=True, conf=0.5, iou=0.5, verbose=False, half=True)

        result = None
        detected_classes = None
        bounding_boxes = None

        for result in results:
            detected_classes = [result.names[i] for i in result.boxes.cls.to('cpu').numpy()]
            bounding_boxes = result.boxes.xyxy.to('cpu').numpy()
            self.distance = self.calculate_distance(detected_classes, bounding_boxes)

            #self.draw_boxes(self.cv_image, result.xyxy[0])
            #cv2.imshow("Object Detection", result.plot(conf=False))
            for cls in self.distance.keys():
                self.get_logger().info(f"Class: {cls}, Distance: {self.distance[cls][0]} meters")
                #date = datetime.now().strftime("%Y/%m/%d-%H:%M:%S")
                #print(f"[{date}] Class: {cls}, Distance: {self.distance[cls][0]} meters")
            #cv2.waitKey(1)

            
            if self.compass_hdg is not None and not any(np.isnan(self.drone_position)):
                for cls in self.distance.keys():
                    #if np.isnan(self.distance[k]):
                    g = self.geod.Direct(self.drone_position[0], self.drone_position[1],self.compass_hdg,self.distance[cls][0])
                    msg = String()
                    msg.data = "The position of "+ cls + " is ({:.10f}), {:.10f}).".format(g['lat2'],g['lon2'])
                    date = datetime.now().strftime("%Y/%m/%d-%H:%M:%S")
                    #print(f"[{date}] Class: {cls}, Distance: {self.distance[cls][0]} meters." + " Position: ({:.10f}), {:.10f}).".format(g['lat2'],g['lon2']))
                    data = {'Datetime': date ,'Class': cls ,'Distance (m)': self.distance[cls][0],'Latitude':g['lat2'],'Longitude':g['lon2']}
                    dataframe2 = pd.DataFrame([data])
                    self.df = self.df.append(dataframe2)
                    self.trash_detections_publisher.publish(msg)
                self.df.to_csv(self.df_path, index=False)
        # Calculate the frame rate
        self.counter+=1
        if (time.time() - self.start_time) > self.display_every :
            self.fps = self.counter / (time.time() - self.start_time)
            #print("FPS: ", self.fps)
            self.counter = 0
            self.start_time = time.time()
        
        #self.draw_boxes(self.cv_image, results.xyxy[0])
        img_ = self.draw_boxes(detected_classes, bounding_boxes,result.plot(conf=False))
        cv2.imshow("Object Detection", cv2.resize(img_,(1280,720)))
        """for cls in self.distance.keys():
            self.get_logger().info(f"Class: {cls}, Distance: {self.distance[cls]} meters")"""
        cv2.waitKey(1)
        self.out_video.write(img_)
    def calculate_distance(self, detected_classes, bboxes): # returns a dict with the distance of each detection in meters
        if self.depth_image is None:
            return -1
        
        real_distance = {f"{cls_id}_{i}": np.nan*np.ones(3,) for i,cls_id in enumerate(detected_classes)}
        for i, cls in enumerate(real_distance):
            xmin, ymin, xmax, ymax= bboxes[i] 
            center_x = (xmin + xmax) // 2
            center_y = (ymin + ymax) // 2
            depth_image_np = np.array(self.depth_image)
            depth_value = depth_image_np[int(center_y * self.width + center_x)]

            Z = depth_value
            X = (center_x - self.camera_info.k[2]) * Z / (self.camera_info.k[0]) # from https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_components/src/zed_camera/src/zed_camera_component.cpp line 3067
            Y = (center_y - self.camera_info.k[5]) * Z / (self.camera_info.k[4])
            #real_dist, _ = self.real_coordinates(depth_value,center_x,center_y)
            real_distance[cls][0] = np.sqrt(X**2 + Y**2 + Z**2)
            real_distance[cls][1] = center_x
            real_distance[cls][2] = center_y
        
        return real_distance
    def calculate_closest_distance(self, detected_classes, bboxes):
        depth_image = self.depth.get_data()
        real_distance = {f"{cls_id}_{i}": np.nan*np.ones(3,) for i,cls_id in enumerate(detected_classes)}
        for i, cls in enumerate(real_distance):
            xmin, ymin, xmax, ymax= bboxes[i] 
            section = depth_image[int(ymin):int(ymax),int(xmin):int(xmax)]  # Extract the section
            
            proportion=1/5
            radius_y = proportion*(ymax-ymin)
            radius_x = proportion*(xmax-xmin)

            center_x = (xmin + xmax) // 2
            center_y = (ymin + ymax) // 2
            section = depth_image[int(center_y-radius_y):int(center_y+radius_y),int(center_x-radius_x):int(center_x+radius_x)]  # Extract the section
            #min_val = np.nanmin(section)  # Find the minimum value, ignoring NaNs
            #min_val = np.nanmean(section)  # Find the mean value, ignoring NaNs
            min_val = np.nanmedian(section)  # Find the median value, ignoring NaNs
            if False:
                # Flatten the depth values and remove NaNs
                section_flat = section.flatten()
                section_flat_clean = section_flat[~np.isnan(section_flat)]
                min_val = scipy.stats.trim_mean(section_flat_clean, proportiontocut=0.4)  # Trim 10% from both ends
            # min_index = np.where(section == min_val)  # Find the position of the minimum value
            # if min_index[0].size == 0 or min_index[1].size == 0:
            #     section_nonan = np.where(np.isnan(section), np.nanmax(section) + 1, section) # replace NaNs with the maximum value
            #     closest_index = np.abs(section_nonan - min_val).argmin()  # Find the index of the element closest to the target
            #     closest_index = np.unravel_index(closest_index, section.shape)
            #     center_y, center_x = (closest_index[0] + ymin, closest_index[1] + xmin)  # Adjust indices to global coordinates
            # else:
            #     min_index = np.where(section == min_val)  # Find the position of the minimum value
            #     center_y, center_x = (min_index[0][0] + ymin, min_index[1][0] + xmin)
            # Convert pixel coordinates to 3D point in camera frame (Right handed, z-up, x-forward (ROS))
            # Z = depth_image[int(center_y),int(center_x)]
            Z = min_val
            X = (center_x - self.center_left_x) * Z / (self.focal_left_x)
            
            Y = (center_y - self.center_left_y) * Z / (self.focal_left_y)
            point_camera_frame = np.array([X, Y, Z])
            distance = np.sqrt(X**2 + Y**2 + Z**2)
            #point_camera_shifted = self.rotMat.apply(point_camera_frame)
            #print("XYZ: {3} distance to Camera at ({0}, {1}): {2} m".format(center_x, center_y,distance ,cls), end="\n") 

            real_distance[cls][0] = distance 
            real_distance[cls][1] = center_x
            real_distance[cls][2] = center_y
        
        return real_distance
    def real_coordinates(self, z_distance, u, v):

        # Convert pixel coordinates to 3D point in camera frame (Right handed, z-up, x-forward (ROS))
        Z = z_distance
        X = (u - self.camera_info.k[2]) * Z / (self.camera_info.k[0]) # from https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_components/src/zed_camera/src/zed_camera_component.cpp line 3067
        Y = (v - self.camera_info.k[5]) * Z / (self.camera_info.k[4])
        point_camera_frame = np.array([X, Y, Z])

        point_camera_shifted = self.rotMat.apply(point_camera_frame)

        #self.get_logger().info("Point Center Camera: %s " %  str(point_camera_shifted))
        real_distance = np.sqrt(X**2 + Y**2 + Z**2)
        
        return real_distance, point_camera_shifted
    def draw_boxes(self, detected_classes, bboxes,img):
        if detected_classes is not None:
            for i, cls in enumerate(detected_classes):
                xmin, ymin, xmax, ymax= bboxes[i]
                #cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 5)
                #cv2.putText(self.img, f'Class: {cls}', (int(xmin), int(ymin) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                center_x = int((xmin + xmax) / 2)
                center_y = int((ymin + ymax) / 2)
                center_x = int(self.distance[f"{cls}_{i}"][1])
                center_y = int(self.distance[f"{cls}_{i}"][2])
                    # Dibujar un círculo en el centro del cuadro de detección
                cv2.circle(img, (center_x, center_y), 8, (255, 0, 0), -1)
                
                # Dibujar las coordenadas del centro del cuadro de detección
                #cv2.putText(img, f'({center_x}, {center_y})', (center_x, center_y + 15), 
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(img, f'Distance {self.distance[f"{cls}_{i}"][0]:.2f}', (int(xmin), int(ymin) +20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if self.fps is not None:
            cv2.putText(img, f'FPS: {self.fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return img
def main(args=None):
    rclpy.init(args=args)

    custom_object_detection = Custom_object_detection()

    rclpy.spin(custom_object_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    custom_object_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
