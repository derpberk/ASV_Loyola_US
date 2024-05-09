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
from ament_index_python.packages import get_package_share_directory
import utm
import time
from datetime import datetime
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO
from geographiclib.geodesic import Geodesic
import pandas as pd
from asv_interfaces.msg import TrashMsg
class Custom_object_detection(Node):
	
	def parameters(self):

		# Debug mode
		self.declare_parameter('debug', True)
		self.debug = self.get_parameter('debug').get_parameter_value().bool_value

		# Record a video of the detections
		self.declare_parameter('record_video', True)
		self.record_video = self.get_parameter('record_video').get_parameter_value().bool_value	
		self.declare_parameter('recorded_video_path', "utils/videos/")
		self.recorded_video_path = self.get_parameter('recorded_video_path').get_parameter_value().string_value	

		# Show the images of the detections
		self.declare_parameter('show_detections', False)
		self.show_detections = self.get_parameter('show_detections').get_parameter_value().bool_value	

		# Show the images of the detections
		self.declare_parameter('save_logs', True)
		self.save_logs = self.get_parameter('save_logs').get_parameter_value().bool_value	
		self.declare_parameter('logs_path', "utils/detection_logs/")
		self.logs_path = self.get_parameter('logs_path').get_parameter_value().string_value	

		# Weights parameters
		self.declare_parameter('weights_folder_path', "utils/weights/")
		self.weights_folder_path = self.get_parameter('weights_folder_path').get_parameter_value().string_value	

		self.declare_parameter('weights_name', "best_v8n_fp16.engine")
		self.weights_name = self.get_parameter('weights_name').get_parameter_value().string_value	

		self.declare_parameter('conf', 0.5)
		self.conf = self.get_parameter('conf').get_parameter_value().double_value	

		self.declare_parameter('iou', 0.5)
		self.iou = self.get_parameter('iou').get_parameter_value().double_value	

		self.declare_parameter('image_size', 1280)
		self.imgsz = self.get_parameter('image_size').get_parameter_value().integer_value	

		self.declare_parameter('savelogs_timer_period', 1.0) # 0.5  # seconds
		self.savelogs_timer_period = self.get_parameter('savelogs_timer_period').get_parameter_value().double_value	
	
	def declare_topics(self):

		queue_size = 1
		qos_profile_BEF=rclpy.qos.QoSProfile(
			depth=queue_size,
			reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
			history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
			)
		qos_profile_REL=rclpy.qos.QoSProfile(
			depth=queue_size,
			reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
			history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
			durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
			)

		self.depth_sub = message_filters.Subscriber(self, Image, '/zed/zed_node/depth/depth_registered', qos_profile=qos_profile_REL)
		self.camera_image_sub = message_filters.Subscriber(self, Image, '/zed/zed_node/left/image_rect_color',qos_profile=qos_profile_REL)        
		self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/zed/zed_node/left/camera_info',qos_profile=qos_profile_REL)
		tss = message_filters.TimeSynchronizer([self.camera_info_sub, self.camera_image_sub , self.depth_sub], queue_size=queue_size)
		tss.registerCallback(self.image_callback)
		# Create a subscription to the position and the compass heading of the ASV
		self.asv_position_subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)

		self.compass_hdg_subscription = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.compass_hdg_callback, qos_profile_BEF)

		# self.drone_coordinates = self.create_subscription()
		self.trash_detections_publisher = self.create_publisher(TrashMsg, '/zed2i_trash_detections/trash_localization', qos_profile_REL)
		
	def __init__(self):
		super().__init__('custom_object_detection_node')

		self.parameters()

		self.declare_topics()  

		np.bool = np.bool_ # Quick fix for some deprecation problems 
		self.geod = Geodesic.WGS84
		
		self.current_directory = get_package_share_directory("zed2i_camera")
		#self.current_directory = setup_tools.find_package("zed2i_camera")
		yolo_directory = os.path.join(self.current_directory,self.weights_folder_path,self.weights_name)
		self.get_logger().info(f"Loading weights from {yolo_directory}")
		self.model = YOLO(yolo_directory, task='detect')
		np.bool = np.bool_ # Quick fix for some deprecation problems

		self.model(np.zeros((640,640,3)), imgsz=self.imgsz)
		self.get_logger().info("Weights Loaded Succesfully")

		self.bridge = CvBridge()
		self.distance=None
		self.cv_image=None
		self.point_cloud_data=None
		self.depth_image=None
		self.drone_position = np.nan*np.ones((2,))
		self.compass_hdg = None  
		self.proportion_of_bb = 1/5      

		# fps
		self.start_time = time.time()
		self.display_every = 1 # displays the frame rate every 1 second 
		self.counter = 0
		self.fps = None


		date = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
		if self.save_logs:
			self.df_list = []
			self.df = pd.DataFrame(columns=["Datetime","Class","Distance (m)","Drone Lat","Drone Lon","Drone Heading","Object Lat","Object Lon","Object Heading"]) # self.current_directory
			if not os.path.exists(self.logs_path): 
				os.makedirs(self.logs_path) 
			self.df_path = os.path.join(self.logs_path,f"Detections_{date}.log")

		if self.record_video:
			weights, _ = self.weights_name.split(".")
			if not os.path.exists(self.recorded_video_path): 
				os.makedirs(self.recorded_video_path) 
			video_name = os.path.join(self.recorded_video_path,f"{date}_{weights}.avi")
			self.out_video = cv2.VideoWriter(video_name,cv2.VideoWriter_fourcc(*"MJPG"),10.0,(self.imgsz,self.imgsz))
			self.out_video_imgs = []
			os.chmod(video_name, 0o777)

		
		self.timer = self.create_timer(self.savelogs_timer_period, self.savelogs_timer_callback)

	def savelogs_timer_callback(self):
		if self.save_logs:
			self.save_logs_fnc()

	def save_logs_fnc(self):
		#self.get_logger().info(f'{self.df_list}')
		self.df = pd.DataFrame(self.df_list, columns=["Datetime","Class","Distance (m)","Drone Lat","Drone Lon","Drone Heading","Object Lat","Object Lon","Object Heading"]) # self.current_directory
		self.df.to_csv(self.df_path, index=False)

	def save_video_fnc(self):
		for img in self.out_video_imgs:
			self.out_video.write(img)

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
		#self.depth_image = memoryview(depth_msg.data).cast('f')
		self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")  # encoding: 32FC1
		self.width=depth_msg.width
		
		# Infer with YOLO
		self.get_logger().info(f"------------------------------------------------------------------------------")
		results = self.model.predict(self.cv_image, imgsz=self.imgsz, stream=True, conf=self.conf, iou=self.iou, verbose=False, half=True)

		result = None
		detected_classes = None
		bounding_boxes = None

		for result in results:
			detected_classes = [result.names[i] for i in result.boxes.cls.to('cpu').numpy()]
			bounding_boxes = result.boxes.xyxy.to('cpu').numpy()
			self.distance = self.calculate_closest_distance(detected_classes, bounding_boxes)

			for cls in self.distance.keys():
				self.get_logger().info(f"Class: {cls}, Distance: {self.distance[cls][0]} meters, Object Heading: {self.distance[cls][3]}")

			
			if self.compass_hdg is not None and not any(np.isnan(self.drone_position)):
				for cls in self.distance.keys():
					hdg_object = self.compass_hdg + self.distance[cls][3]
					g = self.geod.Direct(self.drone_position[0], self.drone_position[1],hdg_object,self.distance[cls][0])
					#msg = String()
					#msg.data = "The position of "+ cls + " is ({:.10f}), {:.10f}).".format(g['lat2'],g['lon2'])
					date = datetime.now().strftime("%Y/%m/%d-%H:%M:%S")

					if self.save_logs:
						self.df_list.append([date, cls, self.distance[cls][0],*self.drone_position, self.compass_hdg, g['lat2'], g['lon2'], hdg_object])
						#data = {'Datetime': date ,'Class': cls ,'Distance (m)': self.distance[cls][0],'Drone Position':self.drone_position , 'Latitude':g['lat2'],'Longitude':g['lon2']}
						#dataframe2 = pd.DataFrame([data])
						#self.df = self.df.append(dataframe2)

					msg = TrashMsg()
					msg.drone_lat = self.drone_position[0]
					msg.drone_lon = self.drone_position[1]
					msg.drone_heading = self.compass_hdg
					msg.object_lat = g['lat2']
					msg.object_lon = g['lon2']
					msg.object_heading = hdg_object
					msg.date = date
					#msg.cls = cls
					msg.success = True
     
					self.trash_detections_publisher.publish(msg)

				# if self.save_logs:
				# 	self.df = pd.DataFrame(self.df_list, columns=["Datetime","Class","Distance (m)","Drone Position","Drone Heading","Object Position","Object Heading"])
				# 	self.df.to_csv(self.df_path, index=False)

		# Calculate the frame rate
		self.counter+=1
		if (time.time() - self.start_time) > self.display_every :
			self.fps = self.counter / (time.time() - self.start_time)
			#print("FPS: ", self.fps)
			self.counter = 0
			self.start_time = time.time()
		
		if self.show_detections or self.record_video:
			img_ = self.draw_boxes(detected_classes, bounding_boxes,result.plot(conf=False))
			if self.show_detections:
				cv2.imshow("Object Detection", cv2.resize(img_,(1280,720)))
				cv2.waitKey(1)

		if self.record_video:
			#self.out_video.write(cv2.resize(img_,(self.imgsz,self.imgsz)))
			self.out_video_imgs.append(cv2.resize(img_,(self.imgsz,self.imgsz)))

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
			real_distance[cls][0] = np.sqrt(X**2 + Y**2 + Z**2)
			real_distance[cls][1] = center_x
			real_distance[cls][2] = center_y
		
		return real_distance
	def calculate_closest_distance(self, detected_classes, bboxes):
		
		real_distance = {f"{cls_id}_{i}": np.nan*np.ones(4,) for i,cls_id in enumerate(detected_classes)}
		for i, cls in enumerate(real_distance):
			xmin, ymin, xmax, ymax= bboxes[i] 
			
			radius_x, radius_y = self.dimension_of_bb(xmin, xmax, ymin, ymax)

			center_x = (xmin + xmax) // 2
			center_y = (ymin + ymax) // 2
			section = self.depth_image[int(center_y-radius_y):int(center_y+radius_y),int(center_x-radius_x):int(center_x+radius_x)]  # Extract the section
			#min_val = np.nanmin(section)  # Find the minimum value, ignoring NaNs
			#min_val = np.nanmean(section)  # Find the mean value, ignoring NaNs
			min_val = np.nanmedian(section)  # Find the median value, ignoring NaNs

			Z = min_val
			X = (center_x - self.camera_info.k[2]) * Z / (self.camera_info.k[0]) # from https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_components/src/zed_camera/src/zed_camera_component.cpp line 3067
			Y = (center_y - self.camera_info.k[5]) * Z / (self.camera_info.k[4])
			
			distance = np.sqrt(X**2 + Y**2 + Z**2)

			point_XZ = np.array([X,0,Z])
			z_axis = np.array([0,0,1])
			
			real_distance[cls][0] = distance 
			real_distance[cls][1] = center_x
			real_distance[cls][2] = center_y
			real_distance[cls][3] = np.degrees(np.arcsin(X/np.linalg.norm(point_XZ))) #self.angle_between_vectors(point_XZ,z_axis)
		
		return real_distance

	def draw_boxes(self, detected_classes, bboxes,img):
		if detected_classes is not None:
			for i, cls in enumerate(detected_classes):
				xmin, ymin, xmax, ymax= bboxes[i]

				center_x = int((xmin + xmax) / 2)
				center_y = int((ymin + ymax) / 2)
				center_x = int(self.distance[f"{cls}_{i}"][1])
				center_y = int(self.distance[f"{cls}_{i}"][2])
				
				# Dibujar un círculo en el centro del cuadro de detección
				cv2.circle(img, (center_x, center_y), 8, (255, 0, 0), -1)
				
				radius_x, radius_y = self.dimension_of_bb(xmin, xmax, ymin, ymax)
				cv2.rectangle(img, (int(center_x-radius_x), int(center_y-radius_y)), (int(center_x+radius_x), int(center_y+radius_y)), (0, 255, 0), 1)
				cv2.putText(img, f'Distance {self.distance[f"{cls}_{i}"][0]:.2f}', (int(xmin), int(ymin) +20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
		if self.fps is not None:
			cv2.putText(img, f'FPS: {self.fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
		return img

	def dimension_of_bb(self,xmin, xmax, ymin, ymax):
		return self.proportion_of_bb*(xmax-xmin), self.proportion_of_bb*(ymax-ymin)

	@staticmethod
	def angle_between_vectors(v1, v2):
		# Calculate dot product
		dot_product = np.dot(v1, v2)
		
		# Calculate magnitudes of vectors
		mag_v1 = np.linalg.norm(v1)
		mag_v2 = np.linalg.norm(v2)
		
		# Calculate cosine of the angle between the vectors
		cos_angle = dot_product / (mag_v1 * mag_v2)
		
		# Convert cosine to angle in radians
		angle_rad = np.arccos(np.clip(cos_angle, -1.0, 1.0))
		
		# Convert angle from radians to degrees
		angle_deg = np.degrees(angle_rad)
		
		return angle_deg

def main(args=None):
	rclpy.init(args=args)

	custom_object_detection = Custom_object_detection()
	try:
		rclpy.spin(custom_object_detection)
	except:
		custom_object_detection.get_logger().info("Shutting Down!")
		if custom_object_detection.save_logs:
			custom_object_detection.save_logs_fnc()
			custom_object_detection.get_logger().info("Logs saved")
		if custom_object_detection.record_video:
			custom_object_detection.save_video_fnc()
			custom_object_detection.out_video.release()
			custom_object_detection.get_logger().info("Video saved")
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	custom_object_detection.destroy_node()
	rclpy.shutdown()
	# try:
	# 	rclpy.spin(custom_object_detection)
	# except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):

	# 	custom_object_detection.get_logger().info("on shutdown!!!!")
	# finally:
	# 	rclpy.try_shutdown()

if __name__ == '__main__':
	main()
