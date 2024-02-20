import rclpy
from rclpy.node import Node
from asv_interfaces.srv import PathPlanner
from asv_interfaces.msg import Location
#from .submodulos.astar import astar
#from .submodulos.astar import reduce_path

from .submodulos.dijkstra import Dijkstra
from .submodulos.dijkstra import reduce_path
import os
import numpy as np
from scipy.ndimage import binary_dilation

class PathPlannerNode(Node):

	def parameters(self):

		# Timeout to compute a path
		self.declare_parameter('path_planner_timeout', 20.0)
		self.path_planner_timeout = self.get_parameter('path_planner_timeout').get_parameter_value().double_value	
		# Base obstacle map path
		self.declare_parameter('navigation_map_path', 'mapas/Alamillo95x216')
		self.navigation_map_path = self.get_parameter('navigation_map_path').get_parameter_value().string_value
		# Debug mode
		self.declare_parameter('debug', True)
		self.debug = self.get_parameter('debug').get_parameter_value().bool_value

		# binary dilation
		self.declare_parameter('binary_dilation', 2)
		self.binary_dilation_size = self.get_parameter('binary_dilation').get_parameter_value().integer_value

		# Load the map

		self.navigation_map = np.genfromtxt(self.navigation_map_path + 'plantilla.csv', dtype=int, delimiter=' ')
		if self.binary_dilation_size > 0:
			self.dilated_navigation_map = binary_dilation(self.navigation_map, np.ones((self.binary_dilation_size, self.binary_dilation_size)))
		else:
			self.dilated_navigation_map = self.navigation_map

		grid_xy_to_lat_long = np.genfromtxt(self.navigation_map_path + 'grid.csv', delimiter=';', dtype=str)

		self.navigation_map_lat_long = np.zeros((2, *self.navigation_map.shape))

		# Transform the map to a float for long and lat
		for i in range(self.navigation_map.shape[0]):
			for j in range(self.navigation_map.shape[1]):
				self.navigation_map_lat_long[0, i, j] = float(grid_xy_to_lat_long[i, j].split(',')[0])
				self.navigation_map_lat_long[1, i, j] = float(grid_xy_to_lat_long[i, j].split(',')[1])
		
		# print(self.navigation_map)
					

	def declare_services(self):
		
		# Create a service to compute a path
		self.path_planner_service = self.create_service(PathPlanner, 'path_planner_service', self.path_planner_callback)

	def declare_topics(self):

		# Subscribe to a topic to receive new obstacles
		self.obstacles_subscriber = self.create_subscription(Location, 'detected_obstacles', self.obstacles_callback, 10)

	def __init__(self):
		super().__init__('path_planner_node')

		# Declare parameters, services and topics
		self.parameters()

		# Declare the dijkstra object
		self.dijkstra = Dijkstra(self.dilated_navigation_map, 1)

		self.declare_services()	
		self.declare_topics()
		


	def obstacles_callback(self, msg):

		# Take the lat long and search the closest point in the map
		lat = msg.lat
		long = msg.lon

		x_pos, y_pos = self.lat_long_to_xy(lat, long)

		self.get_logger().info('New obstacle updated at X: %d, Y: %d', x_pos, y_pos)

		# Update the map
		self.navigation_map[x_pos, y_pos] = 1

	def lat_long_to_xy(self, lat, long):

		# Find the closest point in the map
		print(lat)
		distance_lat = np.abs(self.navigation_map_lat_long[0, :, :] - lat)
		distance_long = np.abs(self.navigation_map_lat_long[1, :, :] - long)

		# Find position of minimum distance
		position_lat = np.argmin(distance_lat).astype(int)
		position_lat = np.unravel_index(position_lat, self.navigation_map.shape)[0]

		position_long = np.argmin(distance_long).astype(int)
		position_long = np.unravel_index(position_long, self.navigation_map.shape)[1]

		return position_lat, position_long
	
	def xy_to_lat_long(self, x, y):

		return self.navigation_map_lat_long[0, x, y], self.navigation_map_lat_long[1, x, y]

	def path_planner_callback(self, request, response):

		# Compute the path
		
		start = (request.origin.lat, request.origin.lon)
		goal = (request.destination.lat, request.destination.lon)

	
		start_xy = self.lat_long_to_xy(start[0], start[1])
		goal_xy = self.lat_long_to_xy(goal[0], goal[1])

		self.get_logger().info('New objetive updated: START: {}, GOAL: {}'.format(start_xy, goal_xy))

		response = PathPlanner.Response()
		solution = self.dijkstra.planning(start_xy, goal_xy, self.path_planner_timeout)

		self.get_logger().info('Solution obtained!')

		if solution is None:
			response.success = False
		else:

			response.success = True
			response.path.path = []
			solution = reduce_path(solution, self.dilated_navigation_map).astype(int)

			for point in solution:
				lat, long = self.xy_to_lat_long(point[0], point[1])
				response.path.path.append(Location(lat=lat, lon=long))
			
			response.path.path_length = len(response.path.path)
			
		return response	






def main(args=None):
	rclpy.init(args=args)
	node = PathPlannerNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
