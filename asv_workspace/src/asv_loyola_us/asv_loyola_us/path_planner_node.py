import numpy as np
import matplotlib.pyplot as plt

import time

class Node():
	"""A node class for A* Pathfinding"""

	def __init__(self, parent=None, position=None):
		self.parent = parent
		self.position = position

		self.g = 0
		self.h = 0
		self.f = 0

	def __eq__(self, other):
		return self.position == other.position


def astar(maze, start, end, timeout=20):
	"""Returns a list of tuples as a path from the given start to the given end in the given maze"""

	# Create start and end node
	start_node = Node(None, start)
	start_node.g = start_node.h = start_node.f = 0
	end_node = Node(None, end)
	end_node.g = end_node.h = end_node.f = 0

	# Initialize both open and closed list
	open_list = []
	closed_list = []

	# Add the start node
	open_list.append(start_node)

	start_time = time.time()



	# Loop until you find the end
	while len(open_list) > 0 and time.time() - start_time < timeout:

		# Get the current node
		current_node = open_list[0]
		current_index = 0
		for index, item in enumerate(open_list):
			if item.f < current_node.f:
				current_node = item
				current_index = index

		# Pop current off open list, add to closed list
		open_list.pop(current_index)
		closed_list.append(current_node)

		# Found the goal
		if current_node == end_node:
			path = []
			current = current_node
			while current is not None:
				path.append(current.position)
				current = current.parent

			return np.asarray(path[::-1]) # Return reversed path

		# Generate children
		children = []
		for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

			# Get node position
			node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

			# Make sure within range
			if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
				continue

			# Make sure walkable terrain
			if maze[node_position[0]][node_position[1]] != 0:
				continue

			# Create new node
			new_node = Node(current_node, node_position)

			# Append
			children.append(new_node)

		# Loop through children
		for child in children:

			# Child is on the closed list
			for closed_child in closed_list:
				if child == closed_child:
					continue

			# Create the f, g, and h values
			child.g = current_node.g + 1
			child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
			child.f = child.g + 0.5*child.h

			# Child is already in the open list
			for open_node in open_list:
				if child == open_node and child.g > open_node.g:
					continue

			# Add the child to the open list
			open_list.append(child)

	return None

def is_line_free(p1, p2, nav_map):
	""" Check if the line between two points is free of obstacles """

	# Get the coordinates of the line
	x1, y1 = p1
	x2, y2 = p2


	# Get the x and y coordinates of the line
	dx = x2 - x1
	dy = y2 - y1

	dt = 100

	# Get the step size
	step_x = dx / dt
	step_y = dy / dt

	# Loop through the steps

	for t in range(dt):
		x = x1 + t * step_x
		y = y1 + t * step_y

		# Check if the point is inside an obstacle
		if nav_map[int(x), int(y)] == 1 or nav_map[int(np.ceil(x)), int(np.ceil(y))] == 1 or nav_map[int(np.ceil(x)), int(np.floor(y))] or nav_map[int(np.floor(x)), int(np.ceil(y))]: 
			return False
		
	return True


def reduce_path(path, nav_map):
	""" Reduce the path by removing unnecessary waypoints if between two waypoints there is a straight line without obstacles """

	# First and last points of the path
	p1 = path[0]
	p2 = path[-1]

	# New path
	new_path = [p1]

	# Loop through the path
	for i in range(len(path)-2):

		# Check if the line between p1 and p3 is free of obstacles
		p3 = path[i+2]
		if is_line_free(p1, p3, nav_map):
			# If it is free, then we can remove p2
			p2 = p3
		else:
			# If it is not free, then we need to add p2 to the path
			new_path.append(p2)
			p1 = p2
			p2 = p3

	# Add the last point
	new_path.append(p2)

	return new_path
	



if __name__ == '__main__':
   




	# start and goal position
	sx = 12  # [m]
	sy = 74  # [m]
	gx = 79 # [m]
	gy = 209  # [m]

	# set obstacle positions

	import sys

	sys.path.append(".")

	nav_map = np.genfromtxt("ASV_Loyola_US/asv_workspace/mapas/Alamillo95x216plantilla.csv", delimiter=" ")

	# Dilate the map using a 3x3 kernel ans scipy
	kernel = np.ones((3, 3))

	from scipy.ndimage import binary_dilation

	dil_nav_map = binary_dilation(nav_map, structure=kernel).astype(nav_map.dtype)

	# Reduce even more the dimensions of the map
	t0 = time.time()
	path = astar(dil_nav_map, (sx,sy), (gx,gy), timeout=5)
	t1 = time.time()
	print("Time elapsed: ", t1-t0)
	
	if path is None:
		print("No path found")
		plt.imshow(nav_map, cmap='Reds', origin='lower', alpha=nav_map)
		plt.imshow(dil_nav_map, cmap='Blues', origin='lower', alpha=0.5)
		plt.plot(sy, sx, "xr")
		plt.plot(gy, gx, "xb")
		plt.show()

	path = np.asarray(path)

	plt.imshow(nav_map, cmap='Reds', origin='lower', alpha=nav_map)
	plt.imshow(dil_nav_map, cmap='Blues', origin='lower', alpha=0.5)
	plt.plot(sy, sx, "xr")
	plt.plot(gy, gx, "xb")
	plt.plot(path[:,1], path[:,0], "o-r")
	

	reduced_path = reduce_path(path, nav_map)

	plt.plot([y for (x, y) in reduced_path], [x for (x, y) in reduced_path], "o-b", linewidth=3)

	plt.show()