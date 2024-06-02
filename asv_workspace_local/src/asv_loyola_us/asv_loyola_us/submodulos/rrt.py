import numpy as np
import time
import math
from collections import defaultdict
import matplotlib.pyplot as plt
from matplotlib import collections as mc


class Tree:
	'''Define Tree Structure'''
	def __init__(self, init_node, goal_node):
		self.init_node = init_node
		self.goal_node = goal_node
		self.nodes = [init_node]
		self.parent_child = defaultdict(list)
		self.vertex_idx = {init_node: 0}
		self.edges = []
		self.samples = 40
		self.distance = {0: 0.}

	def RandomState(self):
		'''Returns random state'''
		rand_x, rand_y = np.random.randint(0, self.samples), np.random.randint(
			0, self.samples)
		return (rand_x, rand_y)

	def add_vertex(self, loc):
		try:
			idx = self.vertex_idx[loc]
		except:
			idx = len(self.nodes)
			self.nodes.append(loc)
			self.vertex_idx[loc] = idx
			self.parent_child[idx] = []
		return (idx)

	def add_edges(self, new_idx, idx, distance):
		self.edges.append((new_idx, idx))
		self.parent_child[new_idx].append((idx, distance))
		self.parent_child[idx].append((new_idx, distance))


def find_obstacle(map_grid):
	'''Returns obstacle in a list'''
	o_x, o_y = map_grid.nonzero()
	obstacle_list = list(zip(o_x, o_y))
	return obstacle_list


def euclidean(p1, p2):
	'''Calculates Euclidean distance'''
	return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def NearestState(rand_node, T, obstacle, map_grid):
	'''Returns the nearest vertex to the random node'''
	near_node = 0
	near_idx = 0
	min_dist = float("inf")
	if (map_grid[rand_node] != 1):
		for idx, v in enumerate(T.nodes):
			edist = euclidean(v, rand_node)
			if edist < min_dist:
				min_dist = edist
				near_node = v
				near_idx = idx
		return near_node, near_idx, min_dist


def intersection_line_sphere(obs, new_node, near_node, radius):
	'''Calculates the intersection between line and obstacle(sphere)'''
	direction = np.array(new_node) - np.array(near_node)
	dist = np.linalg.norm(direction)
	f = np.array(new_node) - np.array(obs)
	direction = direction / dist  
	a = np.dot(direction, direction)
	b = 2 * np.dot(direction, f)
	c = np.dot(f, f) - radius * radius

	discriminant = b * b - 4 * a * c
	if discriminant < 0:
		return False

	t1 = (-b + np.sqrt(discriminant)) / (2 * a)
	t2 = (-b - np.sqrt(discriminant)) / (2 * a)

	if (t1 < 0 and t2 < 0) or (t1 > dist and t2 > dist):
		return False

	return True


def calc_intersection(new_node, near_node, obstacle_list, radius=2):
	for obs in obstacle_list:
		intersection = intersection_line_sphere(obs, new_node, near_node,
												radius)
		if intersection:
			return True
	return False


def NewState(near_node, rand_node, delta, min_dist, obstacle_list):
	'''Returns new vertex in the direction of the random vertex'''
	if (min_dist >= delta):
		D = min_dist
		d = min(D, delta)
		new_x = near_node[0] + (d * (rand_node[0] - near_node[0]) / D)
		new_y = near_node[1] + (d * (rand_node[1] - near_node[1]) / D)
		new_node = int(new_x), int(new_y)
		if (map_grid[new_node] != 1 and not calc_intersection(
				new_node, near_node, obstacle_list, radius)):
			return (new_node)
	else:
		return rand_node


def generateRRT(init_node, goal_node, obstacles, iterations, delta):
	'''Generate RRT'''
	T = Tree(init_node, goal_node)
	for i in range(iterations):
		rand_node = T.RandomState()
		nearest_node_data = NearestState(rand_node, T, obstacles, map_grid)
		if nearest_node_data:
			near_node, near_idx, min_dist = nearest_node_data
			new_node = NewState(near_node, rand_node, delta, min_dist,
								obstacles)
			if new_node == None:
				continue
			new_idx = T.add_vertex(new_node)
			distance = euclidean(new_node, near_node)
			T.add_edges(new_idx, near_idx, distance)
			dist = euclidean(new_node, T.goal_node)
			if euclidean(new_node, goal_node) < delta:
				T.add_vertex(goal_node)
				x_goalidx = T.add_vertex(T.goal_node)
				T.add_edges(new_idx, x_goalidx, dist)
				break
	return T


def min_dist(nodes_list, distance):
	min_node = None
	for node in nodes_list:
		if min_node == None:
			min_node = node
		elif distance[node] < distance[min_node]:
			min_node = node

	return min_node


def dijkstra(T):
	'''Return path using Dijkstra '''
	nodes_list = list(T.parent_child.keys())
	init_idx = T.vertex_idx[init_node]
	goal_idx = T.vertex_idx[goal_node]
	distance = {}
	previous = {}
	for v in nodes_list:
		distance[v] = float('inf')
		previous[v] = None
	distance[init_idx] = 0

	while nodes_list:
		min_dist_node = min_dist(nodes_list, distance)
		nodes_list.remove(min_dist_node)
		if distance[min_dist_node] == float('inf'):
			break
		for neighbor, dist in T.parent_child[min_dist_node]:
			dist_to_currNode = distance[min_dist_node] + dist
			if dist_to_currNode < distance[neighbor]:
				distance[neighbor] = dist_to_currNode
				previous[neighbor] = min_dist_node
	path = []
	min_dist_node = goal_idx
	curNode = goal_idx
	while previous[min_dist_node] is not None:
		path.append(T.nodes[min_dist_node])
		min_dist_node = previous[min_dist_node]
		path.append(T.nodes[min_dist_node])
	return path[::-1]

def plot(G, obstacles, radius, path):
	'''
	Plot for RRT, obstacles and shortest path
	'''
	px = [x for x, y in G.nodes]
	py = [y for x, y in G.nodes]
	fig, ax = plt.subplots()

	for o_x, o_y in obstacles:
		plt.scatter(o_x, o_y, color='red')

	ax.scatter(px, py, c='cyan')
	ax.scatter(G.init_node[0], G.init_node[1], c='black')
	ax.scatter(G.goal_node[0], G.goal_node[1], c='black')

	lines = [(G.nodes[edge[0]], G.nodes[edge[1]]) for edge in G.edges]
	lc = mc.LineCollection(lines, colors='green', linewidths=2)
	ax.add_collection(lc)

	if path is not None:
		paths = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
		lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
		ax.add_collection(lc2)

	ax.autoscale()
	ax.margins(0.1)
	plt.show()

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

if __name__ == "__main__":

	init_node = (12, 74)
	goal_node = (16, 80)
	# map_grid = np.loadtxt('/home/samuel/ASV_loyola_us_repo/ASV_Loyola_US/asv_workspace/mapas/othermap.txt')
	

	map_grid = np.loadtxt('/home/samuel/ASV_loyola_us_repo/ASV_Loyola_US/asv_workspace/mapas/Alamillo95x216plantilla.csv')


	iterations = 1000
	delta = 3
	radius = 1
	obstacle_list = find_obstacle(map_grid)
	RRT = generateRRT(init_node, goal_node, obstacle_list, iterations, delta)
	path = dijkstra(RRT)

	path = reduce_path(path, map_grid)
	plot(RRT, obstacle_list, radius, path)