#!/usr/bin/env python

import rospy
import numpy as np 




class SearchBasedPlanner()
	def _init_(self):
		#get inputs

		#create class parameters
		self.initital_pose = None
		self.goal_pose = None
		self.map = None
		#create subscribers and publishers

	def a_star(self,start_node, end_node):
		""" A* search
        	"""
		
		start = self.discretize(self.initial_pose)
		start_node = Node(start[0], start[1])
		to_expand = [start] 
		explored = set() 
		while len(to_expand) != 0:
		    cur_node = to_expand.pop()
		    explored.add(cur_node)
		    for node in self.get_neighbors(next_node):
		        #update nodes based on distance
		        if node in explored:
		            continue
		        if node.cost < cur_node.cost + self.cost(cur_node, node):
		            node.cost = cur_node.cost + self.cost(cur_node, node)

#	def bfs(start, goal, neighbors, collision, max_iterations=INF):
#	    if collision(start) or collision(goal):
#		return None
#	    iterations = 0
#	    visited = {tuple(start): Node(0, None)}
#	    expanded = []
#	    queue = deque([start])
#	    while len(queue) != 0 and iterations < max_iterations:
#		current = queue.popleft()
#		iterations += 1
#		expanded.append(current)
#		if goal is not None and tuple(current) == tuple(goal):
#		    return retrace(visited, current)
#		for next in neighbors(current):
#		    # TODO get edges betwwen neighbors for bfs
#		    if tuple(next) not in visited and not collision(next):
#		        visited[tuple(next)] = Node(
#		            next, visited[tuple(current)].g + 1, current)
#		        queue.append(next)
#	    return None

	
	def discretize(self, pose):
		pass
	def collision(self, pose, map):
		pass
	def get_neighbors(self, node):
		pass
	def path_planner(self):
		pass
	def visualizer(self):
		pass
	def map_callback(self, map_msg):

		print("Map Created")
