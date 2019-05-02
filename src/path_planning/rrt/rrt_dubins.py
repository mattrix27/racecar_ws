#!/usr/bin/env python

import math
import random
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from nav_msgs.msg import OccupancyGrid
import dubins

### POSSIBLE TODO
## improve finding closest
## intermediate waypoints capability


## IMPORTANT TODO
#invert path


class Tree:
    def __init__(self, root):
        self.root = root
        self.vertices = np.array([root])
        self.child_to_par = {tuple(root): None}
        self.root_distances = {tuple(root): 0}
        self.current_pub = rospy.Publisher("/current_point", PoseArray, queue_size=1)

    def connect(self, parent, child):
        """
        add edge between parent and child
        """
        assert parent in self.vertices, "trying to connect to parent node that is not in graph yet"

        self.vertices = np.vstack((self.vertices, child))

        if tuple(child) in self.child_to_par:
            return #WHY?
        
        self.child_to_par[tuple(child)] = tuple(parent)
        self.root_distances[tuple(child)] = self.root_distances[tuple(parent)] + Utils.calc_distance(parent, child)
        self.current_pub.publish(RRT_planner().path_to_posearray(self.vertices.tolist()))

    def connect_chain(self, parent, chain):
        if len(chain) == 0:
            return

        self.connect(parent, chain[0])

        for index in range(1,len(chain)):
            self.connect(chain[index-1], chain[index])

    def backtrace(self, end):
        """
        compute path from root to end
        """
        print("BACKTRACE")
        path = []
        current = tuple(end)
        while current is not None:
            path.append(current)
            current = self.child_to_par[current]
            # print(current)

        print("FINISHED")

        return path, self.root_distances[tuple(end)]
        #TODO: reverse path

    def nearest(self, point):
        """
        find node/point in current graph that is closest to point
        """
        temp_vertices = self.vertices[:,:-1].copy()
        temp_point = point[:-1].copy()

        dist = np.linalg.norm(temp_vertices - temp_point, axis=1)
        best = np.argmin(dist)
        return {"node": self.vertices[best], "distance": dist[best]}

    def neighbors(self, point, range):
        """
        find nodes/points in current graph that are within range of point
        """
        temp_vertices = self.vertices[:,:-1].copy()
        temp_point = point[:-1].copy()

        dist = np.linalg.norm(temp_vertices - temp_point, axis=1)

        return self.vertices[dist <= range]

    def change_parent(self, point, new_parent):
        """
        change a node/point's parent in the graph
        """
        self.child_to_par[tuple(point)] = tuple(new_parent)

    def get_cost(self, point):
        """
        compute the distance between the root and the point
        """
        if tuple(point) not in self.root_distances:
            raise AssertionError("Trying to get cost of point that is not in graph")
        return self.root_distances[tuple(point)]

class Utils:
    def __init__(self):
        pass

    @staticmethod
    def calc_distance(start, end):
        """
        find the euclidean distance between start and end
        """
        temp_end = end[:-1].copy()
        temp_start = start[:-1].copy()

        return np.linalg.norm(temp_end - temp_start)

    @staticmethod
    def unit_vec(start, end):
        """
        compute the unit vector from start to end
        """
        distance = Utils.calc_distance(start, end)
        if distance == 0:
            return np.array([0,0])
        #print(end - start)
        return (end - start) / distance

    @staticmethod
    def generate_path_dubins(start, end, turning_radius, step_size):
        path = dubins.shortest_path(start, end, turning_radius)
        configurations, _ = path.sample_many(step_size)
        return [np.array(configuration) for configuration in configurations][1:]

class Map:
    def __init__(self, occupancy_grid_msg):

        #resolution in meters/pixel
        self.grid = occupancy_grid_msg.data
        self.np_grid = np.array(self.grid).reshape(occupancy_grid_msg.info.height,occupancy_grid_msg.info.width)
        self.grid = self.np_grid
        #print(self.grid[self.grid != -1].tolist())

        self.resolution = occupancy_grid_msg.info.resolution
        self.height = self.np_grid.shape[0] * self.resolution  #in meters
        self.width = self.np_grid.shape[1] * self.resolution #in meters
        self.dimensions = np.array(self.np_grid.shape) #in pixels
        self.center = (occupancy_grid_msg.info.origin.position.x, occupancy_grid_msg.info.origin.position.y) #(x,y)

    def discretize(self, point):
        ## IMPORTANT: input is Point(x,y) but returned value is Index(r,c)
        c =  int( (-point[0] + self.center[0]) / self.resolution)
        r =  int( (-point[1] + self.center[1]) / self.resolution)
        # print(point, (r,c), self.grid[r][c])
        return (r, c)


class RRT:
    def __init__(self, start, goal, occupancy_grid_msg, max_nodes=10000, step=0.2, range_to_goal=0.5, obstacle_resolution=0.25, greedy_rate=0.2, turning_radius=1):
        """
        :param start: Point: beginning of the path
        :param goal: Point: end of the path
        :param max_nodes: int: maximum number of vertices to explore in RRT
        :param step: length of increment when a new point gets added
        """
        self.start = start
        self.goal = goal
        self.K = max_nodes
        self.step = step
        self.graph = Tree(self.start)
        self.map = Map(occupancy_grid_msg)
        self.RANGE_TO_GOAL = range_to_goal
        self.OBSTACLE_RESOLUTION = obstacle_resolution
        self.NEIGHBOR_RANGE = 20
        self.GREEDY_RATE = greedy_rate
        self.TURNING_RADIUS = turning_radius

    def find_path(self, mode="RRT"):
        for i in range(self.K):
            qrand = self.rand()
            qnearest = self.graph.nearest(qrand)["node"]
            
            # qnew = self.new(qnearest, qrand) #is either a valid point that's at most step away from qnearest or is None (implementation)

            if mode == "RRT":
                self.rrt_extend_chain(qnearest, qrand)

            # if qnew_chain is not None:
                
                    # if self.close_enough(qnew): #early stopping because won't be further optimized
                    #     return self.graph.backtrace(qnew)

            if mode == "RRT*":
                self.rrt_star_extend(qnearest, qrand)

            if i % 100 == 0:
                print('Iteration = ' + str(i))

        goal_nearest = self.graph.nearest(self.goal)
        if goal_nearest["distance"] <= self.RANGE_TO_GOAL:
            return self.graph.backtrace(goal_nearest["node"])

        return None

    def rrt_extend(self, qnearest, qnew):
        self.graph.connect(qnearest, qnew)

    def rrt_extend_chain(self, qnearest, qrand):
        qnew_chain = self.new_dubins(qnearest, qrand)
        self.graph.connect_chain(qnearest, qnew_chain)

    def rrt_star_extend(self, qnearest, qrand):
        generated_path = self.new_dubins(qnearest, qrand)

        if len(generated_path) > 0:
            qnew = generated_path[-1]
        else:
            return

        qmin = qnearest
        qnears = self.graph.neighbors(qnew, self.NEIGHBOR_RANGE)
        min_cost = float('+inf')
        best_path = generated_path

        for qnear in qnears:
            temp_path = self.connect_dubins(qnear, qnew)
            if temp_path != None:
                cost = self.graph.get_cost(qnear) + len(temp_path)
                if cost <= min_cost:
                    qmin = qnear
                    best_path = temp_path
                    min_cost = cost # TODO confirm that this needs to be done

        if len(best_path) == 0:
            return

        self.graph.connect_chain(qmin, best_path)

        # for point in best_path:
        #     for qnear in qnears:
        #         temp_path = self.connect_dubins(point, qnear)
        #         if (qnear != point).any() and temp_path != None:
        #             current_cost = self.graph.get_cost(qnear)
        #             potential_cost = self.graph.get_cost(point) + len(temp_path)
        #             if potential_cost < current_cost:
        #                 print("YAY")
        #                 self.graph.change_parent(qnear, point)

    def close_enough(self, point):
        """
        determine whether point is close enough to the goal
        (i.e. path finding was successful)
        """
        return Utils.calc_distance(self.goal, point) <= self.RANGE_TO_GOAL

    def rand(self):
        """
        sample a random point in the space delimited by the map
        """
        #print(np.array([random.uniform(self.map.center[0], self.map.center[0] + self.map.width), random.uniform(self.map.center[1], self.map.center[1] + self.map.height)]))
        #print(self.map.center, self.map.width, self.map.height)
        if random.random() < self.GREEDY_RATE:
            return self.goal

        return np.array([random.uniform(self.map.center[0], self.map.center[0] - self.map.width), random.uniform(self.map.center[1], self.map.center[1] - self.map.height), random.uniform(-np.pi,np.pi)])

    def connect_dubins(self, qnearest, qtarget):
        path = Utils.generate_path_dubins(qnearest, qtarget, self.TURNING_RADIUS, self.OBSTACLE_RESOLUTION)

        # path_backwards = Utils.generate_path_dubins(qnearest-np.array([0,0,np.pi]), qtarget-np.array([0,0,np.pi]), self.TURNING_RADIUS, self.OBSTACLE_RESOLUTION)
        # path_backwards = [point + np.array([0,0,np.pi]) for point in path_backwards]

        # if len(path_backwards) < len(path):
        #     path = path_backwards

        for i in range(len(path)):
            (r,c) = self.map.discretize(path[i])
            if (0 <= r < self.map.dimensions[0] and 0 <= c < self.map.dimensions[1]) and not (0 <= self.map.grid[(r,c)] < 100):
                return None

        return path

    def new_dubins(self, qnearest, qrand):
        path = Utils.generate_path_dubins(qnearest, qrand, self.TURNING_RADIUS, self.OBSTACLE_RESOLUTION)

        # path_backwards = Utils.generate_path_dubins(qnearest-np.array([0,0,np.pi]), qrand-np.array([0,0,np.pi]), self.TURNING_RADIUS, self.OBSTACLE_RESOLUTION)
        # path_backwards = [point + np.array([0,0,np.pi]) for point in path_backwards]

        # if len(path_backwards) < len(path):
        #     path = path_backwards

        # #print(len(path))

        increment = self.step
        loop_count = int(np.ceil(increment / self.OBSTACLE_RESOLUTION))

        for i in range(min(loop_count + 1, len(path))):
            (r,c) = self.map.discretize(path[i])
            if (0 <= r < self.map.dimensions[0] and 0 <= c < self.map.dimensions[1]) and not (0 <= self.map.grid[(r,c)] < 100):
                return path[:i]

        return path[:i+1]

    def new(self, qnearest, qrand):
        increment = self.step
        loop_count = int(np.ceil(increment / self.OBSTACLE_RESOLUTION))

        unit = Utils.unit_vec(qnearest, qrand)

        if Utils.calc_distance(unit,np.array([0,0])) == 0:
            return None

        for i in range(1, loop_count + 1):
            point = qnearest + unit * i * self.OBSTACLE_RESOLUTION
            (r,c) = self.map.discretize(point)
            if (0 <= r < self.map.dimensions[0] and 0 <= c < self.map.dimensions[1]) and not (0 <= self.map.grid[(r,c)] < 100):
                #print("HIT!")
                return None

        if (qnearest + unit * increment < self.map.dimensions).all():
            #print("GOOD POINT!")
            return qnearest + unit * increment

        return None
        #TODO: or return something that's closest to what it should be, but within bounds

    def obstacle_free(self, qnearest, qrand):
        increment = self.step
        loop_count = int(np.ceil(increment / self.OBSTACLE_RESOLUTION))

        unit = Utils.unit_vec(qnearest, qrand)

        if Utils.calc_distance(unit,np.array([0,0])) == 0:
            return False # TODO IDK IF THIS IS RIGHT?

        for i in range(1, loop_count + 1):
            point = qnearest + unit * i * self.OBSTACLE_RESOLUTION
            (r,c) = self.map.discretize(point)
            if (0 <= r < self.map.dimensions[0] and 0 <= c < self.map.dimensions[1]) and not (0 <= self.map.grid[(r,c)] < 100):
                return False
        return True

class RRT_planner():
    def __init__(self):
        self.MAX_NODES = rospy.get_param("~max_nodes", 10000)
        self.RRT_STEP = rospy.get_param("~step", 0.2)
        self.RANGE_TO_GOAL = rospy.get_param("~range_to_goal", 0.5)
        self.OBSTACLE_RESOLUTION = rospy.get_param("~obstacle_resolution", 0.25)
        self.RRT_MODE = rospy.get_param("~rrt_mode", "RRT*")
        self.MAP_TOPIC = rospy.get_param("~map_topic", "/map")
        self.GREEDY_RATE = rospy.get_param("~greedy_rate", 0.2)

        #TODO: After integration with localization, this should probably listen to current pose instead
        self.map_sub = rospy.Subscriber(self.MAP_TOPIC, OccupancyGrid, self.map_callback, queue_size=1)
        self.start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=1)
        self.path_pub = rospy.Publisher("/planned_path", PoseArray, queue_size=1)

        self.start = None #type np.array([x,y])
        self.goal = None #type np.array([x,y])
        self.occupancy_grid = None
        

    def start_callback(self, msg):
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.start = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, euler_from_quaternion(quaternion)[2]])
        print(Map(self.occupancy_grid).discretize(self.start))

    def goal_callback(self, msg):
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.end = np.array([msg.pose.position.x, msg.pose.position.y, euler_from_quaternion(quaternion)[2]])
        #TODO: get map: self.map =

        self.plan_path()

    def plan_path(self):
        print(self.start, self.end)

        rrt = RRT(self.start, self.end, self.occupancy_grid, max_nodes=self.MAX_NODES, step=self.RRT_STEP, range_to_goal=self.RANGE_TO_GOAL, obstacle_resolution=self.OBSTACLE_RESOLUTION, greedy_rate=self.GREEDY_RATE)
        path, length = rrt.find_path(mode=self.RRT_MODE)
        path_msg = self.path_to_posearray(path)
        self.path_pub.publish(path_msg)

    def map_callback(self, message):
        self.occupancy_grid = message

    def path_to_posearray(self, path):
        path_msg = PoseArray()
        path_msg.header.frame_id = "map"
        list_of_poses = []
        for path_index in range(len(path)):
            temp_pose = Pose()
            temp_pose.position.x = path[path_index][0]
            temp_pose.position.y = path[path_index][1]
            #TODO does pure pursuit need thetas or not?
            quaternion = quaternion_from_euler(0, 0, path[path_index][2])
            temp_pose.orientation.x = quaternion[0]
            temp_pose.orientation.y = quaternion[1]
            temp_pose.orientation.z = quaternion[2]
            temp_pose.orientation.w = quaternion[3]
            list_of_poses.append(temp_pose)

        path_msg.poses = list_of_poses
        return path_msg

if __name__ == "__main__":
    rospy.init_node("rrt_planning")
    RRT_planner()
    rospy.spin()
    #grid_python =\
    #      [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #       [0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
    #       [0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
    #       [0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
    #       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    #grid = np.array(grid_python)
    #qnearest = np.array([0, 0])
    #end = np.array([0,8])

    #rrt = RRT(qnearest, end, grid, 10000, 0.5)
    #path, length = rrt.find_path(mode="RRT*")

    #def discretize(point):
    #    return (int(np.floor(point[0])), int(np.floor(point[1])))

    #for point in path:
    #    disc = discretize(point)
    #    if grid_python[disc[0]][disc[1]] == 1:
    #        print("PROBLEM")
    #    grid_python[disc[0]][disc[1]] = 3

    #print(np.array(grid_python))
    #print(length)
    # #
    #
    # vertices = np.array([[0,0], [1,1], [2,2]])
    # point = np.array([1.2, 1.2])
    # dist = np.linalg.norm(vertices - point, axis=1)
    # print(vertices[dist<0.4])
    #
