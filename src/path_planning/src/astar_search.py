#!/usr/bin/env python

import numpy as np
import Queue as q
import math
import random
import rospy
import json
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose, PointStamped, PolygonStamped, Point32
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from nav_msgs.msg import OccupancyGrid

#TODO: PUT IN PUBLISHERS AND SUBCRIBERS AT SOME POINT
#/subscribe to /initial_pose
#TODO make code take in an occupancy grid message during integration

class astarGraph:
    #create a graph for the A-star algorithm
    def __init__(self,diluted_map):
        #need to look at format of map in order to convert to graph later
        #map is just a grid of 1s and 0's
        self.dmap = diluted_map
        self.edges =None


    def get_neighbors(self, pose):
        #maybe find a way to get non grid neighbors
        n = []
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
            nx = pose[0] + dx
            ny = pose[1] + dy
            if nx < 0 or nx > np.shape(self.dmap)[1]-1 or ny < 0 or ny > np.shape(self.dmap)[0]-1:
                continue
            n.append(float('%.5f'%(nx)), float('%.5f'%(ny)))


        return n

    def heuristic(self, current_pose, end_pose):
        #distance heuristic
        dx, dy = (abs(current_pose[0] - end_pose[0]), abs(current_pose[1] - end_pose[1]))
        return ((dx**2 + dy**2)**.5)
        # consider a different distance metric instead of euclidean, mannhattan, chess if grid?
        #return np.linalg.norm(end_pose[0]-current_pose[0],end_pose[1]-current_pose[1])


    def collision(self, current_pose):
        return self.dmap[current_pose] == 1

    def collision_cost(self, current_pose, goal_pose):
        #cost is sum of edges
        if self.collision(goal_pose):
            # want large constant to check all other paths first
            # return len(self.dmap)
            return 50
        else:
            return 1

class searchPlanner:
    #how do I initialize the diluted map here
    def __init__(self):
        #TODO: Create subscribers : initial_pose and the /map topic of type occupancy grid (google occupancy grid ros) create a map callback method (for this Subscriber)
        rospy.logerr("A STAR")
        self.start_pose = None
        self.end_pose = None
        self.set_start = True
        self.map = None

        self.MAP_TOPIC = rospy.get_param("~map_topic", "/map")
        self.FINAL_TRAJECTORY_TOPIC = rospy.get_param("~trajectory_topic", "/trajectory/current")

        self.map_sub = rospy.Subscriber(self.MAP_TOPIC, OccupancyGrid, self.map_callback, queue_size=1)
        self.start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_callback, queue_size=1)
        #self.waypoint_sub = rospy.Subscriber("/clicked_point", PointStamped, self.waypoint_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=1)
        self.path_pub = rospy.Publisher("/planned_path", PoseArray, queue_size=1)
        self.final_trajectory_pub = rospy.Publisher(self.FINAL_TRAJECTORY_TOPIC, PolygonStamped, queue_size=1)

        self.waypoints = None #type np.array([[x,y], [x,y]])
        #self.start = None #type np.array([x,y])
        #self.goal = None #type np.array([x,y])
        self.occupancy_grid = None

    def map_callback(self, message):
        rospy.logerr("MAP REC IEVED")
        self.map = message
        self.asgraph = astarGraph(self.map)

    def goal_callback(self, goal_msg):
        rospy.logerr("ENDING, MAKE PATH NOW")
        self.end_pose = (goal_msg.pose.position.x, goal_msg.pose.position.y)
        came_from, cost_so_far = self.aStarPlanner(self.start_pose, self.end_pose)    
        rospy.logerr(came_from)
        path = self.reconstruct_path(came_from,self.start_pose, self.end_pose)
        posearray = self.path_to_posearray(path)
        self.path_pub.publish(posearray)

    def clicked_callback(self, point_msg):
            rospy.logerr("STARTING")
            if self.set_start:
                self.start_pose = (point_msg.pose.pose.position.x, point_msg.pose.pose.position.y)
                self.set_start = False
                #self.show_start() #place markers
            # else:
            #     self.end_pose = (point_msg.point.x, point_msg.point.y)
            #     self.set_start = True
            #     self.show_end() #place markers
            #     self.aStarPlanner(self.asgraph, self.start_pose, self.end_pose)

    def aStarPlanner(self,start_pose, end_pose):
        heuristic_graph =self.asgraph
        cstart = float('%.8f'%(start_pose[0])), float('%.8f'%(start_pose[1]))

        start = tuple(cstart)

        frontier = q.PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
           current = frontier.get()

           for next in heuristic_graph.get_neighbors(current):
              ropsy.logerr("CHECKING NEIGHBORS")
              new_cost = cost_so_far[current] + heuristic_graph.collision_cost(current, next)
              if next not in cost_so_far or new_cost < cost_so_far[next]:
                 cost_so_far[next] = new_cost
                 rospy.logerr("AHHHHHHHH")
                 priority = float('%.8f'%(new_cost + heuristic_graph.heuristic(end_pose, next)))
                 frontier.put(next, priority)
                 cnext = float('%.8f'%(next[0])),float('%.8f'%(next[1]))
                 came_from[cnext] = current


        return came_from, cost_so_far

    def reconstruct_path(self, came_from, start, goal):
        current = (float('%.8f'%(goal[0])), float('%.8f'%(goal[1])))
        path = []
        print(came_from)
        while current != start:
            path.append(current)
            curr = tuple(current)
            current = came_from[curr]
        path.append(start) # optional
        path.reverse() # optional
        return path


    def path_to_posearray(self, path):
        path_msg = PoseArray()
        path_msg.header.frame_id = "map"
        list_of_poses = []
        for path_index in range(len(path)):
            temp_pose = Pose()
            temp_pose.position.x = path[path_index][0]
            temp_pose.position.y = path[path_index][1]
            #TODO does pure pursuit need thetas or not?
            quaternion = quaternion_from_euler(0, 0, 0)
            temp_pose.orientation.x = quaternion[0]
            temp_pose.orientation.y = quaternion[1]
            temp_pose.orientation.z = quaternion[2]
            temp_pose.orientation.w = quaternion[3]
            list_of_poses.append(temp_pose)

        path_msg.poses = list_of_poses
        return path_msg

    def path_to_polygon(self, path):
        path_msg = PolygonStamped()
        path_msg.header.frame_id = "/map"
        path_msg.header.stamp = rospy.Time.now()
        list_of_poses = []
        for path_index in range(len(path)):
            temp_pose = Point32()
            temp_pose.x = path[path_index][0]
            temp_pose.y = path[path_index][1]
            temp_pose.z = -1
            list_of_poses.append(temp_pose)

        path_msg.polygon.points = list_of_poses
        return path_msg

#if __name__=="__main__":

    #test_map =  np.array([
#            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#            [0, 0, 0, 0, 1, 1, 1, 1, 0, 0],
#            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#           [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#           [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
#
#   graph = astarGraph(test_map)
#   planner = searchPlanner(test_map)
#    result, cost = planner.aStarPlanner((0,0), (5,5))
#    path = planner.reconstruct_path(result,(0,0),(5,5))
#    print(path)

#!/usr/bin/env python

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
        self.visualize = rospy.get_param("~viz", False)

    def connect(self, parent, child):
        """
        add edge between parent and child
        """
        assert parent in self.vertices, "trying to connect to parent node that is not in graph yet"

        if tuple(parent) == tuple(child):
            return

        self.vertices = np.vstack((self.vertices, child))
        self.child_to_par[tuple(child)] = tuple(parent)
        self.root_distances[tuple(child)] = self.root_distances[tuple(parent)] + Utils.calc_distance(parent, child)
        if self.visualize:
            self.current_pub.publish(RRT_planner().path_to_posearray(self.vertices.tolist()))

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
        return path[::-1], self.root_distances[tuple(end)]
        #TODO: reverse path

    def nearest(self, point):
        """
        find node/point in current graph that is closest to point
        """
        dist = np.linalg.norm(self.vertices - point, axis=1)
        best = np.argmin(dist)
        return {"node": self.vertices[best], "distance": dist[best]}

    def neighbors(self, point, range):
        """
        find nodes/points in current graph that are within range of point
        """
        dist = np.linalg.norm(self.vertices - point, axis=1)
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
        return np.linalg.norm(end - start)

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
        c =  int( -1*(-point[0] + self.center[0]) * self.resolution)
        r =  int( -1*(-point[1] + self.center[1]) * self.resolution)
        # print(point, (r,c), self.grid[r][c])
        return (r, c)


class RRT:
    def __init__(self, start, goal, occupancy_grid_msg, max_nodes=10000, step=0.2, range_to_goal=0.5, obstacle_resolution=0.25, greedy_rate=0.2):
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
        self.NEIGHBOR_RANGE = step * 1.5
        self.GREEDY_RATE = greedy_rate

    def find_path(self, mode="RRT"):
        
        pathplanner = searchPlanner(self.map.grid)
        result, cost = pathplanner.aStarPlanner(self.start, self.goal)
        path = pathplanner.reconstruct_path(result,self.start, self.goal)
        return path
    

    
    def rrt_extend(self, qnearest, qnew):
        self.graph.connect(qnearest, qnew)

    def rrt_star_extend(self, qnearest, qnew):
        qmin = qnearest
        qnears = self.graph.neighbors(qnew, self.NEIGHBOR_RANGE)
        min_cost = float('+inf')

        for qnear in qnears:
            if self.obstacle_free(qnear, qnew):
                cost = self.graph.get_cost(qnear) + Utils.calc_distance(qnear, qnew)
                if cost <= min_cost:
                    qmin = qnear
                    min_cost = cost # TODO confirm that this needs to be done

        self.graph.connect(qmin, qnew)

        for qnear in qnears:
            if (qnear != qmin).any() and self.obstacle_free(qnew, qnear):
                current_cost = self.graph.get_cost(qnear)
                potential_cost = self.graph.get_cost(qnew) + Utils.calc_distance(qnew, qnear)
                if potential_cost < current_cost:
                    self.graph.change_parent(qnear, qnew)

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

        return np.array([random.uniform(self.map.center[0], self.map.center[0] - self.map.width), random.uniform(self.map.center[1], self.map.center[1] - self.map.height)])

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
        self.FINAL_TRAJECTORY_TOPIC = rospy.get_param("~trajectory_topic", "/trajectory/current")

        #TODO: After integration with localization, this should probably listen to current pose instead
        self.map_sub = rospy.Subscriber(self.MAP_TOPIC, OccupancyGrid, self.map_callback, queue_size=1)
        self.start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_callback, queue_size=1)
        self.waypoint_sub = rospy.Subscriber("/clicked_point", PointStamped, self.waypoint_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=1)
        self.path_pub = rospy.Publisher("/planned_path", PoseArray, queue_size=1)
        self.final_trajectory_pub = rospy.Publisher(self.FINAL_TRAJECTORY_TOPIC, PolygonStamped, queue_size=1)

        self.waypoints = None #type np.array([[x,y], [x,y]])
        self.start = None #type np.array([x,y])
        self.goal = None #type np.array([x,y])
        self.occupancy_grid = None

    def start_callback(self, msg):
        self.start = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        print(Map(self.occupancy_grid).discretize(self.start))

    def waypoint_callback(self, msg):
        point = np.array([msg.point.x, msg.point.y])
        if self.waypoints is None:
            self.waypoints = point
        else:
            self.waypoints = np.vstack((self.waypoints, point))

    def goal_callback(self, msg):
        self.end = np.array([msg.pose.position.x, msg.pose.position.y])
        #TODO: get map: self.map =

        self.plan_path()

    def plan_path(self):
        print(self.start, self.end)

        if self.waypoints is None:
            rrt = RRT(self.start, self.end, self.occupancy_grid, max_nodes=self.MAX_NODES, step=self.RRT_STEP, range_to_goal=self.RANGE_TO_GOAL, obstacle_resolution=self.OBSTACLE_RESOLUTION, greedy_rate=self.GREEDY_RATE)
            path, length = rrt.find_path(mode=self.RRT_MODE)
            path_msg = self.path_to_posearray(path)
            self.path_pub.publish(path_msg)

        else:
            self.waypoints = np.vstack((self.waypoints, self.end)) #add the goal as a final waypoint for easy looping
            current_start = self.start
            total_path = []
            total_length = 0
            for i in range(self.waypoints.shape[0]):
                current_end = self.waypoints[i, :]
                rrt = RRT(current_start, current_end, self.occupancy_grid, max_nodes=self.MAX_NODES, step=self.RRT_STEP,
                          range_to_goal=self.RANGE_TO_GOAL, obstacle_resolution=self.OBSTACLE_RESOLUTION,
                          greedy_rate=self.GREEDY_RATE)
                partial_path, partial_length = rrt.find_path(mode=self.RRT_MODE)
                total_path.extend(partial_path[1:])
                total_length += partial_length
                current_start = np.array(total_path[-1])  #end of current subpath is beginning of next subpath
            path_msg = self.path_to_posearray(total_path)
            self.path_pub.publish(path_msg)

            trajectory = self.path_to_polygon(total_path)

            # self.save(trajectory, "/home/matthew/racecar_ws/src/path_planning/trajectories/GeoffreyMap2.traj")
            self.final_trajectory_pub.publish(trajectory)
            print("PUBLISHED!")

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
            quaternion = quaternion_from_euler(0, 0, 0)
            temp_pose.orientation.x = quaternion[0]
            temp_pose.orientation.y = quaternion[1]
            temp_pose.orientation.z = quaternion[2]
            temp_pose.orientation.w = quaternion[3]
            list_of_poses.append(temp_pose)

        path_msg.poses = list_of_poses
        return path_msg

    def path_to_polygon(self, path):
        path_msg = PolygonStamped()
        path_msg.header.frame_id = "/map"
        path_msg.header.stamp = rospy.Time.now()
        list_of_poses = []
        for path_index in range(len(path)):
            temp_pose = Point32()
            temp_pose.x = path[path_index][0]
            temp_pose.y = path[path_index][1]
            temp_pose.z = -1
            list_of_poses.append(temp_pose)

        path_msg.polygon.points = list_of_poses
        return path_msg

    def save(self, points, path):
        print "Saving trajectory to:", path
        data = {}
        data["points"] = []
        for p in points.polygon.points:
            data["points"].append({"x": p.x, "y": p.y})
        with open(path, 'w') as outfile:
            json.dump(data, outfile)

if __name__ == "__main__":
    rospy.init_node("rrt_planning")
    searchPlanner()
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
