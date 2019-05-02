import random
import numpy as np


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

    def connect(self, parent, child):
        """
        add edge between parent and child
        """
        assert parent in self.vertices, "trying to connect to parent node that is not in graph yet"

        self.vertices = np.vstack((self.vertices, child))
        self.child_to_par[tuple(child)] = tuple(parent)
        self.root_distances[tuple(child)] = self.root_distances[tuple(parent)] + Utils.calc_distance(parent, child)

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
        return path, self.root_distances[tuple(end)]
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
        return (end - start) / distance

class Map:
    def __init__(self, occupancy_grid_msg):
        #resolution in meters/pixel
        self.grid = [ [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 100, 100, 100, 100, 0, 0, 0],
                      [0, 0, 0, 100, 100, 100, 100, 0, 0, 0],
                      [0, 0, 0, 100, 100, 100, 100, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
        self.np_grid = np.array(self.grid)
        self.grid = self.np_grid
        self.resolution = 1
        self.height = self.np_grid.shape[0] * self.resolution  #in meters
        self.width = self.np_grid.shape[1] * self.resolution #in meters
        self.dimensions = np.array(self.np_grid.shape) #in pixels
        self.center = (-5, 5) #(x,y)

    def discretize(self, point):
        ## IMPORTANT: input is Point(x,y) but returned value is Index(r,c)
        c =  int( (point[0] - self.center[0]) / self.resolution)
        r =  int( (point[1] - self.center[1]) / self.resolution)
        return (r, c)

class RRT:
    def __init__(self, start, goal, occupancy_grid_msg, max_nodes, step):
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
        self.RANGE_TO_GOAL = 0.1
        self.OBSTACLE_RESOLUTION = 1
        self.NEIGHBOR_RANGE = 0.3

    def find_path(self, mode="RRT"):
        for i in range(self.K):
            qrand = self.rand()
            qnearest = self.graph.nearest(qrand)["node"]
            qnew = self.new(qnearest, qrand) #is either a valid point that's at most step away from qnearest or is None (implementation)

            if qnew is not None:
                if mode == "RRT":
                    self.rrt_extend(qnearest, qnew)
                    if self.close_enough(qnew): #early stopping because won't be further optimized
                        return self.graph.backtrace(qnew)

                if mode == "RRT*":
                    self.rrt_star_extend(qnearest, qnew)

            if i % 1000 == 0:
                print('Iteration = ' + str(i))

        goal_nearest = self.graph.nearest(self.goal)
        if goal_nearest["distance"] <= self.RANGE_TO_GOAL:
            return self.graph.backtrace(goal_nearest["node"])

        return None

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
        return np.array([random.uniform(self.map.center[0], self.map.center[0] + self.map.width), random.uniform(self.map.center[1] - self.map.height, self.map.center[1])])

    def new(self, qnearest, qrand):
        increment = self.step
        loop_count = int(np.ceil(increment / self.OBSTACLE_RESOLUTION))

        unit = Utils.unit_vec(qnearest, qrand)

        for i in range(1, loop_count):
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

        for i in range(1, loop_count):
            point = qnearest + unit * i * self.OBSTACLE_RESOLUTION
            (r,c) = self.map.discretize(point)
            if (0 <= r < self.map.dimensions[0] and 0 <= c < self.map.dimensions[1]) and not (0 <= self.map.grid[(r,c)] < 100):
                return False
        return True

def test_plot_simple():
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    start = np.array([-4, 4])
    end = np.array([4,-4])
    nodes = 10000
    step = 1

    rrt = RRT(start, end, "useless", nodes, step)
    path, length = rrt.find_path(mode="RRT*")
    print(path)
    fig, ax = plt.subplots(1)

    xs = [c[0] for c in path]
    ys = [c[1] for c in path]

    rect = patches.Rectangle((-2, -1), 4, 3, color='black', hatch='x', fill=False)

    ax.add_patch(rect)
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    plt.plot(start[0], start[1], marker='o', markersize=6, color="green")
    plt.plot(end[0], end[1], marker='o', markersize=6, color="red")
    plt.plot(xs, ys)
    plt.show()


if __name__ == "__main__":
    test_plot_simple()
