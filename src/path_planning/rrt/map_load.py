import rospy
from nav_msgs.msg import OccupancyGrid
import pickle

class LoadedMap:
    def __init__(self, occupancy_grid_msg):
        self.grid = occupancy_grid_msg.data
        self.resolution = occupancy_grid_msg.info.resolution
        self.height = occupancy_grid_msg.info.height
        self.width = occupancy_grid_msg.info.width
        self.center = (occupancy_grid_msg.info.origin.position.x, occupancy_grid_msg.info.origin.position.y)

class ROS_Map_Loader():
    def __init__(self):
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

    def map_callback(self, message):
        ros_map = LoadedMap(message)
        file = open('stata_map.obj', 'w')
        pickle.dump(ros_map, file)

if __name__ == "__main__":
    rospy.init_node("RosMapLoader")
    ROS_Map_Loader()
    rospy.spin()