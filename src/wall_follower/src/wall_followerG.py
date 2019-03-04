#!/usr/bin/env python


import rospy
import util
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    VIZ_TOPIC = rospy.get_param("wall_follower/viz", "/viz")

    def __init__(self):
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.viz_pub = rospy.Publisher(self.VIZ_TOPIC, MarkerArray, queue_size=1)

    within = False

    def callback(self, msg):
        if self.SIDE < 1:
            polar_points = util.laser_scan_to_points(msg)
        else:
            polar_points = np.flip(util.laser_scan_to_points(msg),0)

        # polar_points = polar_points[5:(int)(np.size(polar_points,axis=0)*3/4)]

        # polar_points = polar_points[polar_points[:,0] > 0.1]

        rect_points = util.polar_to_cartesian(polar_points)
        rect_points = rect_points + np.array([0.275, 0])
        x, y = util.split_points(rect_points)
        # average_count = 3      # 5
        # rect_points = util.combine_points(util.moving_average(x,n=average_count),util.moving_average(y,n=average_count))

        path = util.generate_path_from_wall(rect_points, self.DESIRED_DISTANCE, 0)
        # lookahead = 0.8        # 0.42
        # if self.within:
        #    lookahead = 0.75    # 0.75
        # target, turning, self.within = util.pure_pursuit(path, lookahead)

        path_marker = util.create_cloud_msg(rect_points)
        # targer_marker = util.create_cloud_msg(target, color=(0,0,1,1), scale=(0.2,0.2,0.2), id=1)
        self.viz_pub.publish(MarkerArray(markers=[path_marker])) # , targer_marker

        drive = AckermannDriveStamped()
        drive.header.frame_id = "base_link"
        drive.drive.speed = self.VELOCITY
        drive.drive.steering_angle = 0 # turning
        drive.drive.steering_angle_velocity = 0.1
        self.drive_pub.publish(drive)

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
