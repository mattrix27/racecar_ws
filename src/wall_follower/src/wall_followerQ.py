#!/usr/bin/env python2

import numpy as np
import rospy
from scipy import stats
import util

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=5)
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, callback=self.react_to_scan)
        self.rate = rospy.Rate(40)  # todo check what scan rate is
        self.errors = [0, 0, 0]
        self.integral = 0
        self.deriv = 0

    def react_to_scan(self, scan):
        Kfront = 3
        wall_ranges, front_ranges = self.get_wall_ranges(scan)
        side_error = self.get_error(wall_ranges, scan.angle_min, scan.angle_increment)
        front_error = self.get_front_error(front_ranges, scan.angle_min, scan.angle_increment)
        error = side_error + Kfront * front_error
        self.basic_controller(error)

    def basic_controller(self, error):
        Kp = 0.35
        Ki = 0.0
        Kd = 0.2
        self.errors.append(error)
        self.deriv = (self.errors[-1] - self.errors[-2]) / 3 + (self.errors[-2] - self.errors[-3]) / 3 + (
                    self.errors[-3] - self.errors[-4]) / 3
        self.integral = self.integral + self.errors[-1]
        angle = self.SIDE * (Kp * error + Kd * self.deriv + Ki * self.integral)
        self.publish_angle(angle)

    def slice_up_scan(self, scan):
        # assert scan.angle_min < 0
        # front_index = int(round((0.0 - scan.angle_min) / (scan.angle_increment)))
        minus_90 = int(round((-np.pi / 2 - scan.angle_min) / scan.angle_increment))
        plus_90 = int(round((np.pi / 2 - scan.angle_min) / scan.angle_increment))
        front_index = len(scan.ranges) / 2
        right = scan.ranges[minus_90:front_index]
        left = scan.ranges[front_index:plus_90]
        front = scan.ranges[front_index - len(scan.ranges) / 8: front_index + len(scan.ranges) / 8]
        return right, left, front

    def get_wall_ranges(self, scan):
        right, left, front = self.slice_up_scan(scan)
        if self.SIDE == 1.0:  # follow left wall
            return left, front
        else:  # follow right wall
            return right, front

    def to_cartesian(self, ranges, angle_min, angle_increment):
        x = []
        y = []

        for i in range(len(ranges)):
            r = ranges[i]
            theta = angle_min + i * angle_increment
            x.append(r * np.cos(theta))
            y.append(r * np.sin(theta))
        return x, y

    def get_error(self, ranges, angle_min, angle_increment):
        dist_to_wall = self.get_dist_to_wall(ranges, angle_min, angle_increment)
        return dist_to_wall - self.DESIRED_DISTANCE

    def get_front_error(self, ranges, angle_min, angle_increment):
        front_dist = self.get_dist_to_wall(ranges, angle_min, angle_increment)
        return min(front_dist - 2, 0)

    def get_dist_to_wall(self, ranges, angle_min, angle_increment):
        x, y = self.to_cartesian(ranges, angle_min, angle_increment)
        slope, intercept, _, _, _ = stats.linregress(x, y)
        dist_to_wall = np.abs(intercept) / np.sqrt(1 + slope ** 2)
        return dist_to_wall

    def publish_angle(self, angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = angle
        msg.drive.steering_angle_velocity = 0
        msg.drive.speed = self.VELOCITY
        msg.drive.acceleration = 0
        msg.drive.jerk = 0
        rospy.loginfo(msg)
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
