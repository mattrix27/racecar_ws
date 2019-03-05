#!/usr/bin/env python2

import numpy as np
import math
import rospy
from scipy import stats 

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


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
	self.subscan = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
	self.linemarker = rospy.Publisher("linemarker", Marker, queue_size=10)        
	self.pubdrive = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
	self.counter = 0

    def callback(self, data):
	drive = AckermannDriveStamped()
	drive.header.stamp = rospy.get_rostime()
	drive.header.frame_id = "base_link" 
	drive.drive.speed = self.VELOCITY
	
	ranges, angles = self.get_wall_stuff(data)	

	sortedA = np.sort(ranges)
	sortedA_small = sortedA[:15]
	middle_dist = np.median(sortedA_small)

	wallAngle, wallX, wallY = self.find_wall(middle_dist, angles, ranges)
	angle = self.PDcontroller(middle_dist, wallAngle)
	drive.drive.steering_angle = angle
	
	self.pubdrive.publish(drive)

    def find_wall(self, med_dist, angles, ranges):
	lineX = ranges[(ranges >= (med_dist -1.6)) & (ranges<= (med_dist+1.6))]
	ang_array2 = angles[(ranges >= (med_dist -1.6)) & (ranges<= (med_dist+1.6))]
#
	points = []
	wallX = []
	wallY = []
	for i, r in np.ndenumerate(lineX):
	    x = r*math.cos(ang_array2[i])
	    wallX.append(x)
	    y = r*math.sin(ang_array2[i])
	    wallY.append(y)
    	    points.append((x,y))
	slope, y_int, r, p, error = stats.linregress(points)
	angle = math.atan(slope)

	return angle, wallX, wallY

    def get_wall_stuff(self, data):
	range_array=np.array(data.ranges)
	a_min = data.angle_min
	a_max = data.angle_max
	a_increment = data.angle_increment
	a_list = [a_min]
	for i in range(0, len(data.ranges)):
	    a = a_list[-1] + a_increment
	    a_list.append(a)

	ang_array = np.array(a_list)

	if self.SIDE == 1:
	    ranges = range_array[len(range_array)//2-110:]
	    angles = ang_array[len(ang_array)//2-110:]
	else:
	    ranges = range_array[:(1000)]
	    angles = ang_array[:(1000)]

	return ranges, angles
	
    def PDcontroller(self, dist, wallAngle):
	feedback_P = .2
	feedback_D = .5
	e_t = (dist - self.DESIRED_DISTANCE)*self.SIDE
	e_diff = wallAngle 
	angle = feedback_P*e_t + feedback_D*e_diff

	return angle

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()


