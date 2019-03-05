#!/usr/bin/env python2

import numpy as np
import rospy
from scipy import stats

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyController:
    SCAN_TOPIC = rospy.get_param("safety_controller/scan_topic")
    NAV_TOPIC = rospy.get_param("safety_controller/nav_topic")
    SAFETY_TOPIC = rospy.get_param("safety_controller/safety_topic")
    SAFETY_MARGIN = rospy.get_param("safety_controller/safety_margin") # is our turning angle
    CAR_FRONT_RIGHT = rospy.get_param("safety_controller/car_front_right")
    CAR_FRONT_LEFT = rospy.get_param("safety_controller/car_front_left")
    ERROR = rospy.get_param("safety_controller/error")

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
	self.pub = rospy.Publisher(self.SAFETY_TOPIC, AckermannDriveStamped, queue_size=10)
	self.nav_sub = rospy.Subscriber(self.NAV_TOPIC, AckermannDriveStamped, callback = self.react_to_nav)
	self.scan_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, callback = self.react_to_scan)
	self.last_scan = None

    def react_to_scan(self, scan):
	self.last_scan = scan

    def react_to_nav(self, nav):
 	x, y = self.to_cartesian(self.last_scan.ranges, self.last_scan.angle_min, self.last_scan.angle_increment)
	if self.danger_in_box(x, y, nav.drive.steering_angle) and nav.drive.speed > 0:
	    self.stop()

    def to_cartesian(self, ranges, angle_min, angle_increment):
   	x = []
    	y = []

    	for i in range(len(ranges)):
            r = ranges[i]
            theta = angle_min + i*angle_increment
            x.append(r*np.cos(theta))
            y.append(r*np.sin(theta))
        return x,y

    def danger_in_box(self, x, y, steering_angle): 
	#TODO: integrate all in one nice formula thing where signs figure themselves out 
	if steering_angle < 0: #turning right	
	    left_line = lambda x, y : self.sign(0, 0, self.CAR_FRONT_LEFT[1], x, y)
	    right_line = lambda x, y : self.sign(np.tan(steering_angle), self.CAR_FRONT_RIGHT[0], self.CAR_FRONT_RIGHT[1], x, y)

	elif steering_angle > 0: #turning left
	    left_line = lambda x, y : self.sign(np.tan(steering_angle), -self.CAR_FRONT_LEFT[0], self.CAR_FRONT_LEFT[1], x, y)
	    right_line = lambda x, y : self.sign(0, 0, self.CAR_FRONT_RIGHT[1], x, y)

	else:
            left_line = lambda x, y : self.sign(0, 0, self.CAR_FRONT_LEFT[1], x, y)
	    right_line = lambda x, y : self.sign(0, 0, self.CAR_FRONT_RIGHT[1], x, y)
	
	front_line = lambda x, y : self.front(self.SAFETY_MARGIN, x, y)
	back_line = lambda x, y : self.front(0.2, x, y) 

	counter = 0	
	for i in range(len(x)):
            xi = x[i]
	    yi = y[i]
            if left_line(xi, yi)==-1 and right_line(xi, yi)==1 and front_line(xi, yi)==-1 and back_line(xi, yi)==1:
		counter +=1 

	    if counter >= self.ERROR:
		return True

	return False
	
    def front(self, k, x, y): 
	return 1 if x > k else -1

    def sign(self, m, s, k, x, y):
	fx = m*(x-s) + k
	return 1 if y > fx else -1	

    def stop(self):
		msg = AckermannDriveStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_link"
		msg.drive.steering_angle = 0
		msg.drive.steering_angle_velocity = 0
		msg.drive.speed = 0
		msg.drive.acceleration = 0
		msg.drive.jerk = 0  
		rospy.loginfo(msg)
		self.pub.publish(msg)
	

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    safety_controller = SafetyController()
    rospy.spin()
