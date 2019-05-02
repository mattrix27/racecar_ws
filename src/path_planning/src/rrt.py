#!/usr/bin/env python

import rospy
import numpy as np
import time

from nav_msgs.msg import *

class RRT(object):
	def __init__(self):
            getMap = rospy.ServiceProxy('get_map', GetMap)
            print(getMap())

if __name__=="__main__":
	rospy.init_node("rrt")
	pf = RRT()
	rospy.spin()
