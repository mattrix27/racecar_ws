#!/usr/bin/env python
import rospy
import numpy as np
import time
import tf
import tf.transformations
import utils

from geometry_msgs.msg import PolygonStamped, Point
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        #srospy.logerr("WHYYYYYYY")
        self.trajectory_topic = rospy.get_param("~trajectory_topic")
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = rospy.get_param("~lookahead")
        self.speed            = float(rospy.get_param("~speed"))
        self.wrap             = bool(rospy.get_param("~wrap"))
        self.wheelbase_length = float(rospy.get_param("~wheelbase"))
        #self.drive_topic      = rospy.get_param("~drive_topic")
        self.drive_topic      = "/drive"

        self.ct_error_topic   = "/ct_error"
        self.error            = "/error"     
        
        self.trajectory       = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub         = rospy.Subscriber(self.trajectory_topic, PolygonStamped, self.trajectory_callback, queue_size=1)
        self.odom_sub         = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        self.drive_pub        = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=10)
        self.viz_pub          = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.viz_pub1         = rospy.Publisher("/visualization_marker1", Marker, queue_size=10)
        self.viz_pub2         = rospy.Publisher("/visualization_marker2", Marker, queue_size=10)

        self.ct_error_pub     = rospy.Publisher(self.ct_error_topic, Float64, queue_size=1)
        self.error_pub        = rospy.Publisher(self.error, Float64, queue_size=1)

        self.trajectoryRecieved = False
        self.lookahead = 2

        self.good = 0.0
        self.drive_on = 0.0
        
        self.listener = tf.TransformListener()
        self.priorCross = None
        self.end = False
        '''
        Insert code here
        '''
    def odom_callback(self, msg):
        #assuming odometrey data is x,y
        if self.trajectoryRecieved:
            point = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            cur_angle = self.quaternion_to_angle(msg.pose.pose.orientation)
            #something relating to speed
            next_point = self.find_lookahead(point, self.lookahead)
            marker1 = self.visualize(next_point[0], next_point[1], point[0], point[1], msg.header)
            #marker2 =self.visualize(next_point[1][0], next_point[1][1], point[0], point[1])
            self.viz_pub.publish(marker1)
            #self.viz_pub1.publish(marker2)
            self.visualize_radius(point[0], point[1], msg.header)           
            #should i pass in next_point - current_point?
            #angle = self.pure_pursuit(next_point[0]-point, next_point[1]-point) - cur_angle
            angle = self.pure_pursuit(next_point,msg.header)
            
            self.drive(angle,self.speed)  

            #rospy.logerr("WOOT")

            self.speed = (self.speed + (1/abs(angle)*20))/2.0 - 1.0

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.polygon.points), "points" 
        rospy.logerr("RECEIVING")
        self.trajectory.clear()
        self.trajectory.fromPolygon(msg.polygon)
        self.trajectory.make_np_array()
        self.trajectory.publish_viz(duration=0.0)
        self.trajectoryRecieved = True
        #rospy.logerr(self.trajectory.points)

    def find_lookahead(self,current_point, distance):
    	#When getting from Generation
    	#points = self.trajectory.np_points[::2]
    	#Else
    	points = self.trajectory.np_points
        start_points = points[:-1]
        end_points = points[1:]
        past = 0
        intersection_found = False
        point_found = False
        if self.priorCross:
        	if self.priorCross > 1:
        		distance = distance/2
        start_index, closest_point = self.find_closest_segment(start_points, end_points, current_point)
        while start_index < len(end_points) and past < 5:
            start = start_points[start_index]
            end = end_points[start_index]
            if np.linalg.norm(end - current_point) > distance:
                possible_intersection = self.find_intersection(start,end,current_point, distance)
                intersection_found = possible_intersection[0]
                if intersection_found:
                	next_point = possible_intersection[1]
            start_index += 1
            if intersection_found or point_found:
            	past+=1
            	point_found = True

        if point_found:
            return next_point
        else:
            #not sure what to do at the moment just drive to closest
            #rospy.logerr("YOOOOO, where da CIRCLE!!")          
            #return [closest_point,closest_point]
            return closest_point

    def find_closest_segment(self, start,end,cur_pos):
        #could do with with projections idk if theres a way to do in np
        #return start_index corresponding to closest point and the closest point
        # x = (-start+cur_pos)
        # y = np.linalg.norm(-start+cur_pos,axis=1)
        # rospy.logerr(x.shape)
        # rospy.logerr(y)

        # check_point1 = np.divide(x,y)
        # check_point2 = np.divide((end-start),np.linalg.norm(end-start,axis=1))
        # check_point3 = np.dot(check_point1,check_point2)
        # check_end = np.arccos(np.dot(np.divide((-start+cur_pos),np.linalg.norm(-start+cur_pos,axis=1)),(end-start)/np.linalg.norm(end-start,axis=1)))
        # if check_end > np.pi/2:
        #     distances = np.linalg.norm(-start+cur_pos, axis=1)
        # elif np.arccos(np.dot((-end+cur_pos)/np.linalg.norm(-end+cur_pos,axis=1),(start-end)/np.linalg.norm(start-end,axis=1))) > np.pi/2:
        #     distances = np.linalg.norm(-end+cur_pos, axis=1)
        # else:

        # distances = np.cross(end-start, start-cur_pos)/np.linalg.norm(end-start,axis=1)
        # min_index = np.argmin(distances)
        # v = start[min_index]-end[min_index]     
        # point = start[min_index] + np.dot(cur_pos - start[min_index],v) / np.dot(v,v) * (v)
        # return min_index, point
        vecs = end - start #vectors of the segments
        #rospy.logerr(vecs)
        proj = np.sum((cur_pos - start) * vecs, axis=1) / np.sum(vecs * vecs, axis=1) #projection of point to each segment

        proj_in_range = np.clip(proj, 0., 1.).reshape((-1,1)) #clips out of feasible range values and shapes for getting points

        points = vecs * proj_in_range + start

        distances = np.linalg.norm(points - cur_pos, axis=1)

        closest_index = np.argmin(distances)

        error = distances[closest_index]
        self.error_pub.publish(error)
        # if error < 0.1:
        # 	self.good += 1.0
        # self.drive_on += 1.0	
        
        closest_point = points[closest_index]
        if closest_point[0] == end[-1][0] and closest_point[1]==end[-1][1]:
        	self.end = True
        return closest_index, closest_point

    #From page linked to in lab
    def find_intersection(self,p1,p2,center_circle, distance):
        Q = center_circle # car's position
        r = distance# the look ahead distnace
        v = p2 - p1

        a = np.dot(v,v)
        b = 2*np.dot(v,p1 - Q)
        c = np.dot(p1,p1) + np.dot(Q,Q) - 2*np.dot(p1,Q) - r**2

        disc = b**2 - 4*a*c
        if disc < 0:
            return False,None

        sqrt = np.sqrt(disc)
        t1 = (-b + sqrt)/(2*a)
        t2 = (-b - sqrt)/(2*a)

        if (0<= t1 <= 1 and 0<= t2 <= 1):
            t_1 = max(0, min(1,t1))
            t_2 = max(0, min(1,t2))
            #return True, [p1 + t_1*v, p1 + t_2*v]

            #rospy.logerr("AHHHH THERES TWO INTERSECTIONS!!!")

            pt1 = p1 + t_1*v
            pt2 = p1 + t_2*v
            return True, pt1

        elif (0<= t1 <= 1):
            t = max(0, min(1,t1))

            #rospy.logerr("T1")

            #return True, [p1 + t*v, p1 + t*v]
            return True, p1 + t*v

        elif (0<=t2<=1):
            t = max(0, min(1,t2))

            #rospy.logerr("T2")

            #return True, [p1 + t*v, p1 + t*v]
            return True, p1 + t*v
        
        else:
            return False, None

    def pure_pursuit(self, next_point,header):
        point = self.transform_points(next_point[0],next_point[1],header)
        if self.priorCross:
        	adjust = self.priorCross/10
        else:
        	adjust = 0
        sign = np.sign(point[0])
        ct_error = point[1]
        #self.ct_error_pub.publish(ct_error)

        L = self.wheelbase_length
        n = np.arctan2(point[1], point[0] - sign*adjust)
        L_fw = np.linalg.norm(point)
        l_fw = self.wheelbase_length/3
        angle_num = L*np.sin(n)
        angle_denom = L_fw/2+l_fw*np.cos(n)
        #WHY NOT NEGATIVE???
        self.priorCross = next_point[1]
        return np.arctan(angle_num/angle_denom)

    def visualize(self, x, y, x2, y2, header=None):
        marker = Marker()

        if header:
                marker.header = header
        else:
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.id = 0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.color.r = 1.0
        marker.points = [Point(x2,y2,0), Point(x,y,0)]

        return marker

    def visualize_radius(self, x, y, header=None):
        marker = Marker()
        if header:
                marker.header = header
        else:
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = 0
        marker.scale.x = self.lookahead*2
        marker.scale.y = self.lookahead*2
        marker.scale.z = self.lookahead
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        self.viz_pub2.publish(marker)

    def drive(self, steering_angle, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0
        msg.drive.speed = speed

        msg.drive.acceleration = 0
        msg.drive.jerk = 0
        rospy.loginfo(msg)
        self.drive_pub.publish(msg)

    def quaternion_to_angle(self,q):
        """Convert a quaternion _message_ into an angle in radians.
        The angle represents the yaw.
        This is not just the z component of the quaternion."""
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
        return yaw
    
    def transform_points(self,x,y,header):
    	#x,y,z,theta
		matrix = self.listener.asMatrix('/base_link', header)
		point = [x,y,0,1]
		newPoint = np.dot(matrix,point)
		return [newPoint[0], newPoint[1]]
        
if __name__=="__main__":
    rospy.loginfo("WORK PLZ")
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
    ''' 
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #rospy.logerr("WE STARTING")        
        pf.follow_trajectory()
        rate.sleep()
    '''
