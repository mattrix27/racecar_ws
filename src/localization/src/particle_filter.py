#!/usr/bin/env python2
from threading import Lock

import rospy
from sensor_model import SensorModel
from motion_model import MotionModel
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

import tf2_ros
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Vector3, Quaternion, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA


class ParticleFilter:

    def __init__(self):
        # self.odom_update_speed = rospy.get_param("~odom_update_speed")
        self.last_time = rospy.get_time()
        self.num_particles = rospy.get_param("~num_particles")
        self.particles = None
        self.last_pose = None
        self.path = None
        self.counter = 0

        self.probabilities = None
        self.lock = Lock()

        self.br = tf2_ros.StaticTransformBroadcaster()	
        
        self.listener = tf.TransformListener()

	# ROS Topics
        rospy.Subscriber(rospy.get_param("~odom_topic"), Odometry, self.odometry_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.click_callback, queue_size=1)

        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")
        self.delay_to_draw_path = rospy.get_param("~delay_to_draw_path")
        self.best_choice_alg = rospy.get_param("~best_choice_alg")

        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        # ROS Publishers
        self.point_cloud_pub = rospy.Publisher("/viz", Marker, queue_size=1)
        self.best_pose_pub = rospy.Publisher("/best_pose", Marker, queue_size=1)
        self.poses_pub = rospy.Publisher("/particles", PoseArray, queue_size=1)
        self.path_pub = rospy.Publisher("/path", Marker, queue_size=1)
        self.ct_error_pub = rospy.Publisher("/error_ct", Float64, queue_size=1)
        self.error_pub = rospy.Publisher("/error",Float64, queue_size=1)

	#self.publish_transform(np.array([0,0,0]))


    def click_callback(self, msg):
	print("CLICK")
        # Get click information
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        click = ((msg.pose.pose.position.x, msg.pose.pose.position.y), euler_from_quaternion(quaternion)[2])

	self.publish_transform(np.array([click[0][0],click[0][1],click[1]]))

	# Normal distribution around click
        particles = np.random.normal(0,rospy.get_param("~click_xy_sd"),size=(self.num_particles, 2)) + click[0]
        angles = np.random.normal(0, rospy.get_param("~click_t_sd"), size=(self.num_particles, 1)) * 2 * np.pi + click[1]

        self.lock.acquire()
        self.particles = np.hstack((particles, angles))
        self.lock.release()

        # Visualization
        self.publish_poses()

    def odometry_callback(self, msg):
        if self.last_pose is None:
            self.last_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z])
            self.last_time = rospy.get_time()
        if self.particles is not None:

            self.lock.acquire()
            odom = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])
            temp_time = rospy.get_time()
            odom *= temp_time - self.last_time
            self.particles = self.motion_model.evaluate(self.particles, odom)
            self.lock.release()
            
            self.last_time = temp_time

            # Visualization
            self.publish_poses()

    def scan_callback(self, msg):
#       return 
       if self.particles is not None:
            self.lock.acquire()
            ranges = np.array([msg.ranges])     # Lidar Data
            self.probabilities = self.sensor_model.evaluate(self.particles, ranges)

            # Resample
            temp = np.arange(0, self.particles.shape[0])
            indices = np.random.choice(temp, size=self.particles.shape[0], p=self.probabilities)
            self.particles = self.particles[indices]

            self.lock.release()

            # Calculate best pose
            best_particle = self.calculate_best_point()
            # Publish transform for best pose
            self.publish_transform(best_particle)

            # Visualization : Best pose marker
            self.publish_best_pose(best_particle)

            # Visualization : Path robot has been to
            if self.counter % self.delay_to_draw_path == 0:
                self.publish_path(best_particle)
                self.counter = 0
            self.counter += 1

            # Visualization : All particles with color probabilities
            self.publish_point_cloud()

            # Visualization : Poses
            self.publish_poses()

    def publish_poses(self):
        if self.poses_pub.get_num_connections() > 0:
            particle_msg = PoseArray()
            particle_msg.header.frame_id = "map"
            list_of_poses = []
            for particle_index in range(self.particles.shape[0]):
                temp_pose = Pose()
                temp_pose.position.x = self.particles[particle_index][0]
                temp_pose.position.y = self.particles[particle_index][1]
                quaternion = quaternion_from_euler(0, 0, self.particles[particle_index][2])
                temp_pose.orientation.x = quaternion[0]
                temp_pose.orientation.y = quaternion[1]
                temp_pose.orientation.z = quaternion[2]
                temp_pose.orientation.w = quaternion[3]
                list_of_poses.append(temp_pose)

            particle_msg.poses = list_of_poses
            self.poses_pub.publish(particle_msg)

    def publish_transform(self, particle):
	#print("Transforming",particle)
	t = TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "map"
	t.child_frame_id = self.particle_filter_frame
	t.transform.translation.x = particle[0]
	t.transform.translation.y = particle[1]
	t.transform.translation.z = 0
	q = quaternion_from_euler(0,0,particle[2])
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]
	
	#print("Transform sent!", particle)
	self.br.sendTransform(t)
        
        try:
            position, quaternion = self.listener.lookupTransform("base_link_pf",  "laser", rospy.Time())
            self.ct_error_pub.publish(Float64(abs(position[1])))
            self.error_pub.publish(Float64(np.sqrt(position[0]**2+position[1]**2)))
        except:
            pass

    def publish_path(self, best_particle):
        if self.path_pub.get_num_connections() > 0:
            if self.path is None:
                self.path = np.array([best_particle])
            else:
                self.path = np.vstack((self.path, np.array([best_particle])))
            self.path_pub.publish(self.create_cloud_msg(self.path, None, type=4))

    def publish_point_cloud(self):
        if self.point_cloud_pub.get_num_connections() > 0:
            point_cloud = self.create_cloud_msg(self.particles, self.probabilities)
            self.point_cloud_pub.publish(point_cloud)

    def publish_best_pose(self, best_particle):
        if self.best_pose_pub.get_num_connections() > 0:
            best_marker = Marker(
                type=7,
                id=0,
                lifetime=rospy.Duration(0),
                scale=Vector3(0.5, 0.5, 0.5),
                points=[Point(best_particle[0], best_particle[1], 0)],
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                ns="points",
                color=ColorRGBA(0, 1, 0, 1),
                header=Header(frame_id="map", stamp=rospy.Time.now()))
            self.best_pose_pub.publish(best_marker)

    def create_cloud_msg(self, points, probs, frame_id='map', id=0, scale=(0.1, 0.1, 0.1), type=7):
        if probs is not None:
            max_prob = np.max(probs)
            colors = [ColorRGBA(prob/max_prob, prob/max_prob, prob/max_prob, 1) for prob in probs]

            return Marker(
                type=type,  # LINE_STRIP
                id=id,
                lifetime=rospy.Duration(0),
                points=[Point(point[0], point[1], 0) for point in points],
                scale=Vector3(scale[0], scale[1], scale[2]),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                ns="points",
                colors=colors,
                header=Header(frame_id=frame_id, stamp=rospy.Time.now()))
        else:
            color = ColorRGBA(1,1,1,1)
            return Marker(
                type=type,  # LINE_STRIP
                id=id,
                lifetime=rospy.Duration(0),
                points=[Point(point[0], point[1], 0) for point in points],
                scale=Vector3(scale[0], scale[1], scale[2]),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                ns="points",
                color=color,
                header=Header(frame_id=frame_id, stamp=rospy.Time.now()))

    def calculate_best_point(self):
        if self.best_choice_alg == 0:
            best = np.sum(self.particles*np.array([self.probabilities]).T,axis=0)
        else:
            best = self.particles[np.argmax(self.probabilities)]
        return best

if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
