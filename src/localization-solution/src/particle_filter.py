#!/usr/bin/env python2

import numpy as np
import time
from threading import Lock

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, PoseArray, Quaternion, PolygonStamped, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetMap
import tf
import tf.transformations

from sensor_model import SensorModel
from motion_model import MotionModel
import utils as Utils


class ParticleFilter:
    '''
    Monte Carlo Localization based on odometry and a laser scanner.
    '''
    def __init__(self):

        # Get parameters
        self.particle_filter_frame = rospy.get_param("~particle_filter_frame")
        self.MAX_PARTICLES         = int(rospy.get_param("~num_particles"))
        self.MAX_VIZ_PARTICLES     = int(rospy.get_param("~max_viz_particles"))
        self.PUBLISH_ODOM          = bool(rospy.get_param("~publish_odom", "1"))
        self.ANGLE_STEP            = int(rospy.get_param("~angle_step"))
        self.DO_VIZ                = bool(rospy.get_param("~do_viz"))

        # MCL algorithm
        self.iters = 0
        self.lidar_initialized = False
        self.odom_initialized = False 
        self.map_initialized = False
        self.last_odom_pose = None # last received odom pose
        self.last_stamp = None
        self.laser_angles = None
        self.downsampled_angles = None
        self.state_lock = Lock()

        # paritcles
        self.inferred_pose = None
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        self.particle_indices = np.arange(self.MAX_PARTICLES)
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()
        # initialize the state
        self.smoothing = Utils.CircularArray(10)
        self.timer = Utils.Timer(10)
        self.initialize_global() 
        # map
        self.permissible_region = None

        self.SHOW_FINE_TIMING = False

        # these topics are for visualization
        self.pose_pub      = rospy.Publisher("/pf/viz/inferred_pose", PoseStamped, queue_size = 1)
        self.particle_pub  = rospy.Publisher("/pf/viz/particles", PoseArray, queue_size = 1)
        self.pub_fake_scan = rospy.Publisher("/pf/viz/fake_scan", LaserScan, queue_size = 1)
        self.path_pub      = rospy.Publisher('/pf/viz/path', Path, queue_size = 1)
        self.path = Path()

        if self.PUBLISH_ODOM:
            self.odom_pub      = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)

        # these topics are for coordinate space things
        self.pub_tf = tf.TransformBroadcaster()

        # these topics are to receive data from the racecar
        self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.callback_lidar, queue_size=1)
        self.odom_sub  = rospy.Subscriber(rospy.get_param("~odom_topic", "/odom"), Odometry, self.callback_odom, queue_size=1)
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose, queue_size=1)
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=1)

        print "Finished initializing, waiting on messages..."

    def initialize_global(self):
        '''
        Spread the particle distribution over the permissible region of the state space.
        '''
        print("GLOBAL INITIALIZATION")
        # randomize over grid coordinate space
        self.state_lock.acquire()

        print('Waiting for map..')
        while not self.sensor_model.map_set:
            continue    

        self.map_initialized = True
        self.permissible_region = self.sensor_model.permissible_region 
        self.map = self.sensor_model.map
        self.map_info = self.sensor_model.map_info

        permissible_x, permissible_y = np.where(self.permissible_region == 1)
        indices = np.random.randint(0, len(permissible_x), size=self.MAX_PARTICLES)

        permissible_states = np.zeros((self.MAX_PARTICLES,3))
        permissible_states[:,0] = permissible_y[indices]
        permissible_states[:,1] = permissible_x[indices]
        permissible_states[:,2] = np.random.random(self.MAX_PARTICLES) * np.pi * 2.0

        Utils.map_to_world(permissible_states, self.map_info)
        self.particles = permissible_states
        self.weights[:] = 1.0 / self.MAX_PARTICLES
        self.state_lock.release()

    def initialize_particles_pose(self, pose):
        '''
        Initialize particles in the general region of the provided pose.
        '''
        print "SETTING POSE"
        print pose
        self.state_lock.acquire()
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
        self.particles[:,0] = pose.position.x + np.random.normal(loc=0.0,scale=0.5,size=self.MAX_PARTICLES)
        self.particles[:,1] = pose.position.y + np.random.normal(loc=0.0,scale=0.5,size=self.MAX_PARTICLES)
        self.particles[:,2] = Utils.quaternion_to_angle(pose.orientation) + np.random.normal(loc=0.0,scale=0.4,size=self.MAX_PARTICLES)
	
        self.state_lock.release()

    def clicked_pose(self, msg):
        '''
        Receive pose messages from RViz and initialize the particle distribution in response.
        '''
        print('clicked_pose')
        if isinstance(msg, PointStamped):
            self.initialize_particles_pose(msg.pose.pose)
            #self.initialize_global()
        elif isinstance(msg, PoseWithCovarianceStamped):
            self.initialize_particles_pose(msg.pose.pose)


    def publish_tf(self, pose, stamp=None):
        """ Publish a tf for the car. This tells ROS where the car is with respect to the map. """
        if stamp == None:
            stamp = rospy.Time.now()

        # this may cause issues with the TF tree. If so, see the below code.
        self.pub_tf.sendTransform((pose[0],pose[1],0),tf.transformations.quaternion_from_euler(0, 0, pose[2]), 
               stamp , "/laser", "/map")

        # also publish odometry to facilitate getting the localization pose
        if self.PUBLISH_ODOM:
            odom = Odometry()
            odom.header = Utils.make_header("/map", stamp)
            odom.pose.pose.position.x = pose[0]
            odom.pose.pose.position.y = pose[1]
            odom.pose.pose.orientation = Utils.angle_to_quaternion(pose[2])
            self.odom_pub.publish(odom)
        
        """
        Our particle filter provides estimates for the "laser" frame
        since that is where our laser range estimates are measure from. Thus,
        We want to publish a "map" -> "laser" transform.
        However, the car's position is measured with respect to the "base_link"
        frame (it is the root of the TF tree). Thus, we should actually define
        a "map" -> "base_link" transform as to not break the TF tree.
        """

        # Get map -> laser transform.
        map_laser_pos = np.array( (pose[0],pose[1],0) )
        map_laser_rotation = np.array( tf.transformations.quaternion_from_euler(0, 0, pose[2]) )
        # Apply laser -> base_link transform to map -> laser transform
        # This gives a map -> base_link transform
        laser_base_link_offset = (0.265, 0, 0)
        map_laser_pos -= np.dot(tf.transformations.quaternion_matrix(tf.transformations.unit_vector(map_laser_rotation))[:3,:3], laser_base_link_offset).T

        # Publish transform
        self.pub_tf.sendTransform(map_laser_pos, map_laser_rotation, stamp , self.particle_filter_frame, "/map")

    def publish_particles(self, particles):
        pa = PoseArray()
        pa.header = Utils.make_header("map")
        pa.poses = Utils.particles_to_poses(particles)
        self.particle_pub.publish(pa)

    def publish_scan(self, angles, ranges):
        ls = LaserScan()
        ls.header = Utils.make_header("laser", stamp=self.last_stamp)
        ls.angle_min = np.min(angles)
        ls.angle_max = np.max(angles)
        ls.angle_increment = np.abs(angles[0] - angles[1])
        ls.range_min = 0
        ls.range_max = np.max(ranges)
        ls.ranges = ranges
        self.pub_fake_scan.publish(ls)

    def visualize(self):
        '''
        Publish various visualization messages.
        '''
        if not self.DO_VIZ:
            return

        if self.pose_pub.get_num_connections() > 0 and isinstance(self.inferred_pose, np.ndarray):
            ps = PoseStamped()
            ps.header = Utils.make_header("map")
            ps.pose.position.x = self.inferred_pose[0]
            ps.pose.position.y = self.inferred_pose[1]
            ps.pose.orientation = Utils.angle_to_quaternion(self.inferred_pose[2])
            self.pose_pub.publish(ps)
            if self.iters % 10 == 0:
                self.path.header = ps.header
                self.path.poses.append(ps)
                self.path_pub.publish(self.path)

        if self.particle_pub.get_num_connections() > 0:
            if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                # randomly downsample particles
                proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES, p=self.weights)
                # proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES)
                self.publish_particles(self.particles[proposal_indices,:])
            else:
                self.publish_particles(self.particles)

        if self.pub_fake_scan.get_num_connections() > 0 and isinstance(self.sensor_model.particle_scans, np.ndarray):
            # generate the scan from the point of view of the inferred position for visualization
            inferred_scans = self.sensor_model.get_scans(self.inferred_pose[None, :])
            self.publish_scan(self.downsampled_angles, inferred_scans[0, :])

    def callback_lidar(self, msg):
        '''
        Initializes reused buffers, and stores the relevant laser scanner data for later use.
        '''
        if not isinstance(self.laser_angles, np.ndarray):
            print "...Received first LiDAR message"
            self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.downsampled_angles = np.copy(self.laser_angles[0::self.ANGLE_STEP]).astype(np.float32)
            #self.viz_queries = np.zeros((self.downsampled_angles.shape[0],3), dtype=np.float32)
            #self.viz_ranges = np.zeros(self.downsampled_angles.shape[0], dtype=np.float32)
            #print self.downsampled_angles.shape[0]
        self.downsampled_ranges = np.array(msg.ranges[::self.ANGLE_STEP])
        self.lidar_initialized = True

    def callback_odom(self, msg):
        '''
        Store deltas between consecutive odometry messages in the coordinate space of the car.
        Odometry data is accumulated via dead reckoning, so it is very inaccurate on its own.
        '''
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y])
        orientation = Utils.quaternion_to_angle(msg.pose.pose.orientation)
        pose = np.array([position[0], position[1], orientation])

        if isinstance(self.last_odom_pose, np.ndarray):
            rot = Utils.rotation_matrix(-self.last_odom_pose[2])
            delta = np.array([position - self.last_odom_pose[0:2]]).transpose()
            local_delta = (rot*delta).transpose()

            # changes in x,y,theta in local coordinate system of the car
            self.odometry_data = np.array([local_delta[0,0], local_delta[0,1], orientation - self.last_odom_pose[2]])
            self.last_odom_pose = pose
            self.last_stamp = msg.header.stamp
            self.odom_initialized = True
        else:
            print "...Received first Odometry message"
            self.last_odom_pose = pose

        # this topic is slower than lidar, so update every time we receive a message
        self.update()
    
    def expected_pose(self, particles):
        # returns the expected value of the pose given the particle distribution
        return np.dot(particles.transpose(), self.weights)

    def MCL(self, odom, scans):
        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        proposal_indices = np.random.choice(self.particle_indices, self.MAX_PARTICLES, p=self.weights)
        proposal_distribution = self.particles[proposal_indices,:]
        if self.SHOW_FINE_TIMING:
            t_propose = time.time()
        #print("resample",self.expected_pose(proposal_distribution))
        # compute the motion model to update the proposal distribution
        proposal_distribution = self.motion_model.evaluate(proposal_distribution, odom)
        if self.SHOW_FINE_TIMING:
            t_motion = time.time()
        #print("motion model:", self.expected_pose(proposal_distribution))

        # compute the sensor model
        self.weights = self.sensor_model.evaluate(proposal_distribution, scans)
        
        if self.SHOW_FINE_TIMING:
            t_sensor = time.time()

        # normalize importance weights
        self.weights /= np.sum(self.weights)
        #print("sensor model:", self.expected_pose(proposal_distribution))

        if self.SHOW_FINE_TIMING:
            t_norm = time.time()
            t_total = (t_norm - t)/100.0

        if self.SHOW_FINE_TIMING and self.iters % 10 == 0:
            print "MCL: propose: ", np.round((t_propose-t)/t_total, 2), "motion:", np.round((t_motion-t_propose)/t_total, 2), \
                  "sensor:", np.round((t_sensor-t_motion)/t_total, 2), "norm:", np.round((t_norm-t_sensor)/t_total, 2)

        # save the particles
        self.particles = proposal_distribution 

        #print(self.expected_pose(self.particles))

        # Publish a transformation frame between the map
        # and the particle_filter_frame.


    def update(self):
        '''
        Apply the MCL function to update particle filter state. 
        Ensures the state is correctly initialized, and acquires the state lock before proceeding.
        '''
        if self.lidar_initialized and self.odom_initialized and self.map_initialized:
            if self.state_lock.locked():
                print("Concurrency error avoided")
            else:
                self.state_lock.acquire()
                self.timer.tick()
                self.iters += 1

                t1 = time.time()
                scans = np.copy(self.downsampled_ranges).astype(np.float32)
                odom = np.copy(self.odometry_data)
                self.odometry_data = np.zeros(3)

                # run the MCL update algorithm
                self.MCL(odom, scans)

                # compute the expected value of the robot pose
                self.inferred_pose = self.expected_pose(self.particles)
                self.state_lock.release()
                t2 = time.time()

                # publish transformation frame based on inferred pose
                self.publish_tf(self.inferred_pose, self.last_stamp)

                # this is for tracking particle filter speed
                ips = 1.0 / (t2 - t1)
                self.smoothing.append(ips)
                if self.iters % 10 == 0:
                    print "iters per sec:", int(self.timer.fps()), " possible:", int(self.smoothing.mean())

                self.visualize()
if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
