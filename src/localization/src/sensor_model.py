from __future__ import division

import numpy as np
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid


class SensorModel:

    def __init__(self):
        
        self.on_robot = rospy.get_param("~on_robot")
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        #self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        
        if self.on_robot == 1:
            self.num_beams_per_particle = 109#1081
        else:
            self.num_beams_per_particle = 100

	self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")

        
        ####################################
        def p_hit(zt, z_star, z_max, sigma):
            output = 1 / np.sqrt(2 * np.pi * sigma ** 2) * np.e ** (-(zt - z_star) ** 2 / (2 * sigma ** 2))

            mask = ((zt >= 0) & (zt <= z_max))
            output[~mask] = 0
            return output

        def p_short(zt, z_star):
            with np.errstate(divide='ignore', invalid='ignore'):
                output = 2 / z_star * (1 - zt / z_star)
                output[output == np.inf] = 0
                output = np.nan_to_num(output)

            mask = ((zt >= 0) & (zt <= z_star))
            output[~mask] = 0
            return output

        def p_max(zt, z_max):
            return (zt == z_max) * 1

        def p_rand(zt, z_max):
            output = np.ones((zt.shape[0], 1))
            output = output / z_max

            mask = ((zt >= 0) & (zt < z_max))
            output[~mask] = 0
            return output

        def p(zt, z_star, z_max, sigma, a_hit, a_short, a_max, a_rand):
            answer = a_hit * p_hit(zt, z_star, z_max, sigma) \
                     + a_short * p_short(zt, z_star) \
                     + a_max * p_max(zt, z_max) \
                     + a_rand * p_rand(zt, z_max)

            # print("Hit: ", p_hit(zt, z_star, z_max, sigma))
            # print("Short: ", p_short(zt, z_star))
            # print("Max: ", p_max(zt, z_max))
            # print("Rand: ", p_rand(zt, z_max))

            return answer

        self.z_max = 10
        sigma = 0.5

        a_hit = rospy.get_param("~a_hit")
        a_short = rospy.get_param("~a_short")
        a_max = rospy.get_param("~a_max")
        a_rand = rospy.get_param("~a_rand")

        self.increment = 0.01

        elements = np.arange(0, self.z_max + self.increment, self.increment)

        pairs = np.array(np.meshgrid(elements, elements)).T.reshape(-1, 2)
        zt = pairs[:, 0:1]  # Takes the first col of pairs (Ex. like the x value of a coordinate pair)
        z_star = pairs[:, 1:2]  # Takes the second col of pairs (Ex. like the y value of a coordinate pair)

        grid = p(zt, z_star, self.z_max, sigma, a_hit, a_short, a_max, a_rand)
        grid = self.normalize(grid)
        self.grid = grid.reshape((elements.shape[0], elements.shape[0])).T

        ####################################

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
            self.num_beams_per_particle,
            self.scan_field_of_view,
            0,  # This is not the simulator, don't add noise
            0.01,  # This is used as an epsilon
            self.scan_theta_discretization)

        # Subscribe to the map
        self.map_set = False
        rospy.Subscriber(
            self.map_topic,
            OccupancyGrid,
            self.map_callback,
            queue_size=1)

    def evaluate(self, particles, observation, downsample=100):
        """
        Evaluate how likely each particle is given
        the observed scan.
        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]
            observation: A vector of lidar data measured
                from the actual lidar.
        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle

        scans = self.scan_sim.scan(particles)
        #print(scans.shape)
        #print(observation.shape)
	
        #if scans.shape[1] > downsample:
        #    stride = scans.shape[1]//downsample
        #    scans = scans[:, ::stride]
        #    observation = observation[:, ::stride]

        if self.on_robot == 1:
	    observation = observation[:, ::10]
	
	#print(scans.shape, observation.shape)
        def limit(values, limit):
            temp_values = np.array(values)
            temp_indicies = temp_values > limit
            temp_values[temp_indicies] = limit
            return temp_values


        rays = scans
        lidar = observation

        rays = limit(rays, self.z_max)
        lidar = limit(lidar, self.z_max)

        testing_z = (lidar / self.increment).astype(int)
        testing_z_star = (rays / self.increment).astype(int)
        
        p_values = self.grid[testing_z_star, testing_z]
        p_values = p_values/np.max(p_values)
        p_values = np.product(p_values, axis = 1)

        #p_values = np.mean(self.grid[testing_z_star, testing_z], axis=1)
        #print(self.grid[testing_z_star, testing_z])
        #print(p_values)
        

        probabilities = self.normalize(p_values)
        return probabilities

    def normalize(self, p):
        total = np.sum(p)
        return p / total

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        map_ = np.array(map_msg.data, np.double) / 100.
        map_ = np.clip(map_, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
            origin_o.x,
            origin_o.y,
            origin_o.z,
            origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            map_,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            origin,
            0.5)  # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
