import numpy as np
import rospy

class MotionModel:

    def __init__(self):
        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        ####################################
        self.x_sd = rospy.get_param("~x_sd")
        self.y_sd = rospy.get_param("~y_sd")
        self.t_sd = rospy.get_param("~t_sd")

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        thetas = particles[:, 2]
        n = thetas.shape[0]

        move_noise_x = np.random.normal(0, self.x_sd, (n, 1))  #0.02
        move_noise_y = np.random.normal(0, self.y_sd, (n, 1)) #0.005

	theta_noise = np.random.normal(0, self.t_sd, (n, 1))

        noise = np.hstack((move_noise_x, move_noise_y, theta_noise))
        noisy_odom = noise + odometry

        rot = np.array([[np.cos(thetas), np.sin(thetas), np.zeros(n)],
                        [-np.sin(thetas), np.cos(thetas), np.zeros(n)],
                        [np.zeros(n), np.zeros(n), np.ones(n)]])
        rotations = rot.T

        rot_odom = np.einsum('ij,ikj->ik', noisy_odom, rotations)
        result = rot_odom + particles

        return result
