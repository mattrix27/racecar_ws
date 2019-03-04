#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy import stats
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion
from std_msgs.msg import Header, ColorRGBA


def laser_scan_to_points(msg):
    """
    Converts LIDAR data into a numpy matrix of points

    Format: Rows of [distance, angle]
    Output matrix shape: (Num of pts x 2)

    :param msg: LaserScan (rospy msg)
    :return: Numpy matrix of points (polar)
    """

    # Creates single dimension matrices
    distances = np.array(msg.ranges)
    angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

    return combine_points(distances, angles)  # Combines distances/angles and reshapes into (Num of pts, 2)


def closest_point(points):
    """
    Returns the point with the lowest magnitude

    :param points: Numpy matrix of points (polar)
    :return: Single point with the lowest magnitude (2, )
    """
    closet_index = np.argmin(points, axis=0)[0]
    return points[closet_index]

def points_in_circle(main_point, list_of_points, radius):
    vectors = list_of_points - main_point
    magnitudes = magnitude(vectors)
    mask = magnitudes < radius
    return list_of_points[mask]


def generate_path_from_wall(points, distance_from_wall, side):
    # TODO Optimize
    path = []
    for current_point in points:
        expand_rate = 0.05
        expansion = 0
        nearby_points = points_in_circle(current_point, points, distance_from_wall)

        while nearby_points.shape[0] < 2:
            nearby_points = points_in_circle(current_point, points, distance_from_wall + expansion)
            expansion += expand_rate
	
	# print(find_vector(nearby_points))
        norm_vec = np.flip(find_vector(nearby_points),0) * np.array([-1, 1])
        unit_vec = norm_vec / magnitude(norm_vec)

        temp_path_point_1 = current_point + unit_vec * distance_from_wall
        temp_path_point_2 = current_point + unit_vec * distance_from_wall * np.array([-1, -1])

        if magnitude(temp_path_point_1) < magnitude(temp_path_point_2):
            path_point = temp_path_point_1
        else:
            path_point = temp_path_point_2

        path.append(path_point)

    return np.array(path)


def filter_laser_scan(points, threshold = 0.5):
    last_distance = points[0][0]
    mask = []
    for point in points:
        mask.append(abs(point[0] - last_distance) < threshold)
        last_distance = point[0]

    print(mask)
    return points[mask]


def create_cloud_msg(points, frame_id='base_link', id=0, color=(1.0, 1.0, 1.0, 1), scale=(0.1,0.1,0.1)):
    return Marker(
        type=Marker.SPHERE_LIST,  # LINE_STRIP
        id=id,
        lifetime=rospy.Duration(0),
        points=[Point(point[0], point[1],0) for point in points],
        scale=Vector3(scale[0], scale[1], scale[2]),
        pose=Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)),
        ns="points",
        color=ColorRGBA(color[0], color[1], color[2], color[3]),
        header=Header(frame_id=frame_id, stamp=rospy.Time.now()))

def magnitude(vector):
    if vector.ndim < 2:
        return np.linalg.norm(vector)
    return np.linalg.norm(vector, axis=1)


def regression(points):
    slope, intercept, r_value, p_value, std_err = stats.linregress(split_points(points))
    return slope, intercept, r_value


def find_vector(points):
    return points[0] - points[-1]


def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

def polar_to_cartesian(points):
    """
    Converts points from polar coordinates to cartesian coordinates

    :param points: Numpy matrix of points (polar)
    :return points: Numpy matrix of points (cartesian)
    TODO Optimize and reshape less
    """

    # Split numpy array for easier conversion
    distances, angles = split_points(points)

    y = distances * np.sin(angles)
    x = distances * np.cos(angles)

    return combine_points(x, y)


def combine_points(arg1, arg2):
    """
    Combines two arguments into a single matrix

    :param arg1: Numpy matrix of argument one (Num of pts, )
    :param arg2: Numpy matrix of argument two (Num of pts, )
    :return points: Numpy matrix of points (Num of pts, 2)
    """
    return np.vstack((arg1, arg2)).T


def split_points(points):
    """
    Splits a matrix of points into two 1D matrices

    :param points: Numpy matrix of points (Num of pts, 2)
    :return: Tuple of two 1D matrices (Num of pts,)
    """

    return points[:, 0].flatten(), points[:, 1].flatten()

def pure_pursuit(points, lookahead):
    near_points = points_in_circle(np.array([0,0]), points, lookahead)
    within = False
    if near_points.size != 0:
        target_point = near_points[-1]
        within = True
    else:
        target_point = points[np.argmin(magnitude(points))]

    return np.array([target_point]), 2*target_point[1]/lookahead**2, within

