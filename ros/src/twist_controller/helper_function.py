from math import sqrt, cos, sin
import numpy as np
import tf

POINTS_TO_FIT = 10


def fit_polynomial(waypoints, degree):
    # TODO Add fit polynomial here
    """Add fit a polynomial for given waypoints"""
    return 0

def find_euler(pose):
    # TODO Returns the roll, pitch yaw angles from a Quaternion """
    return 0


def eucleidian_distance(x0, y0, x1, y1):
    """The Eucleidian distance between points (x0,y0) and (x1,y1)"""
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2))


def dist_parabolic(coefficients, x, y):

    #TODO
    """
    Calculates the distance of a point from a parabola defined
    by its polynomial coefficients.
    Args:
         coefficients (list) : Polynomial coefficients from higher to lower order
         x (float) : X-Coordinate of the point we want distance for
         y (float) : Y-Coordinate of the point we want distance for
         plot (bool) : If True, create a plot of the parabola, the point and the distance
    Returns:
        distance (float) : The distance of the point to the parabola
        left (int) : 1 if the point is on the left of the curve as we "walk" the curve
                     from negative to positive x's. -1 if on the right
    """
    return 0


def shift_rotate_wp(pose, waypoints, points_to_use=None):

    #TODO
    """
    From a pose object transfrom a series of waypoints so that
    the origin is at the pose position and the orientation matches
    the yaw of the pose
    Args:
        pose (object) : A pose object
        waypoints (list) : A list of waypoint objects
        points_to_use (int) : How many points to use (None => all)
    Returns:
        x_coords (list): The transformed x-coordinates of waypoints
        y_coords (list): The transformed y-coordinates of waypoints
    """

    return 0


def is_waypoint_behind(pose, waypoint):

    #TODO
    """Take a waypoint and a pose , do a coordinate system transformation
    setting the origin at the position of the pose object and as x-axis
    the orientation of the z-axis of the pose
    Args:
        pose (object) : A pose object
        waypoints (object) : A waypoint object
    Returns:
        bool : True if the waypoint is behind the car False if in front
    """
    return 0


def cte(pose, waypoints):
    
    #TODO
    """
    From a pose object and a series of waypoints, calculate the distance
    (cross track error) of the point, defined by the pose, to the path,
    defined by the series of waypoints.
    Args:
        pose (object) : A pose object
        waypoints (list) : A list of waypoint objects
    Returns:
        cte (float) : the cross track error (signed)
    """
    return 0
