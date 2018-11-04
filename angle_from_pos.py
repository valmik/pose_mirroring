#!/usr/bin/env python

import numpy as np

def angle_from_pos(ps, pe, pw, th3_0):
    """
    Given the positions of the shoulder, elbow and wrist with respect the torso,
    calculate shoulder and elbow angle

    Inputs:
    ps: (3, ) ndarray: xyz position of the shoulder with respect to the torso, in the torso's reference frame
    pe: (3, ) ndarray: xyz position of the elbow with respect to the torso, in the torso's reference frame
    pw: (3, ) ndarray: xyz position of the wrist with respect to the torso, in the torso's reference frame
    th3_0: float: If theta_4 is zero (the elbow is straight), we can't tell what theta_3 will be, so we set it to this default

    Outputs:
    theta: (4, ) ndarray: joint angles. The angles we use are:
        theta_1: shoulder rotation about z
        theta_2: shoulder tilt
        theta_3: shoulder rotation about forearm
        theta_4: elbow
    """

    pass

def theta_4(ps, pe, pw):
    """
    Calculates theta_4.

    Theta_4 is the only thing that changes the distance between the wrist and the shoulder
    Therefore we use that
    """

    d1 = euclidean_distance(pe, ps)
    d2 = euclidean_distance(pw, pe)
    d3 = euclidean_distance(pw, ps)

    return np.arccos(((d1 ** 2) + (d2 ** 2) - (d3 ** 2))/(2*d1*d2))

def theta_1(ps, pe, pw):
    """
    Calculates theta_1
    """



def euclidean_distance(a, b):
    """
    Euclidean distance between two vectors
    """

    return np.linalg.norm(a-b)



