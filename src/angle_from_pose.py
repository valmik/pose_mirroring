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
        theta_1: shoulder rotation about vertical
        theta_2: shoulder tilt
        theta_3: shoulder rotation about forearm
        theta_4: elbow
    """

    theta_1 = theta_1(ps, pe, pw)
    theta_2 = theta_2(ps, pe, pw)
    theta_3 = theta_3(ps, pe, pw, th3_0)
    theta_4 = theta_4(ps, pe, pw)

    return (theta_1, theta_2, theta_3, theta_4)
    

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
    Calculates theta_1.

    Theta_1 is the projection of the elbow-shoulder on the z axis / the x axis
    """

    se = (pe - ps)
    return np.arctan(se[2] / se[0])

def theta_2(ps, pe, pw):
    """
    Calculates theta_2

    Theta_2 is the projection of the elbow-shoulder on the y axis / the x axis
    """

    se = (pe - ps)
    return np.arctan(se[1] / se[0])

def theta_3(ps, pe, pw, th3_0):
    """
    Calculates theta_3

    Theta_3 depends on the ratio of the yhat and zhat portions of pw
    in a basis defined by pe-ps and the vertical axis
    """

    try:
        (xhat, yhat, zhat) = definebasis(pe - ps, np.array([0, 1, 0]))
    except ValueError:
        # The arm is straight so theta_3 could be anything
        return th3_0

    arm_to_world = np.hstack([xhat.reshape(3,1), yhat.reshape(3,1), zhat.reshape(3,1)])
    world_to_arm = np.linalg.inv(axlebasis)

    arm_w = world_to_arm.dot(pw - ps)

    return np.arctan(arm_w[2]/armw[1])

def unitify(vec):
    """
    Creates a unit vector from a vector

    vec3 - (n,) ndarray
    """
    return vec / np.linalg.norm(vec)


def definebasis(vec1, vec2):
    """
    Defines a basis where vec1 is the x axis of the new basis (xhat)
    vec2 is a linear combination of xhat and yhat

    vec1 - (3,) ndarray
    vec2 - (3,) ndarray
    """

    xhat = unitify(vec1)
    zhat = unitify(np.cross(vec1, vec2))
    yhat = unitify(np.cross(zhat, xhat))

    if np.linalg.norm(xhat) > 0.01:
        if np.linalg.norm(yhat) > 0.01:
            if np.linalg.norm(zhat) > 0.01:
                return xhat, yhat, zhat

    raise ValueError('vectors are parallel')



def euclidean_distance(a, b):
    """
    Euclidean distance between two vectors
    """

    return np.linalg.norm(a-b)



