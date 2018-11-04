#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros

from sensor_msgs.msg import JointState

from angle_from_pose import angle_from_pose



class PoseMirror():
    """
    PoseMirror class
    Takes in tf skeleton tracking information from the kinect
    turns it into robot joints
    publishes joints to a topic
    """
    def __init__(self):
        super(ClassName, self).__init__()
        self._initialized = False

        # Tf
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Rate
        self._rate = rospy.Rate(20)

        rospy.on_shutdown(self.Shutdown)
        
    def Shutdown(self):
        """
        Run on shutdown (for safety)
        """

        pass


    def Initialize(self):
        """
        Initialize the PoseMirror object
        """
        self._name = rospy.get_name() + "/pose_mirror"

        # Load params
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters")
            return False

        if not self.LoadPublisher():
            rospy.logerr("%s: Error Initializing Publisher")

        self._initialized = True
        return True

    def LoadParameters(self):
        """
        Load the parameters from the parameter server
        """

        # Theta zeros
        if not rospy.has_param("~theta/default/1"):
            return False
        self._theta_default_1 = rospy.get_param("~theta/default/1")

        if not rospy.has_param("~theta/default/2"):
            return False
        self._theta_default_2 = rospy.get_param("~theta/default/2")

        if not rospy.has_param("~theta/default/3"):
            return False
        self._theta_default_3 = rospy.get_param("~theta/default/3")

        if not rospy.has_param("~theta/default/4"):
            return False
        self._theta_default_4 = rospy.get_param("~theta/default/4")

        if not rospy.has_param("~theta/default/5"):
            return False
        self._theta_default_5 = rospy.get_param("~theta/default/5")

        if not rospy.has_param("~theta/default/6"):
            return False
        self._theta_default_6 = rospy.get_param("~theta/default/6")

        if not rospy.has_param("~theta/default/7"):
            return False
        self._theta_default_7 = rospy.get_param("~theta/default/7")

        self._joint_names = ["s1", "s2", "e1", "e2", "w1", "w2", "w3"]
        self._default_joints = [self._theta_default_1,
                                self._theta_default_2, 
                                self._theta_default_3,
                                self._theta_default_4,
                                self._theta_default_5,
                                self._theta_default_6,
                                self._theta_default_7]

        # Frames
        if not rospy.has_param("~frames/body"):
            return False
        self._body_frame = rospy.get_param("~frames/body")

        if not rospy.has_param("~frames/shoulder"):
            return False
        self._shoulder_frame = rospy.get_param("~frames/shoulder")

        if not rospy.has_param("~frames/elbow"):
            return False
        self._elbow_frame = rospy.get_param("~frames/elbow")

        if not rospy.has_param("~frames/hand"):
            return False
        self._hand_frame = rospy.get_param("~frames/hand")

        if not rospy.has_param("~topics/joint"):
            return False
        self._joint_topic = rospy.get_param("~topics/joint")

    def LoadPublisher(self):
        """
        Start the joint publisher
        """

        self._joint_pub = rospy.Publisher(self._joint_topic, JointState, queue_size=10)

        return True

    def Run(self):
        """
        Runs the skeleton thing
        """
        if not self._initialized:
            rospy.logerr("Not Yet Initialized")
            return

        th30 = self._theta_default_3

        while not rospy.is_shutdown():
            try:
                shoulder_pose = self._tf_buffer.lookup_transform(
                    self._body_frame, self._shoulder_frame, rospy.Time())
                elbow_pose = self._tf_buffer.lookup_transform(
                    self._body_frame, self._elbow_frame, rospy.Time())
                hand_pose = self._tf_buffer.lookup_transform(
                    self._body_frame, self._hand_frame, rospy.Time())
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException):
                self._rate.sleep()
                continue
              
            shoulder_vec = vector3_to_arr(shoulder_pose.transform.translation)  
            elbow_vec = vector3_to_arr(shoulder_pose.transform.translation)  
            hand_vec = vector3_to_arr(shoulder_pose.transform.translation)  

            (th1, th2, th3, th4) = angle_from_pos(shoulder_vec, elbow_vec, hand_vec, th30)

            th30 = th3

            msg = JointState()
            msg.name = self._joint_names
            msg.position = self._default_joints
            msg.position[0] = msg.position[0] + th1
            msg.position[1] = msg.position[1] + th2
            msg.position[2] = msg.position[2] + th3
            msg.position[3] = msg.position[3] + th4

            self._joint_pub.publish(msg)

            self._rate.sleep()


    def vector3_to_arr(msg):
        """
        Turns a Vector3 message to a numpy array

        Input:
        msg: geometry_msgs/Vector3 message

        Output:
        vec: (3,) ndarray
        """

        return np.array([msg.x, msg.y, msg.z])

