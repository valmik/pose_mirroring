# pose_mirroring
Use a Microsoft Kinect and the Openni skeleton tracking libraries, measure the pose of a human. Translate the pose to a set of joint positions, then send them to a robot for execution. Here I used a robot from Berkeley's Robot Learning Lab, but any 7-Dof (or even as low as 4-dof) arm would suffice if the kinematics are similar. For example, this code was originally designed to work with a RethinkRobotics Baxter.

Technologies used:

ROS

Python / Numpy

Kinect / Openni


Approximate Time Spent:

14h: installing and troubleshooting Kinect drivers (see my ros_setup repo for some scripts I wrote to help with this)

3h: architecting code and doing math

2h: filling in functions

1h: on-robot testing


Won Second Place Overall at CalHacks 2018
