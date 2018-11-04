#!/usr/bin/env python

import rospy
import sys

from pose_mirror import PoseMirror

if __name__ == '__main__':
	rospy.init_node("pose_mirroring_node")

	pm = PoseMirror()
	if not pm.Initialize():
		rospy.logerr("Failed to initialize pose mirror node")
		sys.exit(1)

	pm.Run()

	rospy.spin()