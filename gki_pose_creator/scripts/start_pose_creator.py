#!/usr/bin/env python

import sys
import rospy
from gki_pose_creator.pose_creator import PoseCreator

if __name__ == "__main__":
	rospy.init_node("pose_creator")

	pose_creator = PoseCreator()

	rospy.spin()
