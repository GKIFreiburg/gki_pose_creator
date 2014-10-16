#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_pose_creator")
import rospy
import copy
import numpy as np

import wx
from tf import transformations as tr 
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
#from geometry_msgs.msg import PoseStamped, Pose
from gki_pose_creator.msg import NamedPoseStamped, NamedPoseStampedList

import rostopic
#from rospy.msg import serialize_message, args_kwds_to_message
import yaml
from genpy.message import fill_message_args

tf_listener = None
server = None
frame_id = 'odom'
menu_handler = MenuHandler()
marker_id = 0

def get_open_filename(wildcard):
	app = wx.App(None)
	style = wx.FD_OPEN | wx.FD_FILE_MUST_EXIST
	dialog = wx.FileDialog(None, 'Open', wildcard=wildcard, style=style)
	if dialog.ShowModal() == wx.ID_OK:
		path = dialog.GetPath()
	else:
		path = None
	dialog.Destroy()
	return path

def get_save_filename(wildcard):
	app = wx.App(None)
	style = wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT
	dialog = wx.FileDialog(None, 'Save', wildcard=wildcard, style=style)
	if dialog.ShowModal() == wx.ID_OK:
		path = dialog.GetPath()
	else:
		path = None
	dialog.Destroy()
	return path

def save_poses_cb(feedback):
	filename = get_save_filename('*.yaml')
	if filename:
		with open(filename, 'w') as file:
			npl = NamedPoseStampedList()
			for name in server.marker_contexts:
				marker = server.get(name)
				nps = NamedPoseStamped()
				nps.name = name
				nps.stamped.pose = marker.pose
				nps.stamped.header.frame_id = marker.header.frame_id
				npl.poses.append(nps)
			file.write(str(npl))

def load_poses_cb(feedback):
	filename = get_open_filename('*.yaml')
	if filename:
		with open(filename, 'r') as file:
			content = yaml.load(file.read())
			msg = NamedPoseStampedList()
			fill_message_args(msg, [content])
			for np in msg.poses:
				create_pose_marker(named_pose=np)
	server.applyChanges()

def clear_poses_cb(feedback):
	marker = server.get(feedback.marker_name)
	server.clear()
	server.insert(marker, feedback_cb)
	menu_handler.apply(server, marker.name)
	server.applyChanges()

def duplicate_pose_cb(feedback):
	marker = server.get(feedback.marker_name)
	new_marker = copy.deepcopy(marker)
	new_marker.name = generate_pose_name()
	new_marker.description = new_marker.name
	server.insert(new_marker, feedback_cb)
	menu_handler.apply(server, new_marker.name)
	server.applyChanges()

def delete_pose_cb(feedback):
	# do not delete the last marker
	if len(server.marker_contexts) > 1:
		server.erase(feedback.marker_name)
		server.applyChanges()

def feedback_cb(feedback):
	s = "Feedback from marker '" + feedback.marker_name
	s += "' / control '" + feedback.control_name + "'"

	mp = ""
	if feedback.mouse_point_valid:
		mp = " at " + str(feedback.mouse_point.x)
		mp += ", " + str(feedback.mouse_point.y)
		mp += ", " + str(feedback.mouse_point.z)
		mp += " in frame " + feedback.header.frame_id

	if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
		rospy.loginfo(s + ": button click" + mp + ".")
	elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
		rospy.loginfo(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
	elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
		rospy.loginfo(s + ": pose changed")
# TODO
#		  << "\nposition = "
#		  << feedback.pose.position.x
#		  << ", " << feedback.pose.position.y
#		  << ", " << feedback.pose.position.z
#		  << "\norientation = "
#		  << feedback.pose.orientation.w
#		  << ", " << feedback.pose.orientation.x
#		  << ", " << feedback.pose.orientation.y
#		  << ", " << feedback.pose.orientation.z
#		  << "\nframe: " << feedback.header.frame_id
#		  << " time: " << feedback.header.stamp.sec << "sec, "
#		  << feedback.header.stamp.nsec << " nsec" )
	elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
		rospy.loginfo(s + ": mouse down" + mp + ".")
	elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
		rospy.loginfo(s + ": mouse up" + mp + ".")
	
	server.applyChanges()

def generate_pose_name():
	global marker_id
	marker_id += 1
	return 'pose_{:02}'.format(marker_id)

def create_axes_markers(msg):
	x_marker = Marker()
	x_marker.type = Marker.CYLINDER
	x_marker.scale.x = msg.scale * 0.05
	x_marker.scale.y = msg.scale * 0.05
	x_marker.scale.z = msg.scale * 0.25
	x_marker.color.a = 1.0
	y_marker = copy.deepcopy(x_marker)
	z_marker = copy.deepcopy(x_marker)
	
	x_marker.pose.position.x += x_marker.scale.z * 0.5
	y_marker.pose.position.y += y_marker.scale.z * 0.5
	z_marker.pose.position.z += z_marker.scale.z * 0.5
	x_marker.color.r = 1.0
	y_marker.color.g = 1.0
	z_marker.color.b = 1.0
	
	x_quat = tr.quaternion_from_euler(0, np.deg2rad(90), 0)
	x_marker.pose.orientation.x = x_quat[0]
	x_marker.pose.orientation.y = x_quat[1]
	x_marker.pose.orientation.z = x_quat[2]
	x_marker.pose.orientation.w = x_quat[3]
	y_quat = tr.quaternion_from_euler(np.deg2rad(-90), 0, 0)
	y_marker.pose.orientation.x = y_quat[0]
	y_marker.pose.orientation.y = y_quat[1]
	y_marker.pose.orientation.z = y_quat[2]
	y_marker.pose.orientation.w = y_quat[3]
	
	y_marker.id = 1
	z_marker.id = 2
	
	return x_marker, y_marker, z_marker

def create_pose_marker(named_pose=None):
	global server, menu_handler, marker_id
	marker = InteractiveMarker()
	marker.header.frame_id = frame_id
	marker.scale = 1

	marker.name = generate_pose_name()
	
	if named_pose:
		marker.header.frame_id = named_pose.stamped.header.frame_id
		marker.name = named_pose.name
		marker.pose = named_pose.stamped.pose
	marker.description = marker.name

	control = InteractiveMarkerControl()
	control.orientation.w = 1
	control.orientation.x = 0
	control.orientation.y = 1
	control.orientation.z = 0
	
	# move xy + rotate yaw
	control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
	control.orientation_mode = InteractiveMarkerControl.FIXED
	marker.controls.append(copy.deepcopy(control))
	
	# move z 
	control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
	marker.controls.append(control)

	# rotate roll
	control = InteractiveMarkerControl()
	control.orientation.w = 1
	control.orientation.x = 0
	control.orientation.y = 0
	control.orientation.z = 1
	control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
	control.orientation_mode = InteractiveMarkerControl.INHERIT
	marker.controls.append(control)

	# rotate pitch
	control = InteractiveMarkerControl()
	control.orientation.w = 1
	control.orientation.x = 1
	control.orientation.y = 0
	control.orientation.z = 0
	control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
	control.orientation_mode = InteractiveMarkerControl.INHERIT
	marker.controls.append(control)

	# make one control using default visuals
	control = InteractiveMarkerControl()
	control.interaction_mode = InteractiveMarkerControl.MENU
	control.always_visible = True
	control.markers.extend(create_axes_markers(marker))
	marker.controls.append(control)

	server.insert(marker, feedback_cb)
	menu_handler.apply(server, marker.name)
		
if __name__ == "__main__":
	rospy.init_node("pose_creator")
	if len(sys.argv) > 1:
		frame_id = sys.argv[1]
	else:
		print 'no frame_id argument, defaulting to odom'

	server = InteractiveMarkerServer("pose_creator")

	menu_handler.insert("Duplicate", callback=duplicate_pose_cb)
	menu_handler.insert("Delete", callback=delete_pose_cb)
	menu_handler.insert("Save Poses", callback=save_poses_cb)
	menu_handler.insert("Load Poses", callback=load_poses_cb)
	menu_handler.insert("Clear Other Poses", callback=clear_poses_cb)

	create_pose_marker()

	server.applyChanges()

	rospy.spin()
