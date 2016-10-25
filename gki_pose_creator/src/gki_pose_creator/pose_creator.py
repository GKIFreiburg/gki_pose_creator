#!/usr/bin/env python

import copy
import numpy as np
import sys
import wx

import rospy

from tf import transformations as tr
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from interactive_markers.menu_handler import *
#from geometry_msgs.msg import PoseStamped, Pose
from gki_pose_creator.msg import NamedPoseStamped, NamedPoseStampedList

import rostopic
#from rospy.msg import serialize_message, args_kwds_to_message
import yaml
from genpy.message import fill_message_args

class PoseCreator:
	def __init__(self, frame_id='map'):
		self.tf_listener = None
		self.server = InteractiveMarkerServer("pose_creator")
		self.frame_id = frame_id
		self.menu_handler = MenuHandler()
		self.marker_id = 0
		self.menu_handler.insert("Duplicate", callback=self.duplicate_pose_cb)
		self.menu_handler.insert("Delete", callback=self.delete_pose_cb)
		self.menu_handler.insert("Save Poses", callback=self.save_poses_cb)
		self.menu_handler.insert("Load Poses", callback=self.load_poses_cb)
		self.menu_handler.insert("Clear Other Poses", callback=self.clear_poses_cb)
	
		self.create_control_marker()
		wp = NamedPoseStamped()
		wp.name = self.generate_pose_name()
		wp.stamped.header.frame_id = self.frame_id
		wp.stamped.pose.orientation.w = 1

		self.create_path_marker(wp)
		self.activate_marker(wp.name)
	
		self.waypoint_order = []
	
	def get_open_filename(self, wildcard):
		app = wx.App(None)
		style = wx.FD_OPEN | wx.FD_FILE_MUST_EXIST
		dialog = wx.FileDialog(None, 'Open', wildcard=wildcard, style=style)
		if dialog.ShowModal() == wx.ID_OK:
			path = dialog.GetPath()
		else:
			path = None
		dialog.Destroy()
		return path
	
	def get_save_filename(self, wildcard):
		app = wx.App(None)
		style = wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT
		dialog = wx.FileDialog(None, 'Save', wildcard=wildcard, style=style)
		if dialog.ShowModal() == wx.ID_OK:
			path = dialog.GetPath()
		else:
			path = None
		dialog.Destroy()
		return path

	def save_poses_cb(self, feedback):
		filename = self.get_save_filename('*.yaml')
		if filename:
			with open(filename, 'w') as file:
				npl = NamedPoseStampedList()
				for name in sorted(self.server.marker_contexts.keys()):
					if name == self.control_marker.name:
						continue
					marker = self.server.get(name)
					nps = NamedPoseStamped()
					nps.name = name
					nps.stamped.pose = marker.pose
					nps.stamped.pose.position.z = 0
					nps.stamped.header.frame_id = marker.header.frame_id
					npl.poses.append(nps)
				file.write(str(npl))
	
	def load_poses_cb(self, feedback):
		filename = self.get_open_filename('*.yaml')
		if filename:
			with open(filename, 'r') as file:
				content = yaml.load(file.read())
				msg = NamedPoseStampedList()
				fill_message_args(msg, [content])
				active_name = None
				for np in msg.poses:
					self.create_path_marker(np)
					active_name = np.name
				if active_name: 
					self.activate_marker(active_name)
		self.server.applyChanges()
	
	def clear_poses_cb(self, feedback):
		marker = self.server.get(feedback.marker_name)
		self.server.clear()
		self.server.insert(marker, self.feedback_cb)
		self.menu_handler.apply(self.server, marker.name)
		self.server.applyChanges()
	
	def duplicate_pose_cb(self, feedback):
		marker = self.server.get(feedback.marker_name)
		new_marker = copy.deepcopy(marker)
		new_marker.name = self.generate_pose_name()
		new_marker.description = new_marker.name
		self.server.insert(new_marker, self.feedback_cb)
		self.menu_handler.apply(self.server, new_marker.name)
		self.activate_marker(new_marker.name)
		self.server.applyChanges()
	
	def delete_pose_cb(self, feedback):
		# do not delete the last marker
		if len(self.server.marker_contexts) > 1:
			self.server.erase(feedback.marker_name)
			self.server.applyChanges()
	
	def feedback_cb(self, feedback):
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
			if feedback.marker_name == self.control_marker.name:
				self.server.setPose(self.active_marker.name, copy.deepcopy(self.control_marker.pose), self.control_marker.header)
		elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
			if feedback.marker_name != self.control_marker.name:
				self.activate_marker(feedback.marker_name)
			rospy.loginfo(s + ": mouse down" + mp + ".")
		elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
			rospy.loginfo(s + ": mouse up" + mp + ".")
		
		self.server.applyChanges()
	
	def generate_pose_name(self):
		self.marker_id += 1
		return 'p{:02}'.format(self.marker_id)
	
	def create_axes_markers(self, msg):
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

	def create_control_marker(self):
		marker = InteractiveMarker()
		marker.header.frame_id = self.frame_id
		marker.scale = 1
	
		marker.name = 'control'
		marker.description = ''
	
		self.server.insert(marker, self.feedback_cb)
		#self.menu_handler.apply(self.server, marker.name)
		
		control = InteractiveMarkerControl()
		control.orientation.w = 1
		control.orientation.x = 0
		control.orientation.y = 1
		control.orientation.z = 0
		
		# move xy + rotate yaw
		control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
		#control.orientation_mode = InteractiveMarkerControl.FIXED
		#marker.controls.append(copy.deepcopy(control))
		
		# move z 
		#control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		marker.controls.append(control)
	
		# rotate roll
# 		control = InteractiveMarkerControl()
# 		control.orientation.w = 1
# 		control.orientation.x = 0
# 		control.orientation.y = 0
# 		control.orientation.z = 1
# 		control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
# 		control.orientation_mode = InteractiveMarkerControl.INHERIT
# 		marker.controls.append(control)
# 	
# 		# rotate pitch
# 		control = InteractiveMarkerControl()
# 		control.orientation.w = 1
# 		control.orientation.x = 1
# 		control.orientation.y = 0
# 		control.orientation.z = 0
# 		control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
# 		control.orientation_mode = InteractiveMarkerControl.INHERIT
# 		marker.controls.append(control)
	
		self.control_marker = marker
		
	def create_path_marker(self, np):
		marker = InteractiveMarker()
		marker.header.frame_id = self.frame_id
		marker.scale = 1
	
		marker.name = np.name
		marker.description = np.name
		marker.pose =  np.stamped.pose
		marker.pose.position.z = 0.1
	
		self.server.insert(marker, self.feedback_cb)
		self.menu_handler.apply(self.server, marker.name)
		
		# make one control using default visuals
		control = InteractiveMarkerControl()
		control.interaction_mode = InteractiveMarkerControl.MENU
		control.always_visible = True
		control.markers.extend(self.create_axes_markers(marker))
		marker.controls.append(control)
		
	def create_path_markers(self, named_poses=NamedPoseStampedList()):
		for np in self.waypoints.values():
			self.create_path_marker(np)
			
	def activate_marker(self, pose_name):
		wp = self.server.get(pose_name)
		self.control_marker.pose = wp.pose
		self.server.setPose(self.control_marker.name, wp.pose, wp.header)
		self.active_marker = wp
		self.server.applyChanges()
		
	def update(self):
		self.server.applyChanges()