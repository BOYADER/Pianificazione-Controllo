#!/usr/bin/env python
import numpy as np
import math
import rospy
from classes.Waypoint import Waypoint

def wrap2pi(angle):
		return np.arctan2(np.sin(angle), np.cos(angle))

def isNone(vars_array):
	ans = False
	for var in vars_array:
		if var is None:
			ans = True
	return ans

def clear(vars_array):
	None_array = []
	for var in vars_array:
		None_array.append(None)
	return None_array

def get_waypoint(index):
	if index < len(rospy.get_param('waypoint_list')):
		lld_ned = rospy.get_param('ned_frame_origin')
		string_param = 'waypoint_list/wp' + str(index + 1)			
		latitude = rospy.get_param(string_param)['latitude']
		longitude = rospy.get_param(string_param)['longitude']
		depth = rospy.get_param(string_param)['depth']
		waypoint = Waypoint(latitude, longitude, depth, lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'])
		return waypoint
	elif index == len(rospy.get_param('waypoint_list')):
		lld_ned = rospy.get_param('ned_frame_origin')
		waypoint = Waypoint(lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'], lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'])
		return waypoint
	else:
		return None

def ned2body(pos_ned, eta_2):
	roll = eta_2[0]
	pitch = eta_2[1]
	yaw = eta_2[2]
	R_x = np.array([	[1, 0, 0],
		     		[0, math.cos(roll), math.sin(roll)],
	                 	[0, -math.sin(roll), math.cos(roll)]])
	R_y = np.array([	[math.cos(pitch), 0, -math.sin(pitch)],
				[0, 1, 0],
				[math.sin(pitch), 0, math.cos(pitch)]])
	R_z = np.array([	[math.cos(yaw), math.sin(yaw), 0],
                        	[-math.sin(yaw), math.cos(yaw), 0],
                        	[0, 0, 1]])
	R = np.dot(np.dot(R_x, R_y), R_z)
	pos_body = np.dot(R, pos_ned)
	return [pos_body[0], pos_body[1], pos_body[2]]

def projection(u, v):
	proj = (np.dot(u, v)/np.linalg.norm(v)**2)*v
	return proj

def print_adv(string):
	print(	"###############################################################################\n")	
	print(	"###############################################################################\n")
	print(	"                                   %s                                   \n" %  string)	
	print(	"###############################################################################\n")	
	print(	"###############################################################################\n")
	
