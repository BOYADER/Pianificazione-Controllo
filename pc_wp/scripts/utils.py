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
		waypoint = Waypoint(lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'], lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'])
		return waypoint
	else:
		return None


