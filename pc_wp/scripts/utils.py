#!/usr/bin/env python
import numpy as np
import math
import rospy
from classes.Waypoint import Waypoint

def wrap2pi(angle):
		res = np.arctan2(np.sin(angle), np.cos(angle))
		return [res, int(round(math.degrees(res)))]
		# if angle % (2*math.pi) < math.pi:
		#	return angle % math.pi
		# else:
		#	return angle % (-math.pi)

def clear_vars(array):
	ret = []
	for var in array:
		ret.append(None)
	return ret
	
def get_waypoint(index):
	lld_ned = rospy.get_param('ned_origin')
	string_param = '/waypoint_list/wp' + str(index)			
	latitude = rospy.get_param(string_param)['latitude']
	longitude = rospy.get_param(string_param)['longitude']
	depth = rospy.get_param(string_param)['depth']
	waypoint = Waypoint(latitude, longitude, depth, lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'])
	return waypoint