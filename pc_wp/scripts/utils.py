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

#funzione che imposta tutte le variabili dell'array a None
def clear_vars(array):
	ret = []
	for var in array:
		ret.append(None)
	return ret


def get_waypoint(index):
	if index < len(rospy.get_param('/waypoint_list')):   #se indice corrisponde a un wp esistente
		lld_ned = rospy.get_param('ned_origin')
		string_param = '/waypoint_list/wp' + str(index + 1)			
		latitude = rospy.get_param(string_param)['latitude']
		longitude = rospy.get_param(string_param)['longitude']
		depth = rospy.get_param(string_param)['depth']
		waypoint = Waypoint(latitude, longitude, depth, lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'])  #restituisce le coordinate in ned del wp
		return waypoint
	elif index == len(rospy.get_param('/waypoint_list')):  #se sono finiti i wp
		waypoint = Waypoint(lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'], lld_ned['latitude'], lld_ned['longitude'], lld_ned['depth'])
		return waypoint #restituisce le coordinate dell'origine della ned in ned, quindi [0 0 0]
	else: #altrimenti nessun wp
		return None
	

def min_distance_angle	(final_angle, current_angle):
	if (final_angle - current_angle)%(2*np.pi) < np.pi:
		return (final_angle - current_angle)%np.pi
	else:
		return (final_angle - current_angle)%(-np.pi) 



