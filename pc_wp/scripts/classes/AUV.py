#!/usr/bin/env python
import math
import numpy as np
import pymap3d as pm
import rospy
from Waypoint import Waypoint

class AUV:
	def __init__(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz, critical_pitch):
		self.latitude = latitude
		self.longitude = longitude
		self.depth = depth
		self.latitude_ned = latitude
		self.longitude_ned = longitude
		self.depth_ned = depth
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.pitch_des = None
		self.yaw_des = None
		self.critical_pitch = critical_pitch
		self.x = pm.geodetic2ned(self.latitude, self.longitude, self.depth, self.latitude_ned, self.longitude_ned, self.depth_ned)[0]
		self.y = pm.geodetic2ned(self.latitude, self.longitude, self.depth, self.latitude_ned, self.longitude_ned, self.depth_ned)[1]
		self.z = pm.geodetic2ned(self.latitude, self.longitude, self.depth, self.latitude_ned, self.longitude_ned, self.depth_ned)[2]
		self.vx = vx
		self.vy = vy
		self.vz = vz
		self.strategy = 0
		self.task_seq = []
		self.task_index = 0
		self.waypoints = []
		self.wp_index = 0
		
	def init_waypoints(self):
		tolerance = rospy.get_param('/tolerance_on_waypoint')
		index = 1
		while index <= len(rospy.get_param('/waypoint_list')):
			string_param = '/waypoint_list/wp' + str(index)
			latitude = rospy.get_param(string_param)['latitude']
			longitude = rospy.get_param(string_param)['longitude']
			depth = rospy.get_param(string_param)['depth']
			self.waypoints.append(Waypoint(latitude, longitude, depth, self.latitude_ned, self.longitude_ned, self.depth_ned, tolerance))
			print(self.waypoints[index-1].x,self.waypoints[index-1].y,self.waypoints[index-1].z)
			index = index + 1	
	
	def pitch_desired(self):
		self.pitch_des = - np.arctan2((self.waypoints[self.wp_index].z - self.z),(self.waypoints[self.wp_index].x - self.x))
	
	def yaw_desired(self):
		self.yaw_des = np.arctan2((self.waypoints[self.wp_index].y - self.y),(self.waypoints[self.wp_index].x - self.x))
	
	def update(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz):
		self.latitude = latitude
		self.longitude = longitude
		self.depth = depth
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.vx = vx
		self.vy = vy
		self.vz = vz
		self.x = pm.geodetic2ned(self.latitude, self.longitude, self.depth, self.latitude_ned, self.longitude_ned, self.depth_ned)[0]
		self.y = pm.geodetic2ned(self.latitude, self.longitude, self.depth, self.latitude_ned, self.longitude_ned, self.depth_ned)[1]
		self.z = pm.geodetic2ned(self.latitude, self.longitude, self.depth, self.latitude_ned, self.longitude_ned, self.depth_ned)[2]
	
#def init_auv():
#	global auv
#	latitude = rospy.get_param('/initial_pose/position/latitude')
#	longitude = rospy.get_param('/initial_pose/position/latitude')
#	depth = rospy.get_param('/initial_pose/position/depth')
#	roll = rospy.get_param('/initial_pose/orientation/roll')
#	pitch = rospy.get_param('/initial_pose/orientation/pitch')
#	yaw = rospy.get_param('/initial_pose/orientation/yaw')
#	critical_pitch = rospy.get_param('/critical_pitch')
#	auv = AUV(latitude, longitude, depth, roll, pitch, yaw, critical_pitch)	


