#!/usr/bin/env python
import math
import numpy as np
import pymap3d as pm
import rospy
from Waypoint import Waypoint

class AUV:
	def __init__(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz, critical_pitch, control_radius, tolerance_degrees, tolerance_meters):
		self.lld = [latitude, longitude, depth]
		self.lld_ned = [latitude, longitude, depth]
		self.eta_1 = [	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[0],
										pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[1],
										pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[2]]
		self.eta_2 = [roll, pitch, yaw]
		self.ni_1 = [vx, vy, vz]
		self.critical_pitch = critical_pitch
		self.strategy = 0
		self.task_seq = []
		self.task_index = 0
		self.waypoints = []
		self.wp_index = 0
		self.control_radius = control_radius
		self.tolerance_degrees = tolerance_degrees
		self.tolerance_meters = tolerance_meters
		
	def init_waypoints(self):
		tolerance = rospy.get_param('/tolerance_on_waypoint')
		index = 1
		while index <= len(rospy.get_param('/waypoint_list')):
			string_param = '/waypoint_list/wp' + str(index)
			latitude = rospy.get_param(string_param)['latitude']
			longitude = rospy.get_param(string_param)['longitude']
			depth = rospy.get_param(string_param)['depth']
			self.waypoints.append(Waypoint(latitude, longitude, depth, self.lld_ned[0], self.lld_ned[1], self.lld_ned[2], tolerance))
			print("Waypoint %d coords [NED]: [%s, %s, %s]" % (index, self.waypoints[index-1].eta_1[0],self.waypoints[index-1].eta_1[1],self.waypoints[index-1].eta_1[2]))
			index = index + 1	
	
	def pitch_des(self):
		pitch_des = math.degrees(-np.arctan2((self.waypoints[self.wp_index].eta_1[2] - self.eta_1[2]),(self.waypoints[self.wp_index].eta_1[0] - self.eta_1[0])))
		return pitch_des
	
	def update(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz):
		self.lld = [latitude, longitude, depth]
		self.eta_2 = [roll, pitch, yaw]
		self.ni_1 = [vx, vy, vz]
		self.eta_1 = [	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[0],
										pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[1],
										pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[2]]
	def task_error(self, references):
		if self.task_seq[self.task_index] == 'YAW':
			error = references.rpy.z - self.eta_2[2]
		elif self.task_seq[self.task_index] == 'PITCH':
			error = references.rpy.y - self.eta_2[1]
		elif self.task_seq[self.task_index] == 'SURGE':
			error = math.sqrt((self.eta_1[0]*self.eta_1[0] - self.waypoints[self.wp_index].eta_1[0]*self.waypoints[self.wp_index].eta_1[0]) + (self.eta_1[1]*self.eta_1[1] - self.waypoints[self.wp_index].eta_1[1]*self.waypoints[self.wp_index].eta_1[1]) + (self.eta_1[2]*self.eta_1[2] - self.waypoints[self.wp_index].eta_1[2]*self.waypoints[self.wp_index].eta_1[2]) - self.control_radius*self.control_radius) 											
		elif self.task_seq[self.task_index] == 'HEAVE':
			error = references.pos.z - self.eta_1[2]
		elif self.task_seq[self.task_index] == 'APPROACH':
			error = math.sqrt((self.eta_1[0]*self.eta_1[0] - self.waypoints[self.wp_index].eta_1[0]*self.waypoints[self.wp_index].eta_1[0]) + (self.eta_1[1]*self.eta_1[1] - self.waypoints[self.wp_index].eta_1[1]*self.waypoints[self.wp_index].eta_1[1]) + (self.eta_1[2]*self.eta_1[2] - self.waypoints[self.wp_index].eta_1[2]*self.waypoints[self.wp_index].eta_1[2]) - self.waypoints[self.wp_index].tolerance*self.waypoints[self.wp_index].tolerance) 
		return error


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


