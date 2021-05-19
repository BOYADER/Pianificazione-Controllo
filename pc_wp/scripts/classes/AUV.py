#!/usr/bin/env python
import math
import numpy as np
import pymap3d as pm
import rospy
from Waypoint import Waypoint

class AUV:
	def __init__(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz):
		self.lld = [latitude, longitude, depth]
		self.lld_ned = [latitude, longitude, depth]
		self.eta_1 = [	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[0], 
			      	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[1],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[2]]
		self.eta_2 = [roll, pitch, yaw]
		self.ni_1 = [vx, vy, vz]
		self.strategy = 0
		self.task_seq = []
		self.task_index = 0
		self.waypoints = []
		self.wp_index = 0
		self.critical_pitch = rospy.get_param('/critical_pitch')
		self.tolerance = None
				
	def init_waypoints(self):			# init waypoint array, params from mission.yaml file
		index = 1
		while index <= len(rospy.get_param('/waypoint_list')):
			string_param = '/waypoint_list/wp' + str(index)
			latitude = rospy.get_param(string_param)['latitude']
			longitude = rospy.get_param(string_param)['longitude']
			depth = rospy.get_param(string_param)['depth']
			self.waypoints.append(Waypoint(latitude, longitude, depth, self.lld_ned[0], self.lld_ned[1], self.lld_ned[2]))
			print("Waypoint %d coords [NED]: [%s, %s, %s]" % (	index,
										self.waypoints[index-1].eta_1[0],
										self.waypoints[index-1].eta_1[1],
										self.waypoints[index-1].eta_1[2]))
			index = index + 1	
	
	def pitch_desired(self):			# compute pitch_des in order to decide the strategy: pitch_des = - atan2(wp.z - auv.z, wp.x - auv.x)
		pitch_des = math.degrees(-np.arctan2(	self.waypoints[self.wp_index].eta_1[2] - self.eta_1[2],
							self.waypoints[self.wp_index].eta_1[0] - self.eta_1[0]))
		return pitch_des
	
	def set_strategy(self, pitch_des):		# strategy and task_seq setting 
		if abs(pitch_des) < self.critical_pitch or abs(pitch_des) > (180 - self.critical_pitch):
			self.strategy = 1
		else:
			self.strategy = 2
		string_param = '/task_seq_list/ts' + str(self.strategy)
		self.task_seq = rospy.get_param(string_param)
		print("strategy: %d, task_seq: %s" % (self.strategy, self.task_seq))
	
	def set_tolerance(self):			# set task tolerance error
		string_param = '/error_tolerances_list/' + self.task_seq[self.task_index]
		self.tolerance = rospy.get_param(string_param)

	def update(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz):
		self.lld = [latitude, longitude, depth]
		self.eta_1 = [	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[0],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[1],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[2]]
		self.eta_2 = [roll, pitch, yaw]
		self.ni_1 = [vx, vy, vz]
		
	def task_error(self, references):
		if self.task_seq[self.task_index] == 'YAW':
			error = references.rpy.z - self.eta_2[2]
		elif self.task_seq[self.task_index] == 'PITCH':
			error = references.rpy.y - self.eta_2[1]
		elif self.task_seq[self.task_index] == 'HEAVE':
			error = references.pos.z - self.eta_1[2]
		elif self.task_seq[self.task_index] == 'SURGE' or self.task_seq[self.task_index] == 'APPROACH':
			error = math.sqrt(	(self.eta_1[0] - self.waypoints[self.wp_index].eta_1[0])**2 + 
						(self.eta_1[1] - self.waypoints[self.wp_index].eta_1[1])**2 + 
						(self.eta_1[2] - self.waypoints[self.wp_index].eta_1[2])**2) 
		return error			#sqrt[(auv.x - wp.x)^2 + (auv.y - wp.y)^2 + (auv.z -wp.z)^2]					


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


