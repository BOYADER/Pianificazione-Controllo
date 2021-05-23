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
		self.waypoint = None
		self.wp_index = 0
		self.tolerance = None
				
	def set_waypoint(self):				# set next waypoint, params from mission.yaml file
		if self.wp_index < len(rospy.get_param('/waypoint_list')):
			string_param = '/waypoint_list/wp' + str(self.wp_index+1)			
			latitude = rospy.get_param(string_param)['latitude']
			longitude = rospy.get_param(string_param)['longitude']
			depth = rospy.get_param(string_param)['depth']
			self.waypoint = Waypoint(latitude, longitude, depth, self.lld_ned[0], self.lld_ned[1], self.lld_ned[2])
			print("Waypoint n. %d, coords [NED]: [%s, %s, %s]" % (	self.wp_index+1,
										self.waypoint.eta_1[0],
										self.waypoint.eta_1[1],
										self.waypoint.eta_1[2]))	
	
	def pitch_desired(self):			# compute pitch_des in order to decide the strategy: pitch_des = - atan2(wp.z - auv.z, wp.x - auv.x)
		pitch_des = math.degrees(-np.arctan2(	self.waypoint.eta_1[2] - self.eta_1[2],
							self.waypoint.eta_1[0] - self.eta_1[0]))
		print("pitch_des: %s" % pitch_des)
		return pitch_des
	
	def set_strategy(self, pitch_des):		# strategy and task_seq setting 
		critical_pitch = rospy.get_param('/critical_pitch')		
		if abs(pitch_des) < critical_pitch or abs(pitch_des) > (180 - critical_pitch):
			self.strategy = 1
		else:
			self.strategy = 2
		string_param = '/task_seq_list/ts' + str(self.strategy)
		self.task_seq = rospy.get_param(string_param)
		print("strategy: %d, task_seq: %s" % (self.strategy, self.task_seq))
	
	def set_tolerance(self):			# set task tolerance error
		string_param = '/task_tolerance_list/' + self.task_seq[self.task_index]
		self.tolerance = rospy.get_param(string_param)

	def update(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz):
		self.lld = [latitude, longitude, depth]
		self.eta_1 = [	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[0],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[1],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[2]]
		self.eta_2 = [roll, pitch, yaw]
		self.ni_1 = [vx, vy, vz]
		
	def wrap2pi(self, angle):
		if angle % 360 < 180:
			return angle % 180
		else:
			return angle % (-180)
	
	def task_error(self, references):
		if self.task_seq[self.task_index] == 'YAW':
			error = self.wrap2pi(references.rpy.z - self.eta_2[2])
		elif self.task_seq[self.task_index] == 'PITCH':
			error = self.wrap2pi(references.rpy.y - self.eta_2[1])
		elif self.task_seq[self.task_index] == 'HEAVE':
			error = references.pos.z - self.eta_1[2]
		elif self.task_seq[self.task_index] == 'SURGE' or self.task_seq[self.task_index] == 'APPROACH':
			error = math.sqrt(	(references.pos.x - self.eta_1[0])**2 + 	
						(references.pos.y - self.eta_1[1])**2 + 
						(references.pos.z - self.eta_1[2])**2) 
		print("%s error: %s" % (self.task_seq[self.task_index], abs(error)))
		return error							


