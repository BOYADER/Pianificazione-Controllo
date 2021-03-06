#!/usr/bin/env python
import os
classes_path = os.getcwd()
scripts_path = os.path.abspath(os.path.join(classes_path, os.pardir))

import sys
sys.path.append(scripts_path)

import math
import numpy as np
import pymap3d as pm
import rospy
from utils import wrap2pi
from Waypoint import Waypoint

class AUV:
	def __init__(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz):
		self.lld = [latitude, longitude, depth]
		self.lld_ned = [latitude, longitude, depth]
		self.eta_1 = [	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[0], 
			      	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[1],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[2]]
		self.eta_2 = [	wrap2pi(roll), 
				wrap2pi(pitch),
				wrap2pi(yaw)]
		self.ni_1 = [vx, vy, vz]
		self.strategy = 0
		self.task_seq = []
		self.task_index = 0
		self.waypoint = None
		self.wp_index = 0
		self.tolerance = None
				
	def pitch_desired(self):										# compute pitch_des in order to decide the strategy
		pitch_des = -np.arctan2(	self.waypoint.eta_1[2] - self.eta_1[2],
						math.sqrt((self.waypoint.eta_1[0] - self.eta_1[0])**2 + (self.waypoint.eta_1[1] - self.eta_1[1])**2))
		return pitch_des
	
	def set_strategy(self, pitch_des):									# strategy and task_seq setting 
		critical_pitch = math.radians(rospy.get_param('critical_pitch'))
		critical_depth = rospy.get_param('critical_depth')		
		if (	(abs(pitch_des) < critical_pitch or abs(pitch_des) > (math.pi - critical_pitch)) 	# pitch desider under critical_pitch
			and self.eta_1[2] > critical_depth	):						# auv.eta_1.z over critical_depth
			self.strategy = 1
		else:
			self.strategy = 2
		string_param = 'task_seq_list/ts' + str(self.strategy)
		self.task_seq = rospy.get_param(string_param)
	
	def set_tolerance(self):										# set task tolerance error
		if self.wp_index == len(rospy.get_param('waypoint_list')) and self.task_seq[self.task_index] == 'APPROACH':
			string_param = 'task_tolerance_list/END_MISSION'
			self.tolerance = rospy.get_param(string_param)
		else: 
			string_param = 'task_tolerance_list/' + self.task_seq[self.task_index]
			if self.task_seq[self.task_index] == 'YAW' or self.task_seq[self.task_index] == 'PITCH':
				self.tolerance = math.radians(rospy.get_param(string_param))
			else:
				self.tolerance = rospy.get_param(string_param)


	def update(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz):				# update eta_1, eta_2, ni_1
		self.lld = [latitude, longitude, depth]
		self.eta_1 = [	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[0],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[1],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[2]]
		self.eta_2 = [	wrap2pi(roll), 
				wrap2pi(pitch),
				wrap2pi(yaw)]
		self.ni_1 = [vx, vy, vz]
		
	def task_error(self, references):									# compute current task error
		if self.task_seq[self.task_index] == 'YAW':
			error = wrap2pi(references.rpy.z - self.eta_2[2])
		elif self.task_seq[self.task_index] == 'PITCH':
			error = wrap2pi(references.rpy.y - self.eta_2[1])
		elif self.task_seq[self.task_index] == 'HEAVE':
			error = references.pos.z - self.eta_1[2]
		elif self.task_seq[self.task_index] == 'SURGE' or self.task_seq[self.task_index] == 'APPROACH':
			error = math.sqrt(	(references.pos.x - self.eta_1[0])**2 + 	
						(references.pos.y - self.eta_1[1])**2 + 
						(references.pos.z - self.eta_1[2])**2)
		return error							


