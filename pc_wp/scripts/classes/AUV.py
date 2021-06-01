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
		self.eta_2 = [roll, pitch, yaw]
		self.ni_1 = [vx, vy, vz]
		self.strategy = 0
		self.task_seq = []
		self.task_index = 0
		self.waypoint = None
		self.wp_index = 0
		self.tolerance = None
				
	def print_waypoint(self):
		print("Waypoint n. %d, coords [NED]: [%s, %s, %s]" % (	self.wp_index + 1,
									self.waypoint.eta_1[0],
									self.waypoint.eta_1[1],
									self.waypoint.eta_1[2]))
			
	def pitch_desired(self):			# compute pitch_des in order to decide the strategy
		pitch_des = -np.arctan2(	self.waypoint.eta_1[2] - self.eta_1[2],
						math.sqrt((self.waypoint.eta_1[0] - self.eta_1[0])**2 + (self.waypoint.eta_1[1] - self.eta_1[1])**2))
		print("pitch_des: %s" % math.degrees(pitch_des))
		return pitch_des
	
	def set_strategy(self, pitch_des):		# strategy and task_seq setting 
		critical_pitch = math.radians(rospy.get_param('critical_pitch'))
		critical_depth = rospy.get_param('critical_depth')		
		if (abs(pitch_des) < critical_pitch or abs(pitch_des) > (math.pi - critical_pitch)) and self.eta_1[2] > critical_depth:
			self.strategy = 1
		else:
			self.strategy = 2
		string_param = 'task_seq_list/ts' + str(self.strategy)
		self.task_seq = rospy.get_param(string_param)
		print("strategy: %d, task_seq: %s" % (self.strategy, self.task_seq))
	
	def set_tolerance(self):			# set task tolerance error
		string_param = 'task_tolerance_list/' + self.task_seq[self.task_index]
		if self.task_seq[self.task_index] == 'YAW' or self.task_seq[self.task_index] == 'PITCH':
			self.tolerance = math.radians(rospy.get_param(string_param))
		else:
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
			error = wrap2pi(references.rpy.z - self.eta_2[2])
		elif self.task_seq[self.task_index] == 'PITCH':
			error = wrap2pi(references.rpy.y - self.eta_2[1])
		elif self.task_seq[self.task_index] == 'HEAVE':
			error = references.pos.z - self.eta_1[2]
		elif self.task_seq[self.task_index] == 'SURGE' or self.task_seq[self.task_index] == 'APPROACH':
			error = math.sqrt(	(references.pos.x - self.eta_1[0])**2 + 	
						(references.pos.y - self.eta_1[1])**2 + 
						(references.pos.z - self.eta_1[2])**2)
			
		if self.task_seq[self.task_index] == 'YAW' or self.task_seq[self.task_index] == 'PITCH' :
			print("%s error: %s degrees" % (self.task_seq[self.task_index], int(round(math.degrees(error)))))
		else:
			print("%s error: %s meters" % (self.task_seq[self.task_index], round(error)))
		#print("references: %s, task_error: %s" % (references.pos, error))
		return error							


