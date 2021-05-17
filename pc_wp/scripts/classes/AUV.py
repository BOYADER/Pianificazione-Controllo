#!/usr/bin/env python

class AUV:
	def __init__(self, latitude, longitude, depth, roll, pitch, yaw, critical_pitch):
		self.latitude = latitude
		self.longitude = longitude
		self.depth = depth
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.critical_pitch = critical_pitch
		self.strategy = 0
		self.task_seq = []
		self.task_index = 0
		self.waypoints = []
		self.wp_index = 0

auv = None
