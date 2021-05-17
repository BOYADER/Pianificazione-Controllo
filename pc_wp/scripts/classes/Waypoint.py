#!/usr/bin/env python
import pymap3d as pm

class Waypoint:
	def __init__(self, latitude, longitude, depth, latitude_ned, longitude_ned, depth_ned, tolerance):
		self.latitude = latitude
		self.longitude = longitude
		self.depth = depth
		self.latitude_ned = latitude_ned
		self.longitude_ned = longitude_ned
		self.depth_ned = depth_ned
		self.tolerance = tolerance
		self.x = pm.geodetic2ned(self.latitude, self.longitude, self.depth, self.latitude_ned, self.longitude_ned, self.depth_ned)[0]
		self.y = pm.geodetic2ned(self.latitude, self.longitude, self.depth, self.latitude_ned, self.longitude_ned, self.depth_ned)[1]
		self.z = pm.geodetic2ned(self.latitude, self.longitude, self.depth, self.latitude_ned, self.longitude_ned, self.depth_ned)[2]

