#!/usr/bin/env python
import pymap3d as pm
import rospy

class Waypoint:
	def __init__(self, latitude, longitude, depth, latitude_ned, longitude_ned, depth_ned):
		self.lld = [latitude, longitude, depth]
		lld_ned = rospy.get_param('initial_pose')['position'] ####togli
		self.lld_ned = [latitude_ned, longitude_ned, depth_ned]
		self.lld_ned = [lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth']] ####togli
		self.eta_1 = [	pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[0],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[1],
				pm.geodetic2ned(self.lld[0], self.lld[1], -self.lld[2], self.lld_ned[0], self.lld_ned[1], -self.lld_ned[2])[2]]
