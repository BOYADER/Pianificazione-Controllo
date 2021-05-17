#!/usr/bin/env python
import math
import numpy as np
import rospy
import Waypoint

class AUV:
	def __init__(self, latitude, longitude, depth, roll, pitch, yaw, vx, vy, vz, critical_pitch):
		self.latitude = latitude
		self.longitude = longitude
		self.depth = depth
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.critical_pitch = critical_pitch
		self.x = None
		self.y = None
		self.z = None
		self.vx = vx
		self.vy = vy
		self.vz = vz
		self.latitude_ned = latitude
		self.longitude_ned = longitude
		self.depth_ned = depth
		self.strategy = 0
		self.task_seq = []
		self.task_index = 0
		self.waypoints = []
		self.wp_index = 0
		self.geo2ned(self.latitude, self.longitude, self.depth, -1)	
		
	def geo2ned(self, latitude, longitude, depth, index):
		R_e = rospy.get_param('/R_e') 
		R_p = rospy.get_param('/R_p')
		e = math.sqrt(1 - (R_p*R_p)/(R_e*R_e))
		N_e = R_e/(math.sqrt(1 - e*e*math.sin(latitude)*math.sin(latitude)))
		x_ecef = (N_e - depth) * math.cos(latitude) * math.cos(longitude) 							# h = - depth
		y_ecef = (N_e - depth) * math.cos(latitude) * math.sin(longitude)
		z_ecef = (N_e * (1 - e*e) - depth) * math.sin(latitude)
		coords_ecef = np.array([[x_ecef, y_ecef, z_ecef]]).T
	
		N_e_ned_frame = R_e/(math.sqrt(1 - e*e*math.sin(self.latitude_ned)*math.sin(self.latitude_ned)))
		x_ecef_ned_frame = (N_e_ned_frame - self.depth_ned) * math.cos(self.latitude_ned) * math.cos(self.longitude_ned)
		y_ecef_ned_frame = (N_e_ned_frame - self.depth_ned) * math.cos(self.latitude_ned) * math.sin(self.longitude_ned)
		z_ecef_ned_frame = (N_e_ned_frame * (1 - e*e) - self.depth_ned) * math.sin(self.latitude_ned)
		coords_ecef_ned_frame = np.array([[x_ecef_ned_frame,y_ecef_ned_frame, z_ecef_ned_frame]]).T

		r11 = -(math.sin(self.latitude_ned) * math.cos(self.longitude_ned))
		r12 = -(math.sin(self.latitude_ned) * math.sin(self.longitude_ned))
		r13 = math.cos(self.latitude_ned)
		r21 = -math.sin(self.longitude_ned)
		r22 = math.cos(self.longitude_ned)
		r23 = 0
		r31 = -(math.cos(self.latitude_ned) * math.cos(self.longitude_ned))
		r32 = -(math.cos(self.latitude_ned) * math.sin(self.longitude_ned))
		r33 = -math.sin(self.latitude_ned)
		R_ned_ecef = np.matrix([[r11, r12, r13],[r21, r22, r23],[r31, r32, r33]])

		coords_ned = R_ned_ecef*(coords_ecef - coords_ecef_ned_frame)
		
		if index == -1:
			self.x = coords_ned.item(0)
			self.y = coords_ned.item(1)
			self.z = coords_ned.item(2)
		else:
			self.waypoints[index].x = coords_ned.item(0)
			self.waypoints[index].y = coords_ned.item(1)
			self.waypoints[index].z = coords_ned.item(2)

	def init_waypoints(self):
		tolerance = rospy.get_param('/tolerance_on_waypoint')
		index = 1
		while index <= len(rospy.get_param('/waypoint_list')):
			string_param = '/waypoint_list/wp' + str(index)
			latitude = rospy.get_param(string_param)['latitude']
			longitude = rospy.get_param(string_param)['longitude']
			depth = rospy.get_param(string_param)['depth']
			self.waypoints.append(Waypoint(latitude, longitude, depth, tolerance))
			#print(auv.waypoints[index-1].latitude, auv.waypoints[index-1].longitude, auv.waypoints[index-1].depth)
			self.geo2ned(latitude, longitude, depth, index-1)
			#print(auv.waypoints[index-1].x, auv.waypoints[index-1].y, auv.waypoints[index-1].z)
			index = index + 1	
	
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
		self.geo2ned(self.latitude, self.longitude, self.depth, -1)
	
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


