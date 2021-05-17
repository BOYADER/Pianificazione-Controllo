#!/usr/bin/env python
import math
import numpy as np
import rospy

class AUV:
	def __init__(self, latitude, longitude, depth, roll, pitch, yaw, critical_pitch):
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
		self.vx = None
		self.vy = None
		self.vz = None
		self.latitude_ned = None
		self.longitude_ned = None
		self.depth_ned = None
		self.strategy = 0
		self.task_seq = []
		self.task_index = 0
		self.waypoints = []
		self.wp_index = 0
		
	def ned(self, latitude_ned, longitude_ned, depth_ned):
		self.latitude_ned = latitude_ned
		self.longitude_ned = longitude_ned
		self.depth_ned = depth_ned

	def geo2ned(self):
		R_e = rospy.get_param('/R_e') 
		R_p = rospy.get_param('/R_p')
		e = math.sqrt(1 - (R_p*R_p)/(R_e*R_e))
		N_e_auv = R_e/(math.sqrt(1 - e*e*math.sin(self.latitude)*math.sin(self.latitude)))
		x_ecef_auv = (N_e_auv - self.depth) * math.cos(self.latitude) * math.cos(self.longitude) # h = - depth
		y_ecef_auv = (N_e_auv - self.depth) * math.cos(self.latitude) * math.sin(self.longitude)
		z_ecef_auv = (N_e_auv * (1 - e*e) - self.depth) * math.sin(self.latitude)
		coords_ecef_auv = np.array([[x_ecef_auv, y_ecef_auv, z_ecef_auv]]).T
	
		N_e_ned_frame = R_e/(math.sqrt(1-e*e*math.sin(self.latitude_ned)*math.sin(self.latitude_ned)))
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

		coords_ned_auv = R_ned_ecef*(coords_ecef_auv - coords_ecef_ned_frame)		
		self.x = coords_ned_auv.item(0)
		self.y = coords_ned_auv.item(1)
		self.z = coords_ned_auv.item(2)

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
		self.geo2ned()
	


