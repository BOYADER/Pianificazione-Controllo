#!/usr/bin/env python
import rospy
import numpy as np
import math
from pc_wp.msg import References, Odom

global position_error_ned, pose_error, velocity_error
position_error_ned = np.array([references.pose.x - odom.lld.x, references.pose.y - odom.pose.y, references.pose.z - odom.pose.z])
pose_error = np.array([references.rpy.x - odom.rpy.x, references.rpy.y - odom.rpy.y, references.rpy.z - odom.rpy.z])
velocity_error = np.array([rospy.get_param('/surge_velocity') - odom.lin_vel.x, 0, 0]) 

def error_body (position_error):
#definisco J1(eta2)
	roll = eta2[0]
	pitch = eta2[1]
	yaw = eta2 [2]
	R_x_roll = np.array([1,0,0]
						[0,math.cos(roll), -math.sin(roll)]
						[0,math.sin(roll), math.cos(roll)])
	R_y_pitch = np.array([mah.cos(pitch),0, math.sin(pitch)]
						 [0,1,0]
						 [-math.sin(pitch), 0, math.cos(pitch)])
	R_z_yaw = np.array([[math.cos(yaw), 0, math.sin(yaw)]
						[0, 1, 0]
						[-math.sin(yaw), 0, math.cos(yaw)])
	R_z_t = np.transpose(R_z_yaw)
   	R_y_t = np.transpose(R_y_pitch)
   	R_x_t = np.transpose(R_x_roll)
	J1 = np.dot(np.dot(R_z_t, R_y_t), R_x_t)
	
	global position_error_body = np.dot (np.transpose(J1), position_error)
	
	return(position_error_body)

def pid #scaricare libreria



























