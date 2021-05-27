#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
import time
from pc_wp.msg import References, State, Odom
from utils import wrap2pi

strategy = None
current_task = None
wp_index = None

eta_1 = None
eta_2 = None

task_changed = False

eta_1_init = None
eta_2_init = None

time_start = None

#errori
global position_error_ned, pose_error, velocity_error
position_error_ned = np.array([references.pose.x - odom.lld.x, references.pose.y - odom.pose.y, references.pose.z - odom.pose.z])
pose_error = np.array([references.rpy.x - odom.rpy.x, references.rpy.y - odom.rpy.y, references.rpy.z - odom.rpy.z])
velocity_error = np.array([rospy.get_param('/surge_velocity') - odom.lin_vel.x, 0, 0]) 

def odom_callback(odom):
	global eta_1, eta_2, eta_1_init, eta_2_init
	lld_ned = rospy.get_param('ned_origin')
	eta_1 = [	pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[0], 
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[1],
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[2]]
	eta_2 = [	odom.rpy.x,
			odom.rpy.y,
			odom.rpy.z]
	if not eta_1_init:
		eta_1_init = eta_1
	if not eta_2_init:
		eta_2_init = eta_2

def state_callback(state):
	global strategy, current_task, task_changed, wp_index
	strategy = state.strategy  			#setta strategia
	if not current_task or current_task != state.task:
		task_changed = True 			 #flag per il timer
	current_task = state.task 			 #setta il task corrente
	wp_index = state.wp_index 			 #setta indice del wp
	
def set_yaw_ref(final_value): #TODO 
	global eta_2, time_start, task_changed, current_task
	yaw_angular_velocity = rospy.get_param('/yaw_angular_velocity')
	if not time_start or task_changed:
		time_start = time.time()					# init time_start
		task_changed = False
	dt = time.time() - time_start	
	ref = eta_2_init[2] + np.sign(final_value - eta_2[2])*yaw_angular_velocity*dt
	if abs(final_value) < abs(ref):
		return final_value
	else:
		return ref

#funzione che prende in ingresso i riferimenti e calcola gli errori
def ref_callback(ref):
	global strategy, current_task, eta_1, eta_2
	if current_task == 'YAW': 
		while eta_1 is None or eta_2 is None:
			pass 
		error_x = ref.pos.x - eta_1[0]
		error_y = ref.pos.y - eta_1[1]
		error_z = ref.pos.z - eta_1[2]
		error_pitch = wrap2pi(ref.rpy.y - eta_2[1])		
		error_yaw = wrap2pi(set_yaw_ref(ref.rpy.z) - eta_2[2])
		
#funzione che porta l'errore di posizione in terna body
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
	
	global position_error_body
	postion_error_body = np.dot (np.transpose(J1), position_error)
	
	return(position_error_body)

def control():
	rospy.init_node('control')
	#pub = rospy.Publisher('tau', Tau, queue_size = QUEUE_SIZE)
	rospy.Subscriber('odom', Odom, odom_callback)
	rospy.Subscriber('references', References, ref_callback)
	rospy.Subscriber('state', State, state_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass

