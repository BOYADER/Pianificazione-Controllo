#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
import time
from pc_wp.msg import References, State, Odom
from utils import wrap2pi, isNone

strategy = None
task = None
wp_index = None

eta_1 = None
eta_2 = None
eta_1_init = None
eta_2_init = None

task_changed = False

time_start = None

#errori
global position_error_ned, pose_error, velocity_error
position_error_ned = np.array([references.pose.x - odom.lld.x, references.pose.y - odom.pose.y, references.pose.z - odom.pose.z])
pose_error = np.array([references.rpy.x - odom.rpy.x, references.rpy.y - odom.rpy.y, references.rpy.z - odom.rpy.z])
velocity_error = np.array([rospy.get_param('surge_velocity') - odom.lin_vel.x, 0, 0]) 

def odom_callback(odom):
	global eta_1, eta_2
	lld_ned = rospy.get_param('ned_frame_origin')
	eta_1 = [	pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[0], 
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[1],
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[2]]
	eta_2 = [	odom.rpy.x,
			odom.rpy.y,
			odom.rpy.z]

def state_callback(state):
	global strategy, task, wp_index, task_changed, eta_1_init, eta_2_init		
	if not task or task != state.task:
		task_changed = True 	
		while isNone([eta_1, eta_2]):
			pass
		eta_1_init = eta_1
		eta_2_init = eta_2	
	strategy = state.strategy
	task = state.task
	wp_index = state.wp_index

def set_tv_reference(init_value, actual_value, final_value, task):				# set time varying reference signal
	global time_start, task_changed
	string_param = 'task_velocity_reference_list/' + task
	velocity_reference = rospy.get_param(string_param)
	if not time_start or task_changed:
        	time_start = time.time()  # init time_start
		task_changed = False
	dt = time.time() - time_start
	tv_reference = init_value + np.sign(final_value - actual_value) * velocity_reference * dt
	if abs(final_value) < abs(tv_reference):
		return final_value
	else:
		return tv_reference

def ref_callback(ref):
	global strategy, task, eta_1, eta_2, eta_1_init, eta_2_init
	while isNone([eta_1, eta_2, eta_1_init, eta_2_init]):
		pass
	if task == 'YAW':
		error_x = ref.pos.x - eta_1[0]
		error_y = ref.pos.y - eta_1[1]
		error_z = ref.pos.z - eta_1[2]
		error_pitch = wrap2pi(ref.rpy.y - eta_2[1])
		error_yaw = wrap2pi(set_tv_reference(eta_init_2[2], eta_2[2], ref.rpy.z, task) - eta_2[2])
	if task == 'PITCH':
        while eta_1 is None or eta_2 is None:
            pass
        error_x = ref.pos.x - eta_1[0]
        error_y = ref.pos.y - eta_1[1]
        error_z = ref.pos.z - eta_1[2]
        error_pitch = wrap2pi(set_mini_ref(ref.rpy.y) - eta_2[1])
        error_yaw = wrap2pi(ref.rpy.z - eta_2[2])

    if task == 'SURGE': #TODO
        while eta_1 is None or eta_2 is None:
            pass
        error_x = ref.pos.x - eta_1[0]
        error_y = ref.pos.y - eta_1[1]
        error_z = ref.pos.z - eta_1[2]
        error_pitch = wrap2pi(ref.rpy.y - eta_2[1])
        error_yaw = wrap2pi(set_yaw_ref(ref.rpy.z) - eta_2[2])
        
    if task == 'HEAVE':
        while eta_1 is None or eta_2 is None:
            pass
        error_x = ref.pos.x - eta_1[0]
        error_y = ref.pos.y - eta_1[1]
        error_z = set_mini_ref(ref.pos.z) - eta_1[2]
        error_pitch = wrap2pi(ref.rpy.y - eta_2[1])
        error_yaw = wrap2pi(ref.rpy.z - eta_2[2])
    if task == 'APPROACH': #TODO
        while eta_1 is None or eta_2 is None:
            pass
        error_x = ref.pos.x - eta_1[0]
        error_y = ref.pos.y - eta_1[1]
        error_z = set_mini_ref(ref.pos.z) - eta_1[2]
        error_pitch = wrap2pi(ref.rpy.y - eta_2[1])
        error_yaw = wrap2pi(ref.rpy.z - eta_2[2])
        
    #alla fine metto tutto in un vettore colonna
    error = np.array([[error_x, error_y, error_z, error_yaw, error_pitch, error_roll]]).T
    return error

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

