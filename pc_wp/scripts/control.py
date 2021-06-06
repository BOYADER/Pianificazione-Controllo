#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
import time
from pc_wp.msg import References, State, Odom, tau
from utils import wrap2pi, isNone, ned2body, projection, get_waypoint

task = None

eta_1 = None
eta_2 = None
ni_1 = None

eta_1_init = None
eta_2_init = None
ni_1_init = None

gains_P = [	rospy.get_param('K_P')['x'],
		rospy.get_param('K_P')['y'],
		rospy.get_param('K_P')['z'],
		rospy.get_param('K_P')['roll'],
		rospy.get_param('K_P')['pitch'],
		rospy.get_param('K_P')['yaw'],
		rospy.get_param('K_P')['ni_1.x']]
	
gains_I = [	rospy.get_param('K_I')['x'],
		rospy.get_param('K_I')['y'],
		rospy.get_param('K_I')['z'],
		rospy.get_param('K_I')['roll'],
		rospy.get_param('K_I')['pitch'],
		rospy.get_param('K_I')['yaw'],
		rospy.get_param('K_I')['ni_1.x']]

UP_SAT = rospy.get_param('UP_SAT')
DOWN_SAT = rospy.get_param('DOWN_SAT')

int_error = np.array([[0, 0, 0, 0, 0, 0]]).T

time_start_ref = None
time_start_pid = time.time()
reset_time = False

references = None

stop_set_reference = [	False,		# x
			False,		# y
			False,		# z
			False,		# roll
			False,		# pitch
			False]		# yaw

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

surge_reference_high = rospy.get_param('references/SURGE/high')
surge_reference_low = rospy.get_param('references/SURGE/low')
tolerance = rospy.get_param('references/SURGE/tolerance')

def odom_callback(odom):
	global eta_1, eta_2, ni_1
	lld_ned = rospy.get_param('ned_frame_origin')
	eta_1 = [	pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[0], 
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[1],
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[2]]
	eta_2 = [	odom.rpy.x,
			odom.rpy.y,
			odom.rpy.z]
	ni_1 = [	odom.lin_vel.x,
			odom.lin_vel.y,
			odom.lin_vel.z]	

def state_callback(state, pub):
	global task, eta_1, eta_2, ni_1, eta_1_init, eta_2_init, ni_1_init, time_start_ref, references, tolerance, surge_reference_high, surge_reference_low, reset_time, stop_set_reference, int_error
	if not task or task != state.task:
		references = None
		int_error = np.array([[0, 0, 0, 0, 0, 0]]).T
		time_start_ref = time.time()
		while isNone([eta_1, eta_2, ni_1]):
			pass
		eta_1_init = eta_1
		eta_2_init = eta_2	
		ni_1_init = ni_1
		reset_time = False
		stop_set_reference = [	False,		
					False,
					False,		
					False,	
					False,
					False]
	task = state.task
	while isNone([eta_1_init, eta_2_init, references]):
		pass
	error_x_ned = references.pos.x - eta_1[0]
	error_y_ned = references.pos.y - eta_1[1]
	error_z_ned = references.pos.z - eta_1[2]
	error_roll = wrap2pi(references.rpy.x - eta_2[0])		
	error_pitch = wrap2pi(references.rpy.y - eta_2[1])
	error_yaw = wrap2pi(references.rpy.z - eta_2[2])
	error_ni_1_x = None
	if state.task == 'YAW':
		if not stop_set_reference[5]:
			error_yaw = wrap2pi(set_reference(eta_2_init[2], eta_2[2], references.rpy.z, 'YAW', 5) - eta_2[2])
		else:
			error_yaw = references.rpy.z - eta_2[2]
	elif state.task == 'PITCH':
		if not stop_set_reference[4]:
			error_pitch = wrap2pi(set_reference(eta_2_init[1], eta_2[1], references.rpy.y, 'PITCH', 4) - eta_2[1])
		else:
			error_pitch = wrap2pi(references.rpy.y - eta_2[1])
	elif state.task == 'HEAVE':
		if not stop_set_reference[2]:
			error_z_ned = set_reference(eta_1_init[2], eta_1[2], references.pos.z, 'HEAVE', 2) - eta_1[2]
		else:
			error_z_ned = references.pos.z - eta_1[2]
	elif state.task == 'APPROACH':
		if not stop_set_reference[0]:
			error_x_ned = set_reference(eta_1_init[0], eta_1[0], references.pos.x, 'APPROACH', 0) - eta_1[0]
		else:
			error_x_ned = references.pos.x - eta_1[0]
		if not stop_set_reference[1]:
			error_y_ned = set_reference(eta_1_init[1], eta_1[1], references.pos.y, 'APPROACH', 1) - eta_1[1]
		else:
			error_y_ned = references.pos.y - eta_1[1]
		if not stop_set_reference[2]:
			error_z_ned = set_reference(eta_1_init[2], eta_1[2], references.pos.z, 'APPROACH', 2) - eta_1[2]
		else:
			error_z_ned = references.pos.z - eta_1[2]
		error_pitch = wrap2pi(references.rpy.y - eta_2[1])
	elif state.task == 'SURGE':
		if references.lin_vel.x == surge_reference_low and not reset_time:
			time_start_ref = time.time()
			ni_1_init = ni_1
			reset_time = True
			stop_set_reference[0] = False
		if not stop_set_reference[0]:
			error_ni_1_x = set_reference(ni_1_init[0], ni_1[0], references.lin_vel.x, 'SURGE', 0) - ni_1[0]
		else:
			error_ni_1_x = references.lin_vel.x - ni_1[0]
		waypoint = get_waypoint(state.wp_index)
		u = np.array(eta_1) - np.array(eta_1_init)
		v = np.array(waypoint.eta_1) - np.array(eta_1_init)
		u_on_v = projection(u, v)
		[error_x_ned, error_y_ned, error_z_ned] = u_on_v - u
	error_xyz_ned = [error_x_ned, error_y_ned, error_z_ned]
	[error_x_body, error_y_body, error_z_body] = ned2body(error_xyz_ned, eta_2)
	error_pose_body = np.array([error_x_body, error_y_body, error_z_body, error_roll, error_pitch, error_yaw])
	print("eta_1: [%s, %s, %s], eta_2: [%s, %s, %s]" % (	round(eta_1[0], 2),
								round(eta_1[1], 2),
								round(eta_1[2], 2),
								round(math.degrees(eta_2[0]), 1),
								round(math.degrees(eta_2[1]), 1),
								round(math.degrees(eta_2[2]), 1)))
	print("ni_1: [%s, %s, %s]" % (	round(ni_1[0], 2),
					round(ni_1[1], 2),
					round(ni_1[2], 2)))
	u = pid(error_pose_body, error_ni_1_x)
	control = tau()
	control.tau.force.x = np.float64(u[0]).item()
	control.tau.force.y = np.float64(u[1]).item()
	control.tau.force.z = np.float64(u[2]).item()
	control.tau.torque.x = 0  					# not actuated
 	control.tau.torque.y = np.float64(u[4]).item()
	control.tau.torque.z = np.float64(u[5]).item()
	pub.publish(control)
	print("force: [%s, %s, %s]\ntorque: [%s, %s, %s]" % (	round(control.tau.force.x, 2),
								round(control.tau.force.y, 2),
								round(control.tau.force.z, 2),
								round(control.tau.torque.x, 2),
								round(control.tau.torque.y, 2),
								round(control.tau.torque.z, 2)))

def set_reference(init_value, actual_value, final_value, task_value, index):				# set time varying reference signal
	global time_start_ref, stop_set_reference
	string_param = 'task_velocity_reference_list/' + task_value
	velocity_reference = rospy.get_param(string_param)
	while not time_start_ref:
		pass
	dt = time.time() - time_start_ref
	time_start_ref = time.time()
	if task_value == 'PITCH' or task_value == 'YAW':
		sign = np.sign(wrap2pi(final_value - actual_value))
	else:
		sign = np.sign(final_value - actual_value)
	reference = actual_value + sign * velocity_reference * dt 
	if (final_value > 0 and reference > 0 and sign == 1 and reference > final_value) or (final_value > 0 and reference > 0 and sign == -1 and reference < final_value) or (final_value < 0 and reference < 0 and sign == 1 and reference > final_value) or (final_value < 0 and reference < 0 and sign == -1 and reference < final_value):   
		stop_set_reference[index] = True
		return final_value
	else:
		return reference

def pid(error_pose_body, error_ni_1_x):
	global int_error, UP_SAT, DOWN_SAT, gains_P, gains_I, time_start_pid
	if error_ni_1_x is not None:
		pid_error = np.array([[	error_ni_1_x,
					error_pose_body[1],
					error_pose_body[2],
					error_pose_body[3],
					error_pose_body[4],
					error_pose_body[5]]])
		K_P = [	gains_P[6],
			gains_P[1],
			gains_P[2],
			gains_P[3],
			gains_P[4],
			gains_P[5]]
		K_I = [	gains_I[6],
			gains_I[1],
			gains_I[2],
			gains_I[3],
			gains_I[4],
			gains_I[5]]
	else:
		pid_error = np.array([error_pose_body])
		K_P = [	gains_P[0],
			gains_P[1],
			gains_P[2],
			gains_P[3],
			gains_P[4],
			gains_P[5]]
		K_I = [	gains_I[0],
			gains_I[1],
			gains_I[2],
			gains_I[3],
			gains_I[4],
			gains_I[5]]
	pid_error = pid_error.T
	dt = time.time() - time_start_pid
	time_start_pid = time.time()
	int_error = int_error + np.dot(pid_error, dt)
	u = np.dot(np.diag(K_P), pid_error) + np.dot(np.diag(K_I), int_error)
	for i in range(0, len(u)):
		if u[i] > UP_SAT:
			u[i] = UP_SAT
			int_error = int_error - np.dot(pid_error, dt) # anti reset wind up
		elif u[i] < DOWN_SAT:
			u[i] = DOWN_SAT
			int_error = int_error - np.dot(pid_error, dt)
	return u

def ref_callback(ref):
	global references
	references = ref

def control():
	rospy.init_node('control')
	pub = rospy.Publisher('tau', tau, queue_size = QUEUE_SIZE)
	rospy.Subscriber('odom', Odom, odom_callback)
	rospy.Subscriber('references', References, ref_callback)
	rospy.Subscriber('state', State, state_callback, pub)
	rospy.spin()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass

