#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
import time
from pc_wp.msg import References, State, Odom, tau
from utils import wrap2pi, isNone, ned2body, projection

strategy = None
task = None
wp_index = None

eta_1 = None
eta_2 = None
eta_1_init = None
eta_2_init = None
ni_1 = None

K_P = rospy.get_param('K_P')
K_I = rospy.get_param('K_I')

UP_SAT = rospy.get_param('UP_SAT')
DOWN_SAT = rospy.get_param('DOWN_SAT')

int_error = np.array([[0, 0, 0, 0, 0, 0]]).T

time_start_ref = None
time_start_pid = time.time()

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def odom_callback(odom, pub):
	global task, wp_index, references, eta_1, eta_2, eta_1_init, eta_2_init, ni_1
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
	while isNone([eta_1_init, eta_2_init]):
		pass
	error_x_ned = references.pos.x - eta_1[0]
	error_y_ned = references.pos.y - eta_1[1]
	error_z_ned = references.pos.z - eta_1[2]
	error_roll = wrap2pi(references.rpy.x - eta_2[0])		
	error_pitch = wrap2pi(references.rpy.y - eta_2[1])
	error_yaw = wrap2pi(references.rpy.z - eta_2[2])
	error_ni_1_x = None
	if task == 'YAW':
		error_yaw = wrap2pi(set_reference(eta_2_init[2], eta_2[2], references.rpy.z) - eta_2[2])
	elif task == 'PITCH':
		error_pitch = wrap2pi(set_reference(eta_2_init[1], eta_2[1], references.rpy.y) - eta_2[1])
	elif task == 'HEAVE':
		error_z_ned = set_reference(eta_1_init[2], eta_1[2], references.pos.z) - eta_1[2]
	elif task == 'SURGE':
		error_ni_1_x = references.lin_vel.x - ni_1[0]
		waypoint = get_waypoint(wp_index)
		u = eta_1 - eta_1_init
		v = waypoint.eta_1 - eta_1_init
		u_on_v = projection(u, v)
		error_y_ned = (u_on_v - u)[1]
		error_z_ned = (u_on_v - u)[2] 
	error_xyz_ned = [error_x_ned, error_y_ned, error_z_ned]
	[error_x_body, error_y_body, error_z_body] = ned2body(error_xyz_ned, eta_2)
	error_pose_body = [error_x_body, error_y_body, error_z_body, error_roll, error_pitch, error_yaw]
	print("error_pose_body: %s" % error_pose_body)
	u = pid(error_pose_body, error_ni_1_x)
	tau_ = tau()
	tau_.tau.force.x = u[0]
	tau_.tau.force.y = u[1]
	tau_.tau.force.z = u[2]
	tau_.tau.torque.x = u[3]  # roll non verra' usato
 	tau_.tau.torque.y = u[4]
	tau_.tau.torque.z = u[5]
	pub.publish(tau_)

def state_callback(state):
	global strategy, task, wp_index, eta_1, eta_2, eta_1_init, eta_2_init, time_start_ref
	if not task or task != state.task:
		time_start_ref = time.time()
		while isNone([eta_1, eta_2]):
			pass
		eta_1_init = eta_1
		eta_2_init = eta_2	
	strategy = state.strategy
	task = state.task
	wp_index = state.wp_index

def set_reference(init_value, actual_value, final_value):				# set time varying reference signal
	global task, time_start_ref
	string_param = 'task_velocity_reference_list/' + task
	velocity_reference = rospy.get_param(string_param)
	while not time_start_ref:
		pass
	dt = time.time() - time_start_ref
	reference = init_value + np.sign(final_value - actual_value) * velocity_reference * dt
	if abs(reference) > abs(final_value):
		#print("final_value: %s" % (final_value))
		return final_value
	else:
		#print("reference: %s" % (reference))
		return reference

def pid(error_pose_body, error_ni_1_x):
	global int_error, UP_SAT, DOWN_SAT, K_P, K_I, time_start_pid
	if error_ni_1_x is not None:
		pid_error = np.array([	error_ni_1_x,
					error_pose_body[1],
					error_pose_body[2],
					error_pose_body[3],
					error_pose_body[4],
					error_pose_body[5]])
	else:
		pid_error = np.array(error_pose_body)
	pid_error = pid_error.T
	dt = time.time() - time_start_pid
	time_start_pid = time.time()
	int_error = int_error + np.dot(pid_error, dt)
	u = np.dot(np.diag(K_P), pid_error) + np.dot(np.diag(K_I), int_error)
	"""for i in range(0, 6):
		if u[i] > UP_SAT:
			u[i] = UP_SAT
			int_error = int_error - (error_body * dt) # anti reset wind up
		elif u[i] < DOWN_SAT:
			u[i] = DOWN_SAT
			int_error = int_error - (error_body * dt)"""
	return u

def ref_callback(ref):
	global references
	references = ref

def control():
	rospy.init_node('control')
	pub = rospy.Publisher('tau', tau, queue_size = QUEUE_SIZE)
	rospy.Subscriber('odom', Odom, odom_callback, pub)
	rospy.Subscriber('references', References, ref_callback)
	rospy.Subscriber('state', State, state_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass

