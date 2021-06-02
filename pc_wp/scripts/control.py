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
eta_1_init = None
eta_2_init = None
ni_1 = None

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

references = None

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def odom_callback(odom):
	global eta_1, eta_2, ni_1
	lld_ned = rospy.get_param('ned_frame_origin')
	#print("NOI ODOM %s" % odom.lld)
	eta_1 = [	pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[0], 
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[1],
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[2]]
	eta_2 = [	odom.rpy.x,
			odom.rpy.y,
			odom.rpy.z]
	ni_1 = [	odom.lin_vel.x,
			odom.lin_vel.y,
			odom.lin_vel.z]
	print("ni_1.x: %s" % (ni_1[0]))
	

def state_callback(state, pub):
	global task, eta_1, eta_2, eta_1_init, eta_2_init, ni_1, time_start_ref, references
	if not task or task != state.task:
		references = None
		int_error = np.array([[0, 0, 0, 0, 0, 0]]).T
		time_start_ref = time.time()
		while isNone([eta_1, eta_2]):
			pass
		eta_1_init = eta_1
		eta_2_init = eta_2	
	task = state.task
	while isNone([eta_1_init, eta_2_init, references]):
		pass
	#print(references)
	#print("eta_1: %s, %s, %s" % (eta_1[0], eta_1[1], eta_1[2]))
	error_x_ned = references.pos.x - eta_1[0]
	error_y_ned = references.pos.y - eta_1[1]
	error_z_ned = references.pos.z - eta_1[2]
	error_roll = wrap2pi(references.rpy.x - eta_2[0])		
	error_pitch = wrap2pi(references.rpy.y - eta_2[1])
	error_yaw = wrap2pi(references.rpy.z - eta_2[2])
	error_ni_1_x = None
	if state.task == 'YAW':
		error_yaw = wrap2pi(set_reference(eta_2_init[2], eta_2[2], references.rpy.z) - eta_2[2])
	elif state.task == 'PITCH':
		error_pitch = wrap2pi(set_reference(eta_2_init[1], eta_2[1], references.rpy.y) - eta_2[1])
	elif state.task == 'HEAVE':
		error_z_ned = set_reference(eta_1_init[2], eta_1[2], references.pos.z) - eta_1[2]
	elif state.task == 'APPROACH':
		error_x_ned = set_reference(eta_1_init[0], eta_1[0], references.pos.x) - eta_1[0]
		error_y_ned = set_reference(eta_1_init[1], eta_1[1], references.pos.y) - eta_1[1]
		error_z_ned = set_reference(eta_1_init[2], eta_1[2], references.pos.z) - eta_1[2]
	elif state.task == 'SURGE':
		error_ni_1_x = references.lin_vel.x - ni_1[0]
		print("error_ni_x: %s" % error_ni_1_x)
		error_x_ned = eta_1_init[0] - eta_1[0]
		waypoint = get_waypoint(state.wp_index)
		u = np.array(eta_1) - np.array(eta_1_init)
		v = np.array(waypoint.eta_1) - np.array(eta_1_init)
		print("u: %s, v: %s, eta_1_init: %s" % (u, v, eta_1_init))
		u_on_v = projection(u, v)
		error_ned = u_on_v - u
		print("error_ned: %s " % error_ned)
	if state.task == 'SURGE':
		error_xyz_ned = error_ned
	else:
		error_xyz_ned = [error_x_ned, error_y_ned, error_z_ned]
	#print("error_xyz_ned: [%s, %s, %s]" % (round(error_xyz_ned[0]), round(error_xyz_ned[1]), error_xyz_ned[2]))
	[error_x_body, error_y_body, error_z_body] = ned2body(error_xyz_ned, eta_2)
	error_pose_body = np.array([error_x_body, error_y_body, error_z_body, error_roll, error_pitch, error_yaw])
	print("eta_1: %s, eta_2: %s" % (eta_1, eta_2))
	print("task: %s, error_pose_body: [%s, %s, %s, %s, %s, %s]" % (task,error_pose_body[0],error_pose_body[1],error_pose_body[2],error_pose_body[3],error_pose_body[4],error_pose_body[5]))
	u = pid(error_pose_body, error_ni_1_x)
	tau_ = tau()
	tau_.tau.force.x = np.float64(u[0]).item()
	tau_.tau.force.y = np.float64(u[1]).item()
	tau_.tau.force.z = np.float64(u[2]).item()
	tau_.tau.torque.x = 0  # roll non verra' usato
 	tau_.tau.torque.y = np.float64(u[4]).item()
	tau_.tau.torque.z = np.float64(u[5]).item()
	print(tau_)
	pub.publish(tau_)

def set_reference(init_value, actual_value, final_value):				# set time varying reference signal
	global task, time_start_ref
	string_param = 'task_velocity_reference_list/' + task
	velocity_reference = rospy.get_param(string_param)
	while not time_start_ref:
		pass
	dt = time.time() - time_start_ref
	reference = init_value + np.sign(final_value - actual_value) * velocity_reference * dt
	if (final_value < 0 and reference < final_value) or (final_value > 0 and  reference > final_value):   
		print("CIAOOOOOOOOOOOOOOOO!")
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
	print("pid_error %s" % pid_error)
	pid_error = pid_error.T
	dt = time.time() - time_start_pid
	time_start_pid = time.time()
	int_error = int_error + np.dot(pid_error, dt)
	u = np.dot(np.diag(K_P), pid_error) + np.dot(np.diag(K_I), int_error)
	for i in range(0, 6):
		if u[i] > UP_SAT:
			u[i] = UP_SAT
			#int_error = int_error - np.dot(pid_error, dt) # anti reset wind up
		elif u[i] < DOWN_SAT:
			u[i] = DOWN_SAT
			#int_error = int_error - np.dot(pid_error, dt)
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

