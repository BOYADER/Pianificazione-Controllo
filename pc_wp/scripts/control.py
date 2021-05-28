#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
import time
from pc_wp.msg import References, State, Odom
from utils import wrap2pi, isNone, ned2body

strategy = None
task = None
wp_index = None

eta_1 = None
eta_2 = None
eta_1_init = None
eta_2_init = None
ni_1 = None

task_changed = False

time_start = None

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

def set_tv_reference(init_value, actual_value, final_value):				# set time varying reference signal
	global task, time_start, task_changed
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


##################################### DA FARE ############################################################
#funzione che prende in ingresso l'errore in body e sputa fuori le tau da pubblicare sul topic wrench
def pid(error_body):                             #tutte cose da definire come parametri in un file yaml
    global int_error, dt, UP_SAT, DOWN_SAT, K_P, K_I  #integrale dell'errore, intervallo di tempo, saturazione superiore, saturazione inferiore, matrice dei guadagni P, matrice dei guadagni I 
    int_error = int_error + (error_body * dt)     #calcolo integrale dell'errore
    u = np.dot(K_P, error_body) + np.dot(K_I, int_error) #controllore
   # controllo che nessuna tau superi la saturazione superiore o inferiore
    for i in range(0, 6):
        if u[i] > UP_SAT:
            u[i] = UP_SAT
            int_error = int_error - (error_body * dt) #anti reset windup
        elif u[i] < DOWN_SAT:
            u[i] = DOWN_SAT
            int_error = int_error - (error_body * dt)
    return u

def ref_callback(ref, pub):
	global strategy, task, eta_1, eta_2, eta_1_init, eta_2_init, ni_1
	while isNone([eta_1, eta_2, eta_1_init, eta_2_init, ni_1]):
		pass
	error_x = ref.pos.x - eta_1[0]
	error_y = ref.pos.y - eta_1[1]
	error_z = ref.pos.z - eta_1[2]
	error_roll = wrap2pi(ref.rpy.x - eta_2[0])		
	error_pitch = wrap2pi(ref.rpy.y - eta_2[1])
	error_yaw = wrap2pi(ref.rpy.z - eta_2[2])
	if task == 'YAW':
		error_yaw = wrap2pi(set_tv_reference(eta_2_init[2], eta_2[2], ref.rpy.z) - eta_2[2])
	elif task == 'PITCH':
		error_pitch = wrap2pi(set_tv_reference(eta_2_init[1], eta_2[1], ref.rpy.y) - eta_2[1])
	elif task == 'HEAVE':
		error_z = set_tv_reference(eta_1_init[2], eta_1[2], ref.pos.z) - eta_1[2]
	elif task == 'SURGE':
		error_x = ref.lin_vel.x - ni_1[0]
		error_y = ref.pos.y - eta_1[1]			###### da fare
		error_z = ref.pos.z - eta_1[2]			###### da fare
	error_ned = np.array([error_x, error_y, error_z, error_roll, error_pitch, error_yaw])
	error_body = ned2body(error_ned, eta_2)
	tau = pid(error_body)
	pub.publish(tau)
####################################################################################################################

def control():
	rospy.init_node('control')
	pub = rospy.Publisher('tau', Tau, queue_size = QUEUE_SIZE)
	rospy.Subscriber('odom', Odom, odom_callback)
	rospy.Subscriber('references', References, ref_callback, pub)
	rospy.Subscriber('state', State, state_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass

