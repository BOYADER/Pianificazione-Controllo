#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
import time
from pc_wp.msg import References, State, Odom

strategy = None
current_task = None
last_task_rcvd = None		# last State.task received

yaw_angular_velocity = rospy.get_param('/yaw_angular_velocity')

wp_index = None

eta_1 = []
eta_2 = []
lld_ned = rospy.get_param('ned_origin')

eta_1_init = None
eta_2_init = None

time_start = None

def odom_callback(odom):
	global eta_1, eta_2, lld_ned, eta_1_init, eta_2_init
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
	global strategy, current_task, last_task_rcvd, wp_index
	strategy = state.strategy
	if current_task:
		last_task_rcvd = current_task
	current_task = state.task
	wp_index = state.wp_index
	
def yaw_ref(yaw_ref):
	global eta_2, time_start, last_task_rcvd, current_task, yaw_angular_velocity
	if not last_task_rcvd or last_task_rcvd != current_task:
		time_start = time.time()
	dt = time.time() - time_start	
	ref = eta_2[2] + np.sign(yaw_ref - eta_2[2])*math.degrees(yaw_angular_velocity)*dt
	print("ref_mini: %s, yaw_ref_final: %s" % (str(ref), str(yaw_ref)))
	if (yaw_ref >= 0 and ref > yaw_ref) or (yaw_ref < 0 and ref < yaw_ref):
		return yaw_ref
	else:
		return ref

def ref_callback(ref):
	global strategy, current_task, eta_1, eta_2
	if current_task == 'YAW': 
		error_x = ref.pos.x - eta_1[0]
		error_y = ref.pos.y - eta_1[1]
		error_z = ref.pos.z - eta_1[2]
		error_pitch = ref.rpy.y - eta_2[1] #wrap to pi		
		#error_yaw = yaw_ref(ref.rpy.z) - eta_2[2]
		r = yaw_ref(ref.rpy.z)
		print("riferimento da dare al cazzo di PID: %s" % str(r))

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

