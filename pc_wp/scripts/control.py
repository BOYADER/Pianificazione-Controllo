#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
import time
from pc_wp.msg import References, State, Odom

strategy = None
current_task = None
task_changed = False		

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
	global strategy, current_task, task_changed, wp_index
	strategy = state.strategy
	if not current_task or current_task != state.task:
		task_changed = True
	current_task = state.task
	wp_index = state.wp_index
	
def yaw_ref(yaw_ref):
	global eta_2, time_start, task_changed, current_task, yaw_angular_velocity
	if not time_start or task_changed:
		time_start = time.time()
		task_changed = False
	dt = time.time() - time_start	
	ref = eta_2[2] + np.sign(yaw_ref - eta_2[2])*math.degrees(yaw_angular_velocity)*dt
	print("ref_mini: %s, yaw_ref_final: %s" % (str(ref), str(yaw_ref)))
	r = 0.6*ref + 0.4*eta_2[2]
	if (yaw_ref >= 0 and r > yaw_ref) or (yaw_ref < 0 and r < yaw_ref):
		return yaw_ref
	else:
		return r

def ref_callback(ref):
	global strategy, current_task, eta_1, eta_2
	if current_task == 'YAW': 
		error_x = ref.pos.x - eta_1[0]
		error_y = ref.pos.y - eta_1[1]
		error_z = ref.pos.z - eta_1[2]
		error_pitch = ref.rpy.y - eta_2[1] #wrap to pi		
		#error_yaw = yaw_ref(ref.rpy.z) - eta_2[2]
		#r = 0.9*yaw_ref(ref.rpy.z) + 0.1*eta_2[2]
		u = yaw_ref(ref.rpy.z)
		print("riferimento da dare al cazzo di PID: %s" % str(u))

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

