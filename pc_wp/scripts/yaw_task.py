#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
from classes.Waypoint import Waypoint
from pc_wp.msg import References, State, Odom
from utils import get_waypoint, clear, isNone

task = None
waypoint = None

eta_1 = None
eta_2 = None
eta_1_init = None
eta_2_init = None

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def odom_callback(odom): 
	global task, eta_1, eta_2, eta_1_init, eta_2_init
	if task == 'YAW':
		lld_ned = rospy.get_param('ned_frame_origin')
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

def state_callback(state, pub):
	global task, waypoint, eta_1, eta_2, eta_1_init, eta_2_init
	if state.task == 'YAW':  
		task = state.task
		while isNone([eta_1_init, eta_2_init]):
			pass	
		references = References()
		references.pos.x = eta_1_init[0]
		references.pos.y = eta_1_init[1]
		references.pos.z = eta_1_init[2]		
		references.rpy.x = eta_2_init[0] 	
		references.rpy.y = eta_2_init[1]
		if not waypoint:
			waypoint = get_waypoint(state.wp_index)
		references.rpy.z = np.arctan2(	waypoint.eta_1[1] - eta_1[1],
						waypoint.eta_1[0] - eta_1[0])
		pub.publish(references)
		
	elif state.task != 'YAW':  
		if not isNone([task, waypoint, eta_1, eta_2, eta_1_init, eta_2_init]):
			[task, waypoint, eta_1, eta_2, eta_1_init, eta_2_init] = clear([task, waypoint, eta_1, eta_2, eta_1_init, eta_2_init])

def yaw_task():
	rospy.init_node('yaw_task')
	pub = rospy.Publisher('references', References, queue_size = QUEUE_SIZE)
	rospy.Subscriber('state', State, state_callback, pub)
	rospy.Subscriber('odom', Odom, odom_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		yaw_task()
	except rospy.ROSInterruptException:
		pass
