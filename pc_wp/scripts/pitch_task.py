#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
from classes.Waypoint import Waypoint
from pc_wp.msg import References, State, Odom
from utils import get_waypoint, clear, isNone

task = None
wp_index = None
strategy = None
waypoint = None

eta_1 = None
eta_2 = None
eta_1_init = None
eta_2_init = None

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def odom_callback(odom, pub):
	global task, wp_index, strategy, waypoint, eta_1, eta_2, eta_1_init, eta_2_init
	if task == 'PITCH':
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
		references = References()
		references.pos.x = eta_1_init[0]
		references.pos.y = eta_1_init[1]
		references.pos.z = eta_1_init[2]
		references.rpy.x = eta_2_init[0] 
		if strategy == 1:
			if not waypoint:
				waypoint = get_waypoint(wp_index)
			references.rpy.y = np.arctan2(	waypoint.eta_1[2] - eta_1[2],
          						math.sqrt((waypoint.eta_1[0] - eta_1[0])**2 + (waypoint.eta_1[1] - eta_1[1])**2))
		elif strategy == 2:
			references.rpy.y = 0
		references.rpy.z = eta_2_init[2]
		pub.publish(references)
	elif task != 'PITCH':  
		if not isNone([waypoint, eta_1_init, eta_2_init]):
			[waypoint, eta_1_init, eta_2_init] = clear([waypoint, eta_1_init, eta_2_init])

def state_callback(state):
	global task, wp_index, strategy
	if state.task == 'PITCH':
		task = state.task
		wp_index = state.wp_index
		strategy = state.strategy
	elif state.task != 'PITCH':
		if not isNone([task, wp_index, strategy]):
			[task, wp_index, strategy] = clear([task, wp_index, strategy])

def pitch_task():
	rospy.init_node('pitch_task')
	pub = rospy.Publisher('references', References, queue_size = QUEUE_SIZE)
	rospy.Subscriber('state', State, state_callback)
	rospy.Subscriber('odom', Odom, odom_callback, pub)
	rospy.spin()

if __name__ == '__main__':
	try:
		pitch_task()
	except rospy.ROSInterruptException:
		pass
