#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
from classes.Waypoint import Waypoint
from pc_wp.msg import References, State, Odom
from utils import get_waypoint, clear, isNone, wrap2pi

task = None
waypoint = None

eta_1 = None
eta_2 = None
eta_1_init = None
eta_2_init = None

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def odom_callback(odom):
	global task, eta_1, eta_2, eta_1_init, eta_2_init
	if task == 'HEAVE':
		lld_ned = rospy.get_param('ned_frame_origin')
		eta_1 = [	pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[0], 
				pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[1],
				pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[2]]
		eta_2 = [	wrap2pi(odom.rpy.x),
				wrap2pi(odom.rpy.y),
				wrap2pi(odom.rpy.z)]
		if not eta_1_init:
			eta_1_init = eta_1
		if not eta_2_init:
			eta_2_init = eta_2
	
def state_callback(state, pub):
	global task, eta_1, eta_2, eta_1_init, eta_2_init
	if state.task == 'HEAVE':
		task = state.task
		while isNone([eta_1, eta_2, eta_1_init, eta_2_init]):
			pass
		waypoint = get_waypoint(state.wp_index)
		if waypoint is not None:
			references = References()
			references.pos.x = eta_1_init[0]
			references.pos.y = eta_1_init[1]
			references.pos.z = waypoint.eta_1[2]
			references.rpy.x = eta_2_init[0]
			references.rpy.y = eta_2_init[1]
			references.rpy.z = eta_2_init[2]
			pub.publish(references)
	elif state.task != 'HEAVE':
		task = state.task
		if not isNone([eta_1_init, eta_2_init]):
			[eta_1_init, eta_2_init] = clear([eta_1_init, eta_2_init])
def heave_task():
	global QUEUE_SIZE
	rospy.init_node('heave_task')
	pub = rospy.Publisher('references', References, queue_size = QUEUE_SIZE)
	rospy.Subscriber('state', State, state_callback, pub)
	rospy.Subscriber('odom', Odom, odom_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		heave_task()
	except rospy.ROSInterruptException:
		pass
