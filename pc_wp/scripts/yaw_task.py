#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
from classes.Waypoint import Waypoint
from pc_wp.msg import References, State, Odom
from utils import get_waypoint, clear_vars


yaw_ref = None
yaw_angular_velocity = rospy.get_param('/yaw_angular_velocity')

prev_waypoint = None
next_waypoint = None
eta_1 = []
eta_2 = []

eta_1_init = None
eta_2_init = None

QUEUE_SIZE = rospy.get_param('/QUEUE_SIZE')

def odom_callback(odom):
	global eta_1, eta_2, eta_1_init, eta_2_init
	lld_ned = rospy.get_param('ned_origin')
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
	#print("eta_1: %s; eta_2: %s; eta_1_init: %s; eta_2_init: %s" % (eta_1, eta_2, eta_1_init, eta_2_init))
	print("YAW: %s" % math.degrees(eta_2[2]))

def state_callback(state, pub):
	global prev_waypoint, next_waypoint, eta_1_init, eta_2_init, yaw_ref
	if state.task == 'YAW':
		if not prev_waypoint and state.wp_index > 0:
			prev_waypoint = get_waypoint(state.wp_index)
			pos_ref = prev_waypoint
		else:
			while eta_1_init is None:
				pass
			pos_ref = eta_1_init		
		if not next_waypoint:
			next_waypoint = get_waypoint(state.wp_index+1)
		if not yaw_ref:
			yaw_ref = np.arctan2(	next_waypoint.eta_1[1] - eta_1[1],
						next_waypoint.eta_1[0] - eta_1[0])
		references = References()
		references.pos.x = pos_ref[0]
		references.pos.y = pos_ref[1]
		references.pos.z = pos_ref[2]
		references.rpy.x = eta_2_init[0]
		references.rpy.y = eta_2_init[1]
		references.rpy.z = yaw_ref
		print("YAW REFERENCE: %s" % int(round(math.degrees(yaw_ref))))
		pub.publish(references)
	elif state.task != 'YAW':
		[prev_waypoint, next_waypoint, eta_1_init, eta_2_init, yaw_ref] = clear_vars([prev_waypoint, next_waypoint, eta_1_init, eta_2_init, yaw_ref])

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
