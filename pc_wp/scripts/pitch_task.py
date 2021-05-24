#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
from classes.Waypoint import Waypoint
from pc_wp.msg import References, State, Odom
from utils import get_waypoint, clear_vars

pitch_ref = None
pitch_angular_velocity = ripsy.get_param('/yaw_angular_velocity'/2) #ipotizzo velocità angolare in pitch , metà di quella in yaw

#all'inizio non si ha alcun riferimento per pitch in quanto non è noto a che punto della missione siamo - wp e pose - :
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
		eta_1_init = eta_1 #posa iniziale = prima stima 
	if not eta_2_init:
		eta_2_init = eta_2
	#print("eta_1: %s; eta_2: %s; eta_1_init: %s; eta_2_init: %s" % (eta_1, eta_2, eta_1_init, eta_2_init))
	print("PITCH: %s" % math.degrees(eta_2[1]))

def state_callback(state, pub):
	global prev_waypoint, next_waypoint, eta_1_init, eta_2_init, pitch_ref
	if state.task == 'PITCH':
		if not prev_waypoint and state.wp_index > 0:
			prev_waypoint = get_waypoint(state.wp_index)
			pos_ref = prev_waypoint
		else:
			while eta_1_init is None: #non avevamo già assegnato il valore di eta_1 a eta_1_init?
				pass
			pos_ref = eta_1_init		
		if not next_waypoint:
			next_waypoint = get_waypoint(state.wp_index+1)
		if not pitch_ref:
			pitch_ref = np.arctan2(	next_waypoint.eta_1[2] - eta_1[2],
						math.sqrt((next_waypoint.eta_1[0] - eta_1[0])**2+(next_waypoint.eta_1[1] - eta_1[1])**2)))
		references = References()
		references.pos.x = pos_ref[0]
		references.pos.y = pos_ref[1]
		references.pos.z = pos_ref[2]
		references.rpy.x = eta_2_init[0]
		references.rpy.y = eta_2_init[1]
		references.rpy.z = pitch_ref
		print("PITCH REFERENCE: %s" % int(round(math.degrees(pitch_ref))))
		pub.publish(references)
	elif state.task != 'PITCH':
		[prev_waypoint, next_waypoint, eta_1_init, eta_2_init, pitch_ref] = clear_vars([prev_waypoint, next_waypoint, eta_1_init, eta_2_init, pitch_ref])

def pitch_task():
	rospy.init_node('pitch_task')
	pub = rospy.Publisher('references', References, queue_size = QUEUE_SIZE)
	rospy.Subscriber('state', State, state_callback, pub)
	rospy.Subscriber('odom', Odom, odom_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		pitch_task()
	except rospy.ROSInterruptException:
		pass


