#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
from classes.Waypoint import Waypoint
from pc_wp.msg import References, State, Odom
from utils import get_waypoint, clear_vars

waypoint = None

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
	#print("YAW: %s" % math.degrees(eta_2[2]))

def state_callback(state, pub):
	global waypoint, eta_1_init, eta_2_init
	if state.task == 'PITCH':
		while eta_1_init is None or eta_2_init is None:
				pass
		pos_ref = eta_1_init
		rpy_ref = eta_2_init	
		if not waypoint:
			waypoint = get_waypoint(state.wp_index)
		#riferimento pitch
		rpy_ref[1] = np.arctan2(  waypoint.eta_1[2] - eta_1[2],
            math.sqrt((waypoint.eta_1[0] - eta_1[0])**2+(waypoint.eta_1[1] - eta_1[1])**2)))
		
		references = References()
		references.pos.x = pos_ref[0]
		references.pos.y = pos_ref[1]
		references.pos.z = pos_ref[2]
		references.rpy.x = rpy_ref[0] #non lo useremo
		references.rpy.y = rpy_ref[1]
		references.rpy.z = rpy_ref[2]
		print("PITCH REFERENCE: %s" % int(round(math.degrees(rpy_ref[1]))))
		pub.publish(references)
	elif state.task != 'PITCH':
		[waypoint, eta_1_init, eta_2_init] = clear_vars([waypoint, eta_1_init, eta_2_init])

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
