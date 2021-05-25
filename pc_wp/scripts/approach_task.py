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
	
def state_callback(state, pub):
	global waypoint, eta_1_init, eta_2_init
	if state.task == 'APPROACH':
		while eta_1_init is None or eta_2_init is None:
				pass	
		if not waypoint:
			waypoint = get_waypoint(state.wp_index)
	
		#riferimento yaw
		rpy_ref[2] = np.arctan2(	waypoint.eta_1[1] - eta_1[1],  
						waypoint.eta_1[0] - eta_1[0])
		#riferimento pitch
		rpy_ref[1] = np.arctan2(	waypoint.eta_1[2] - eta_1[2],
						math.sqrt((waypoint.eta_1[0] - eta_1[0])**2+(waypoint.eta_1[1] - eta_1[1])**2)))
		#riferimento roll
		rpy_ref[0] = eta_2_init[0]

		#riferimento posizione x y z
		pos_ref[0] = waypoint.eta_1[0]  #posizione del prossimo waypoint
		pos_ref[1] = waypoint.eta_1[1]
		pos_ref[2] = waypoint.eta_1[2]
		
		#scrivo i riferimenti
		references = References()
		references.pos.x = pos_ref[0]  
		references.pos.y = pos_ref[1]
		references.pos.z = pos_ref[2]
		references.rpy.x = rpy_ref[0] #non lo useremo
		references.rpy.y = rpy_ref[1]
		references.rpy.z = rpy_ref[2]
		pub.publish(references)
	elif state.task != 'APPROACH':
		[waypoint, eta_1_init, eta_2_init] = clear_vars([waypoint, eta_1_init, eta_2_init])

def approach_task():
	rospy.init_node('approach_task')
	pub = rospy.Publisher('references', References, queue_size = QUEUE_SIZE)
	rospy.Subscriber('state', State, state_callback, pub)
	rospy.Subscriber('odom', Odom, odom_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		approach_task()
	except rospy.ROSInterruptException:
		pass