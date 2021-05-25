#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
from classes.Waypoint import Waypoint
from pc_wp.msg import References, State, Odom
from utils import get_waypoint, clear_vars

#variabili globali
waypoint = None
eta_1 = []
eta_2 = []
eta_1_init = None
eta_2_init = None

QUEUE_SIZE = rospy.get_param('/QUEUE_SIZE')

#funzione che salva in eta_1 e eta_2 i valori attuali di posizione e orientazione dell'AUV e imposta eta_1_init e eta_2_init con i primi valori ricevuti all'attivazione di YAW task
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
	global waypoint, eta_1_init, eta_2_init
	if state.task == 'YAW':  #se nel messaggio state il campo task e' YAW
		while eta_1_init is None or eta_2_init is None:  #attendo che vengano inizializzate
				pass
		pos_ref = eta_1_init
		rpy_ref = eta_2_init	
		if not waypoint: #inizializzo il waypoint da raggiungere
			waypoint = get_waypoint(state.wp_index)
		#riferimento di yaw
		rpy_ref[2] = np.arctan2(	waypoint.eta_1[1] - eta_1[1],
						waypoint.eta_1[0] - eta_1[0])
		
		#setto i riferimenti
		references = References()
		references.pos.x = pos_ref[0]
		references.pos.y = pos_ref[1]
		references.pos.z = pos_ref[2]
		references.rpy.x = rpy_ref[0]  #roll non verra' usato 
		references.rpy.y = rpy_ref[1]
		references.rpy.z = rpy_ref[2]
		print("YAW REFERENCE: %s" % int(round(math.degrees(rpy_ref[2]))))
		pub.publish(references)
	elif state.task != 'YAW':  #se non sta al task YAW non viene pubblicato niente e setta le variabili a None nuovamente
		[waypoint, eta_1_init, eta_2_init] = clear_vars([waypoint, eta_1_init, eta_2_init])

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
