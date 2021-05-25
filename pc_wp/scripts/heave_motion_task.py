#!/usr/bin/env python
import rospy
import math
import numpy as np
import pymap3d as pm
from classes.Waypoint import Waypoint
from pc_wp.msg import References, State, Odom
from utils import get_waypoint, clear_vars

#costanti
QUEUE_SIZE = rospy.get_param('/QUEUE_SIZE')

#variabili globali
next_waypoint = None
eta_1_init = None  #posizione all'inizio dell'heave task
eta_2_init = None
eta_1 = []
eta_2 = []


#funzione che appena arriva messaggio odom, setta in eta_1 la posizione attuale e in eta_2 l'orientazione attuale, poi se eta_1_init e eta_2_init non sono ancora state inizializzate le inizializza
def odom_callback(odom): 
	global eta_1, eta_2, eta_1_init, eta_2_init
	lld_ned = rospy.get_param('ned_origin')
	eta_1 = [	pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[0], 
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[1],
			pm.geodetic2ned(odom.lld.x, odom.lld.y, -odom.lld.z, lld_ned['latitude'], lld_ned['longitude'], -lld_ned['depth'])[2]]
	eta_2 = [	odom.rpy.x,
			odom.rpy.y,
			odom.rpy.z]
	if not eta_1_init:	#una volta arrivato il primo messaggio odom inizializzo queste due in quella posizione e orientazione, poi non le tocco più
		eta_1_init = eta_1
	if not eta_2_init:
		eta_2_init = eta_2



def state_callback(state, pub):
	global next_waypoint, eta_1_init, eta_2_init
	if state.task == 'HEAVE':  #se sta a me	
		if not next_waypoint and state.wp_index < len(rospy.get_param('/waypoint_list')): #se non ho ancora settato il prossimo wp e ho il prossimo wp (ovvero se non sono nell'ultimo wp)
			next_waypoint = get_waypoint(state.wp_index+1) #lo setto
		else: #se invece ho già raggiunto l'ultimo wp allora il prossimo wp da raggiungere sarà la posizione che avevo a inizio missione, ovvero origine della terna ned
			next_waypoint = np.array([0 0 0]) #origine terna ned

		x_ref = eta_1_init[0] #il riferimento x è la x che avevo quando è iniziato l'heave task
		y_ref = eta_1_init[1] #idem per la y
		z_ref = next_waypoint[2] #il riferimento z è quello del waypoint successivo (quello da raggiungere)
		roll_ref = eta_2_init[0] #il riferimento roll è il roll che avevo quando è iniziato l'heave task
		pitch_ref = eta_2_init[1] #idem per il pitch
		yaw_ref = eta_2_init[2] #idem per lo yaw
		
		references = References()             #setto i riferimenti
		references.pos.x = x_ref
		references.pos.y = y_ref
		references.pos.z = z_ref
		references.rpy.x = roll_ref
		references.rpy.y = pitch_ref
		references.rpy.z = yaw_ref
		pub.publish(references)			#pubblico i riferimenti
	elif state.task != 'HEAVE': #se non sta più a me risetto a None le variabili, cosi la prossima volta che questo task viene chiamato (new wp) ricomincia a lavorare da zero e non pubblico niente
		[next_waypoint, eta_1_init, eta_2_init] = clear_vars([next_waypoint, eta_1_init, eta_2_init])






def surge_motion_task():
	rospy.init_node('heave_motion_task')
	pub = rospy.Publisher('references', References, queue_size = QUEUE_SIZE)
	rospy.Subscriber('state', State, state_callback, pub)
	rospy.Subscriber('odom', Odom, odom_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		heave_motion_task()
	except rospy.ROSInterruptException:
		pass
