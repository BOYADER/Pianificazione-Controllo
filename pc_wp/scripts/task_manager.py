#!/usr/bin/env python
import rospy
import math
from classes.AUV import AUV
from classes.Waypoint import Waypoint
from pc_wp.msg import Odom, References, State
	
auv = None

def init_auv():
	global auv
	latitude = rospy.get_param('/initial_pose/position/latitude')
	longitude = rospy.get_param('/initial_pose/position/latitude')
	depth = rospy.get_param('/initial_pose/position/depth')
	roll = rospy.get_param('/initial_pose/orientation/roll')
	pitch = rospy.get_param('/initial_pose/orientation/pitch')
	yaw = rospy.get_param('/initial_pose/orientation/yaw')
	critical_pitch = rospy.get_param('/critical_pitch')
	auv = AUV(latitude, longitude, depth, roll, pitch, yaw, critical_pitch)	

def init_waypoints():
	global auv
	tolerance = rospy.get_param('/tolerance_on_waypoint')
	index = 1
	while index <= len(rospy.get_param('/waypoint_list')):
		string_param = '/waypoint_list/wp' + str(index)
		latitude = rospy.get_param(string_param)['latitude']
		longitude = rospy.get_param(string_param)['longitude']
		depth = rospy.get_param(string_param)['depth']
		auv.waypoints.append(Waypoint(latitude, longitude, depth, tolerance))
		index = index + 1
		
def odom_callback(odom, pub):
	global auv
	if not auv.strategy:
		#calcolare pitch desiderato a partire dalla posizione dell'auv (odom.lla -> x y z) e dalla posizione del prossimo waypoint
		#controllare se pitch desiderato e' minore di critical_pitch, se si auv.strategy = 1 else auv.strategy = 2
		#inizializzare auv.task_seq a seconda della strategia
		#publish messaggio con task corrente		
		auv.ned(odom.lla.x, odom.lla.y, odom.lla.z)
		auv.strategy = 1
		auv.update(odom.lla.x, odom.lla.y, odom.lla.z, odom.rpy.x, odom.rpy.y, odom.rpy.z, odom.lin_vel.x, odom.lin_vel.y, odom.lin_vel.z)
		print(auv.x, auv.y, auv.z)
	else:
		auv.update(odom.lla.x, odom.lla.y, odom.lla.z, odom.rpy.x, odom.rpy.y, odom.rpy.z, odom.lin_vel.x, odom.lin_vel.y, odom.lin_vel.z)
		print(auv.x, auv.y, auv.z)

def ref_callback(ref):
	global references
	references = ref

def task_manager():
	pub = rospy.Publisher('current_state', State, queue_size=10)
	rospy.Subscriber('odom', Odom, odom_callback, pub)
	rospy.Subscriber('references', References, ref_callback)
	rospy.init_node('task_manager')
	rospy.spin()

if __name__ == '__main__':
	init_auv()
	init_waypoints()
	try:
		task_manager()
	except rospy.ROSInterruptException:
		pass

	
