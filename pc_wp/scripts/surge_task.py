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

distance_tolerance = rospy.get_param('references/SURGE/distance_tolerance')
surge_reference_high = rospy.get_param('references/SURGE/high')
surge_reference_low = rospy.get_param('references/SURGE/low')

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def odom_callback(odom):									# when node is active: eta_1, eta_2 update - setting of eta_1_init, eta_2_init
	global task, eta_1, eta_2, eta_1_init, eta_2_init
	if task == 'SURGE':
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
	global task, eta_1, eta_2, eta_1_init, eta_2_init, distance_tolerance, surge_reference_high, surge_reference_low
	if state.task == 'SURGE':								# this node is active
		task = state.task
		while isNone([eta_1, eta_2, eta_1_init, eta_2_init]):				# wait until eta_1, eta_2, eta_1_init, eta_2_int are initialized
			pass
		waypoint = get_waypoint(state.wp_index)						# get next waypoint
		if waypoint is not None:
			references = References()						# prepare msg References to control and task_manager nodes
			references.pos.x = waypoint.eta_1[0]					# set final eta_1.x reference
			references.pos.y = waypoint.eta_1[1]					# set final eta_1.y reference
			references.pos.z = waypoint.eta_1[2]					# set final eta_1.z reference
			references.rpy.x = eta_2_init[0]					# useless, roll not actuated
			references.rpy.y = eta_2_init[1]					# maintain the same pitch as it was at the start of the task 
			references.rpy.z = eta_2_init[2]					# maintain the same yaw as it was at the start of the task 
			distance = math.sqrt(	(references.pos.x - eta_1[0])**2 + 		# compute actual distance from waypoint
						(references.pos.y - eta_1[1])**2 + 
						(references.pos.z - eta_1[2])**2)		
			if distance <= distance_tolerance:					# set low reference surge velocity, approaching to waypoint
				references.lin_vel.x = surge_reference_low
			else:
				references.lin_vel.x = surge_reference_high			# set high reference surge velocity
			pub.publish(references)							# msg published to topic
	elif state.task != 'SURGE':								# this node is inactive
		task = state.task
		if not isNone([eta_1_init, eta_2_init]):
			[eta_1_init, eta_2_init] = clear([eta_1_init, eta_2_init])		# clear init variables
def surge_task():
	rospy.init_node('surge_task')
	pub = rospy.Publisher('references', References, queue_size = QUEUE_SIZE)
	rospy.Subscriber('state', State, state_callback, pub)
	rospy.Subscriber('odom', Odom, odom_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		surge_task()
	except rospy.ROSInterruptException:
		pass
