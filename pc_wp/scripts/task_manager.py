#!/usr/bin/env python
import rospy
import math
from classes.AUV import AUV
from classes.Waypoint import Waypoint
from pc_wp.msg import Odom, References, State
	
auv = None
references = References()

def odom_callback(odom, pub):
	global auv, references
	if not auv:
		auv = AUV(	odom.lla.x, odom.lla.y, odom.lla.z,
				odom.rpy.x, odom.rpy.y, odom.rpy.z,
				odom.lin_vel.x, odom.lin_vel.y, odom.lin_vel.z)
		auv.init_waypoints()
		pitch_des = auv.pitch_desired()
		auv.set_strategy(pitch_des)
		print("pitch_des: %s" % str(pitch_des))
		auv.set_strategy(pitch_des)											# set strategy and tasks_sequence
		auv.set_tolerance()															# set task tolerance error 
	else:
		auv.update(	odom.lla.x, odom.lla.y, odom.lla.z,	# update eta_1, eta_2, ni_1
				odom.rpy.x, odom.rpy.y, odom.rpy.z,
				odom.lin_vel.x, odom.lin_vel.y, odom.lin_vel.z)
		task_error = auv.task_error(references)					# task_error computation
		print("%s error: %s" % (auv.task_seq[auv.task_index], str(abs(task_error))))		
		if abs(task_error) < auv.tolerance:							# check if current task is completed or not
			if auv.task_index == (len(auv.task_seq) - 1):	# waypoint approached
				auv.wp_index = auv.wp_index + 1							# next waypoint, TODO: check waypoints list
				pitch_des = auv.pitch_desired()							# compute pitch_des to decide the strategy
				auv.set_strategy(pitch_des)									# strategy determined, task_seq initialized
			else:																					# task completed, waypoint not yet approached
				auv.task_index = auv.task_index + 1					# next task
				auv.set_tolerance()													# update task tolerance
	state = State()																		# prepare msg State to task_nodes
	state.strategy = auv.strategy
	state.task = auv.task_seq[auv.task_index]
	print(state)
	pub.publish(state)																# msg published to topic

def ref_callback(ref):
	global references
	references = ref
	#TODO

def task_manager():
	pub = rospy.Publisher('current_state', State, queue_size=10)
	rospy.Subscriber('odom', Odom, odom_callback, pub)
	rospy.Subscriber('references', References, ref_callback)
	rospy.init_node('task_manager')
	rospy.spin()

if __name__ == '__main__':
	try:
		task_manager()
	except rospy.ROSInterruptException:
		pass

	
