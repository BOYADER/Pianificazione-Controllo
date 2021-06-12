#!/usr/bin/env python
import rospy
import math
from classes.AUV import AUV
from classes.Waypoint import Waypoint
from utils import get_waypoint, print_end_mission
from pc_wp.msg import Odom, References, State
	
auv = None
references = None

end_mission = False

sub = None											# subscriber object initialized to None
pub = None											# publisher object initialized to None

QUEUE_SIZE = rospy.get_param('QUEUE_SIZE')

def odom_callback(odom):
	global auv, references, end_mission, sub, pub
	if not auv:
		rospy.set_param('ned_frame_origin', {	'latitude': odom.lld.x, 		# set ned_origin on the frame.yaml file
							'longitude': odom.lld.y,
							'depth': odom.lld.z})
		auv = AUV(	odom.lld.x, odom.lld.y, odom.lld.z,				# create AUV object
				odom.rpy.x, odom.rpy.y, odom.rpy.z,
				odom.lin_vel.x, odom.lin_vel.y, odom.lin_vel.z)
		auv.waypoint = get_waypoint(auv.wp_index)					# set next waypoint
		if not auv.waypoint:
			end_mission = True							# mission completed
		else:		
			pitch_des = auv.pitch_desired()						# compute pitch_des to decide the strategy
			auv.set_strategy(pitch_des)						# set strategy and task_sequence
			auv.set_tolerance()							# set task tolerance error 
	elif not end_mission:
		auv.update(	odom.lld.x, odom.lld.y, odom.lld.z,				# update eta_1, eta_2, ni_1
				odom.rpy.x, odom.rpy.y, odom.rpy.z,
				odom.lin_vel.x, odom.lin_vel.y, odom.lin_vel.z)	
		if references is not None:	
			task_error = auv.task_error(references)					
			if abs(task_error) <= auv.tolerance:					# check if current task is completed
				if auv.task_index == len(auv.task_seq) - 1:			# waypoint approached
					references = None					# references cleaning
					auv.task_index = 0					# task_index reset
					auv.wp_index = auv.wp_index + 1	
					auv.waypoint = get_waypoint(auv.wp_index)		# next waypoint
					if not auv.waypoint:
						end_mission = True				# mission completed
					else:										
						pitch_des = auv.pitch_desired()			# compute pitch_des to decide the strategy
						auv.set_strategy(pitch_des)			# set strategy and task_sequence		
						auv.set_tolerance()				# set task tolerance error
				else:								# task completed, waypoint not yet approached
					references = None					# references cleaning
					auv.task_index = auv.task_index + 1			# next task
					auv.set_tolerance()					# update task tolerance
		state = State()									# prepare msg State to task nodes
		state.strategy = auv.strategy
		state.task = auv.task_seq[auv.task_index]
		state.wp_index = auv.wp_index
		pub.publish(state)								# msg published to topic
	elif end_mission:									# mission completed
		print_end_mission()	
		sub.unregister()								# unregistered from topic						

def ref_callback(ref):
	global references
	references = ref									# references update

def task_manager():
	global QUEUE_SIZE, sub, pub
	rospy.init_node('task_manager')
	pub = rospy.Publisher('state', State, queue_size = QUEUE_SIZE)
	sub = rospy.Subscriber('odom', Odom, odom_callback)
	rospy.Subscriber('references', References, ref_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		task_manager()
	except rospy.ROSInterruptException:
		pass

	
