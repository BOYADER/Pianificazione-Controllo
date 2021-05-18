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
		critical_pitch = rospy.get_param('/critical_pitch')
		control_radius = rospy.get_param('/control_radius')
		tolerance_degrees = rospy.get_param('/tolerance_degrees')
		tolerance_meters = rospy.get_param('/tolerance_meters')
		auv = AUV(odom.lla.x, odom.lla.y, odom.lla.z,
							odom.rpy.x, odom.rpy.y, odom.rpy.z,
							odom.lin_vel.x, odom.lin_vel.y, odom.lin_vel.z,
							critical_pitch, control_radius, tolerance_degrees, tolerance_meters)
		auv.init_waypoints()
		pitch_des = auv.pitch_des()
		print("pitch_des: %s" % str(pitch_des))
		if abs(pitch_des) < auv.critical_pitch or abs(pitch_des) > (180 - auv.critical_pitch):
			auv.strategy = 1
		else:
			auv.strategy = 2
	else:
		auv.update(	odom.lla.x, odom.lla.y, odom.lla.z,
							 	odom.rpy.x, odom.rpy.y, odom.rpy.z,
								odom.lin_vel.x, odom.lin_vel.y, odom.lin_vel.z)
		task_error = auv.task_error(references)
		print("%s error: %s" % (auv.task_seq[auv.task_index], str(abs(task_error))))
		if auv.task_index <= 2:
			tol = auv.tolerance_degrees
		else:
			tol = auv.tolerance_meters
		if abs(task_error) < tol:
			if auv.task_index == (len(auv.task_seq) - 1):	# waypoint approached
				auv.wp_index = auv.wp_index + 1							# next waypoint, TODO: check waypoints list end
				pitch_des = auv.pitch_des()
				if  abs(pitch_des) < auv.critical_pitch or abs(pitch_des) > (180 - auv.critical_pitch):
					auv.strategy = 1
				else:
					auv.strategy = 2
			else:																					# task completed
				auv.task_index = auv.task_index + 1					# next task
	state = State()
	state.strategy = auv.strategy
	string_param = '/task_seq_list/ts' + str(auv.strategy)
	auv.task_seq = rospy.get_param(string_param)
	state.task = auv.task_seq[auv.task_index]
	print(state)
	pub.publish(state)		

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

	
