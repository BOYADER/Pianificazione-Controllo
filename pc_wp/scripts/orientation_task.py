#!/usr/bin/env python
import rospy
from pc_wp.msg import References, State

def state_callback(state):
	rospy.loginfo(state)
	pub = rospy.Publisher('references', References, queue_size = 10)
	references = References()
	pub.publish(data)
	

def orientation_task():
	rospy.init_node('orientation_task')
	rospy.Subscriber('current_task', State, state_callback)
	#rospy.Subscriber('odom', Odom, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		orientation_task()
	except rospy.ROSInterruptException:
		pass
