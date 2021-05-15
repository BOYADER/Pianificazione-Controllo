#!/usr/bin/env python
import rospy
from pc_wp.msg import Odom, References, Task

references = References()
current_task = Task()

def odom_callback(odom, pub):
	global current_task	
	current_task.value = "ROTATE"
	pub.publish(current_task)

def ref_callback(ref):
	global references
	references = data
	
def task_manager():
	pub = rospy.Publisher('current_task', Task, queue_size=10)
	rospy.Subscriber('odom', Odom, odom_callback, pub)
	rospy.Subscriber('references', References, ref_callback)
	rospy.init_node('task_manager')
	rospy.spin()

if __name__ == '__main__':
	try:
		task_manager()
	except rospy.ROSInterruptException:
		pass

