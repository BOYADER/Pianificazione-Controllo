#!/usr/bin/env python
import rospy
from pc_wp.msg import References, Task

def task_callback(task):
	if task.value == "ROTATE":
		rospy.loginfo(task.value)
	#pub = rospy.Publisher('references', References, queue_size = 10)
	#pub.publish(data)
	

def orientation_task():
	rospy.init_node('orientation_task')
	rospy.Subscriber('current_task', Task, task_callback)
	#rospy.Subscriber('odom', Odom, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		orientation_task()
	except rospy.ROSInterruptException:
		pass
