#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


def callback(data):
	rospy.loginfo(data.data)
	
def task_detector():
	rospy.Subscriber('current_position', Float64, callback)
	pub = rospy.Publisher('current_task', Float64, queue_size=10)
	rospy.init_node('task_detector')
	rate = rospy.Rate(1) # 1hz
	num = 1
	while not rospy.is_shutdown():
		rospy.loginfo(num)
		pub.publish(num)
		num = num + 1
		rate.sleep()

if __name__ == '__main__':
	try:
		task_detector()
	except rospy.ROSInterruptException:
		pass

