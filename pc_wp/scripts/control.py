#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


def callback(data):
	rospy.loginfo(data.data)
	pub = rospy.Publisher('tau', Float64, queue_size = 10)
	pub.publish(data.data + 0.5)

def control():
	rospy.init_node('control')
	rospy.Subscriber('odom', Float64, callback)
	rospy.Subscriber('current_task', Float64, callback)
	rospy.Subscriber('des_velocity', Float64, callback)
	rospy.Subscriber('references', Float64, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass
