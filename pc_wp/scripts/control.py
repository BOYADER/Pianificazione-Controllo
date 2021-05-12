#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


def callback(data):
	rospy.loginfo(data.data)
	pub = rospy.Publisher('tau', Float64, queue_size = 10)
	pub.publish(data.data + 0.5)

def control():
	rospy.Subscriber('des_position', Float64, callback)
	rospy.Subscriber('des_orientation', Float64, callback)
	rospy.Subscriber('des_velocity', Float64, callback)
	rospy.Subscriber('current_orientation', Float64, callback)
	rospy.Subscriber('current_position', Float64, callback)
	rospy.Subscriber('current_velocity', Float64, callback)
	rospy.init_node('control')
	rospy.spin()

if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass
