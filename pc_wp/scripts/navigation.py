#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def navigation():
	pub = rospy.Publisher('odom', Float64, queue_size=10)
	rospy.init_node('navigation')
	rate = rospy.Rate(1) # 1hz
	num = 1
	while not rospy.is_shutdown():
		rospy.loginfo(num)
		pub.publish(num)
		num = num + 1
		rate.sleep()

if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
