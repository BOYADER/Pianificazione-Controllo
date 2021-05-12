#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def navigation():
	pub1 = rospy.Publisher('current_position', Float64, queue_size=10)
	pub2 = rospy.Publisher('current_orientation', Float64, queue_size=10)
	pub3 = rospy.Publisher('current_velocity', Float64, queue_size=10)
	rospy.init_node('navigation')
	rate = rospy.Rate(1) # 1hz
	num = 1
	while not rospy.is_shutdown():
		rospy.loginfo(num)
		pub1.publish(num)
		pub2.publish(num)
		pub3.publish(num)
		num = num + 1
		rate.sleep()

if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
