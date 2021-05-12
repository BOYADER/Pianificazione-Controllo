#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def callback(data):
	rospy.loginfo(data.data)
	pub1 = rospy.Publisher('des_position', Float64, queue_size = 10)
	pub2 = rospy.Publisher('des_orientation', Float64, queue_size = 10)
	pub1.publish(data.data + 0.5)
	pub2.publish(data.data + 0.5)

def heave_motion_task():
	rospy.init_node('heave_motion_task')
	rospy.Subscriber('current_task', Float64, callback)
	rospy.Subscriber('current_position', Float64, callback)
	rospy.spin()

if __name__ == '__main__':
	heave_motion_task()
