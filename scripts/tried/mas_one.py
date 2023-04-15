import rospy
from geometry_msgs.msg import Vector3, Pose
from gazebo_msgs.msg import ModelStates
import json
import numpy


rospy.init_node('base', anonymous=True)
# rospy.Subscriber("/gazebo/model_states", ModelStates, _set_odom)
pub = rospy.Publisher('D1', Vector3, queue_size=10) 

def init_line():
	# initial height = 5
	v=Vector3()
	v.x = 3
	v.y = 6
	v.z = 5
	rate = rospy.Rate(10)
	while True:
		pub.publish(v)
		rospy.loginfo("command sent")
		rate.sleep()
	

init_line()
rospy.spin()

