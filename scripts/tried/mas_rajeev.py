#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Pose
from gazebo_msgs.msg import ModelStates
import json
import numpy


rospy.init_node('base', anonymous=True)
model_names = [0 for i in range(12)] #string array of models name
model_poses = [0 for i in range(12)] #pose array of type geometry_msgs/Pose
init_pose = [0 for i in range(12)]   #initial position of drones
flag = True
def _set_odom(modelstates):
	# print(type(modelstates), type(modelstates.name), type(modelstates.pose))
	global model_names, model_poses, flag
	if flag: 
		for i in range(12):
			init_pose[i] = modelstates.pose[i]
			flag = False
		for i in range(12):
			model_names[i] = modelstates.name[i]
	for i in range(12):
		model_poses[i] = modelstates.pose[i]
	# _get_odom()

rospy.Subscriber("/gazebo/model_states", ModelStates, _set_odom)
pub_arr = [rospy.Publisher(f"D{i+1}", Vector3, queue_size=10) for i in range(12)]

def _get_odom():
	for i in range(12):
		print("----------------------------------------------------\n", 
			model_names[i], model_poses[i])

def is_reached(i, goalx, goaly, goalz):
	thres = 0.05
	prate = rospy.Rate(2)
	while True:	
		prate.sleep()
		if  abs(model_poses[i].position.x - init_pose[i].position.x - goalx) <= thres and \
			abs(model_poses[i].position.y - init_pose[i].position.y - goaly) <= thres and \
			abs(model_poses[i].position.z - init_pose[i].position.z - goalz) <= thres:
			print("Got to the point")
			return True
	
def init_line():
	# initial height = 5
	print("Sending command to drones")
	v=Vector3()
	v.x = 3
	v.y = 6
	v.z = 5
	pub_arr[4].publish(v)
	pub_arr[5].publish(v)
	v.y=6
	pub_arr[6].publish(v)
	pub_arr[7].publish(v)
	v.x = 6
	v.y=-12
	pub_arr[8].publish(v)
	pub_arr[9].publish(v)
	v.y=12
	pub_arr[10].publish(v)
	pub_arr[11].publish(v)
	print("Checking their positions!!!")
	# is_reached(4,  3, -6, 5)
	# is_reached(5,  3, -6, 5)
	# is_reached(6,  3,  6, 5)
	# is_reached(7,  3,  6, 5)
	# is_reached(8,  6,-12, 5)
	# is_reached(9,  6,-12, 5)
	# is_reached(10, 6, 12, 5)
	# is_reached(11, 6, 12, 5)
	return True
	
	

init_line()
rospy.spin()

# f = open('data.json')
# data = json.load(f)

# pub=[]
# for i in range(12):
# 	pub.append(rospy.Publisher('D'+str(i+1), Vector3, queue_size=10))
	
# rate = rospy.Rate(10) # 10hz
# while not rospy.is_shutdown():
# 	a=Vector3()
# 	x=[data['drone'+str(i+1)]['x'] for i in range(12)]
# 	y=[data['drone'+str(i+1)]['y'] for i in range(12)]
# 	z=[data['drone'+str(i+1)]['z'] for i in range(12)]
# 	coords=zip(x,y,z)
# 	print(coords)
# 	coords = coords[coords[:, 1].argsort()]
# 	print(coords)
# 	for i in range(12):
# 		a.x = coords[i][0]
# 		a.y=0
# 		a.z=0
# 		pub['D'+str(i+1)].publish(a)
# 	large= coords[:, 0].argsort()[-1]
# 	while True:
# 		if abs(model_poses[large].x- coords[large][0])>0.5:
# 			break
		
# 	for i in range(12):
# 		a.x = coords[i][0]
# 		a.y = coords[i][1]
# 		a.z = 0
# 		pub['D'+str(i+1)].publish(a)
	
# 	rate.sleep()

