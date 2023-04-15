import rospy
from geometry_msgs.msg import Vector3
import json
import numpy

f = open('data.json')
data = json.load(f)

pub=[]
for i in range(12):
	pub.append(rospy.Publisher('D'+str(i+1), Vector3, queue_size=10))
	
rospy.init_node('base', anonymous=True)
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
	a=Vector3()
	x=[data['drone'+str(i+1)]['x'] for i in range(12)]
	y=[data['drone'+str(i+1)]['y'] for i in range(12)]
	z=[data['drone'+str(i+1)]['z'] for i in range(12)]
	coords=zip(x,y,z)
	print(coords)
	coords = coords[coords[:, 1].argsort()]
	print(coords)
	for i in range(12):
		a.x = coords[i][0]
		a.y = coords[i][1]
		a.z = coords[i][2]
		pub['D'+str(i+1)].publish(a)
	
	rate.sleep()

