#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
#obstale sign
flag = 0
flag1 = 0
txtmap = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       	   [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
           [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
           [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
           [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
           [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
           [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
           [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
           [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
           [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
           [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
           [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]

#a star path
search_path = []
iter = 0
#velocity publisher
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)


#get robot stat and make it follow the path
def f_path(data):
	global currentPos, search_path, iter, flag, flag1, velocity_publisher
	rot = data.pose.pose.orientation
	robot = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
	currentPos = [data.pose.pose.position.x, data.pose.pose.position.y]
	vel = Twist()
	targe_x = rospy.get_param("targe_x")
	targe_y = rospy.get_param("targe_y")
	if abs(currentPos[0] - targe_x) + abs(currentPos[1] - targe_y) < 1:
		print("Techinically saying we are at the point but what do I know")
	elif iter < len(search_path):
		if iter !=len(search_path)-1:
			distance_thod=1
		else:
			distance_thod=0.2
		if abs(currentPos[0] - search_path[iter][0])+abs(currentPos[1] - search_path[iter][1])> distance_thod:
			if flag ==1:
				vel = Twist()
				while flag1 ==1:
					#print("Got stuff in our way now we turn right")
					vel.angular.z = -2
					velocity_publisher.publish(vel)
				f_time = time.time()
				while time.time() - f_time < 1:
					#print("Now we go straigh")
					vel.linear.x = 3
					vel.angular.z = 0.0
					velocity_publisher.publish(vel)
				iter =iter+1
				print "changing points:",search_path[iter]
			else:
				theta = math.atan2(search_path[iter][1] - currentPos[1], search_path[iter][0] - currentPos[0])
				if abs(theta - robot[2]) > 1:
					vel.linear.x = 0.0
					vel.angular.z = theta - robot[2]
				else:
					vel.linear.x = 3
					vel.angular.z = 0.0
		else:
			print "changing points:",search_path[iter]
			iter += 1
		velocity_publisher.publish(vel)


def laserscan(data):
	global flag, flag1
	ranges = data.ranges
	min=1000000
	#check to turn left or right
		
	for i in range(180,250):
		if ranges[i] <min:
			min=ranges[i]
	if min<1:
		flag =1
	if min<2:
		flag1 =1
	else:
		flag = 0
		flag1 = 0






# function to get the goal locations from the arguments and initiate A* algorithm
def init_astar():
	global search_path
	global txtmap
	startPos = [-8.0, -2.0]
	rospy.Subscriber('base_pose_ground_truth', Odometry, f_path)
	rospy.Subscriber('base_scan', LaserScan, laserscan)
	targe_x = rospy.get_param("targe_x")
	targe_y = rospy.get_param("targe_y")
	#set starting point at top left
	initpoit = [int(10 - startPos[1]), int(startPos[0] + 9)]
	print(txtmap[12][1])
	print(initpoit)
	#set target as int type and set coodranite 
	target = [int(x) for x in [round(targe_x), round(targe_y)]]
	target = [int(10 - target[1]), int(target[0] + 8)]
	print(target)
	print(txtmap[target[0]][target[1]])
	#a star search path
	path, _ = a_star(txtmap, initpoit, target)
	#change target map to path so the robot can follow
	printUsetxtMap = txtmap
	#print(printUsetxtMap)
	save_path = [[sub_path[1], sub_path[0]] for sub_path in path]
	#save_path.append([targe_x, targe_y])
	print(save_path)

	search_path = [[sub_path[1] - 8.5, 9.5 - sub_path[0]] for sub_path in path]
	search_path.append([targe_x, targe_y])
	for path in range(len(save_path)):
		changepoint = search_path[path]	
		adjustPoint = save_path[path]	
		#print(changepoint)
		#print(changepoint[0])
		#print(changepoint[1])		
		printUsetxtMap[int(adjustPoint[0])][int(adjustPoint[1])] = '#'
	for row in range(len(printUsetxtMap)):
		newrow = str(printUsetxtMap[row]).replace("'","")
		print(newrow)
	#print(printUsetxtMap, sep =', ')
	rospy.spin()


def a_star(txtmap, begin_point, target_point):
	theBase=[]
	theClose=[]
	ToGo=[]
	alldata=[]
	flag = [0,0] 
	direction=[]
	for i in range(4):
		if  i ==0:
			direction.append([-1, 0])
		if  i ==1:
			direction.append([0,-1])
		if  i ==2:
			direction.append([1, 0])
		if  i ==3:
			direction.append([0,1])
	for row in range(len(txtmap)):
		for col in range(len(txtmap[0])):
			tempt=abs(row - target_point[0]) + abs(col - target_point[1])
			alldata.append(tempt)
			if txtmap[row][col] == 1:
				alldata[col] = 1000000  
		theBase.append(alldata)
		alldata = []
	alldata = []
	for row in range(len(txtmap)):
		for col in range(len(txtmap[0])):
			if row==begin_point[0] and col ==begin_point[1]:
				alldata.append(1)
			else:
				alldata.append(0)
		theClose.append(alldata)
		alldata = []
	alldata = []
	for row in range(len(txtmap)):
		for col in range(len(txtmap[0])):
			alldata.append(0)
		ToGo.append(alldata)
		alldata = []
	x=begin_point
	y=[0,theBase[x[0]][x[0]]]
	stack = [[y[1], y[0], x[0], x[1]]]
	while True:
		if x == target_point:
			flag[0] = 1
		if flag ==[0,0]:
			if len(stack) == 0:
				return None, None
			else:
				stack.sort()
				stack.reverse()
				tempt = stack.pop()
				x=tempt[2:4]
				y=tempt[0:2]
				y.reverse()

				for i in range(len(direction)):  
					x2=[x[0] + direction[i][0],x[1] + direction[i][1]]
					if x2[0] >= 0 and x2[0] < len(txtmap) and x2[1] >= 0 and x2[1] < len(txtmap[0]) and theClose[x2[0]][x2[1]] == 0 and txtmap[x2[0]][x2[1]] == 0:
						y2=[y[0] + 1,y[0] + 1 +theBase[x2[0]][x2[1]]]
						stack.append([y2[1], y2[0], x2[0], x2[1]])
						theClose[x2[0]][x2[1]] = 1
						ToGo[x2[0]][x2[1]] = i
		else:
			break
	allpath = []
	x=target_point
	allpath.append(x)
	while True:
		if x[0] != begin_point[0] or x[1] != begin_point[1]:
			x2 = [x[0] - direction[ToGo[x[0]][x[1]]][0], x[1] - direction[ToGo[x[0]][x[1]]][1]]
			x = x2
			allpath.append(x)
		else:
			break
	allpath.reverse()
	return allpath, ToGo


if __name__ == '__main__':
	rospy.init_node('astar', anonymous=True)
	init_astar()

