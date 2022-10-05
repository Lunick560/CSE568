#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
robot_pos = 0
robot_orien = 0
#Use to check if there are wall on the robot left or right hand side
#flag is left and flag1 is right
flag = 0
flag1 = 0
#Check if robot on the line
robot_on_line = False


def judge_wall(data):
    global flag, flag1
    ranges = data.ranges
    flag = 0
    flag1 = 0
    #Check if there are wall on the robot left hand side or right hand side
    #Here is the thing. Left and front are from 0 to 265. right hand side is from 265 to 360
    for i in range(265):
        if ranges[i] < 1:
            flag = 1
            break
    for i in range(265, 360):
        if ranges[i] < 1:
            flag1 = 1
            break


def get_robot_data(data):
    global robot_orien
    global robot_pos, robot_on_line
    robot_pos = data.pose.pose.position
    #Check if the point on the line
    robot_orien = data.pose.pose.orientation

    distance = abs((-8 * (9 - robot_pos.y) + 4.5 * (robot_pos.y - -2) + robot_pos.x * (-2 - 9)) / 2.0)
    if (distance <= 0.5):
        robot_on_line = True
    else:
        robot_on_line = False


def Bug2(goal_angle, change_flag):
    global flag, flag1
    twist = Twist()
    vel = 0.0
    #When hit the wall set the speed to 0
    if (flag == 1 and flag1 == 1):
        vel = 0.0
    else:
        vel = 3
    twist.linear.x = vel
    #If robot on the line or there are no obstacle in front then its goal seeking, otherwise line follow
    if change_flag == 0:
        print("Now we goal seek")
        twist.angular.z = min(goal_angle, 2)
        if (flag == 1 or flag1 == 1):
            twist.angular.z = -2
            twist.linear.x = 0
        if (not robot_on_line and flag == 0 and flag1 == 1):
            change_flag = 1
    else:
	if (flag == 0 and flag1 == 0):
            print("Turn left as deafult")
            twist.angular.z = 2
	if (flag == 0 and flag1 == 1):
            print("we got a wall here now we go straight boyss")
            twist.angular.z = 0
        #if goal angle less than 0 means robot is now currently at the opposite of the obsctale
        if (goal_angle < 0 and flag1 == 1 and robot_on_line):
            print("We got on the target line! Now its time to goal seek!")
            change_flag = 0
        if (flag == 1 and flag1 == 1):
            print("We got a wall here nowe we turn right boyss")
            twist.angular.z = -2
            twist.linear.x = 0
        
         
        
    return twist, change_flag


if __name__ == '__main__':
    rospy.init_node('bug2', anonymous=True)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, judge_wall)
    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry,get_robot_data)
    pub_cmd_vel = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    change_flag = 0
    while True:
        # print pos, orien
        if robot_orien != 0:
            #Calculating the face direction of the robot
            robot_angle = 2 * np.arcsin(robot_orien.z)
            #Calculating the distance between the goal area and robot
            robotgoal = abs(4.5 - robot_pos.x) + abs(9 - robot_pos.y)
 	    #Calculating angle between robot and target area
            goal_angle = math.atan((9 - robot_pos.y) / (4.5 - robot_pos.x)) - robot_angle
            twist = Twist()
            #Check if robot at the target spot
            if robotgoal <=0.5:
                twist.linear.x = 0
                twist.angular.z = 0
                break
            else:
                twist, change_flag = Bug2(goal_angle, change_flag)
            pub_cmd_vel.publish(twist)
            rate.sleep()
