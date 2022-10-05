#!/usr/bin/env python
import rospy
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


Pos = Twist()

#When got the message from suscrb will call back
def Laser_Callback(msg):
    # will print the callback
    # Assume there is nothing in our way	
    IsStuffInTheWay = 0
    # I tried with msg.ranges[0] but that will resulting crash into walls
    # So I assume 360 is the way to go
    for i in range(360):
    	if (msg.ranges[i] < 1):
		#If there is a thing, rotate direction.
		#Well saw it on ROS toturial turtle bot turn left/right
        	Pos.angular.z = 3.14
		#We stop moving forward       	
		Pos.linear.x = 0
		#Set flag to it, break it, enter next loop
        	IsStuffInTheWay = 1
        	break
		#If nothing in our way we move with speed of 2
    	elif (IsStuffInTheWay == 0):
		#And we not turning any direction
        	Pos.angular.z = 0
        	Pos.linear.x = 2

#The reason why I use thread here. In order to put a subscriber and publisher together
#I need to put subscriber in a seperate node
#Subscriber will only call back when reach rospy.spin
#but rospy.spin will stop the following function
#So... thread.
def thread_job():
    # Create a subscriber and register Laser_callback
    # As long as we do not call stop it will loop through the rotation and moving	
    rospy.Subscriber("base_scan", LaserScan, Laser_Callback)
    rospy.spin()


if __name__ == '__main__':
    # ros node initial
    # Create a node for subscrib because that call back have to have rospy.spin	
    rospy.init_node('evader', anonymous=True)
    add_thread = threading.Thread(target=thread_job)
    add_thread.start()
    # create a publisher so it can move-
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    # rate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(Pos)
        rate.sleep()
