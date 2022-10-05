#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import tf
import math

#SO pursuer evader is a bit different.
#Here I initial both evader and pursuer. First of course to use tf module to get their location
#So in launch file I remap the base scan and velocity.
#This part has the reference from ROS tutorial on TF
#When we got the msg from the subscriber we will trying to call back variable
def poseCallback0(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     msg.header.stamp, msg.header.frame_id, "world")


#When we got the msg from the subscriber we will trying to call back variable
def poseCallback1(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     msg.header.stamp, msg.header.frame_id, "world")

#Well- two subscriber. Again we using threads here 
def thread_job():
    rospy.Subscriber("robot_0/base_pose_ground_truth", Odometry, poseCallback0)
    rospy.Subscriber("robot_1/base_pose_ground_truth", Odometry, poseCallback1)
    rospy.spin()


if __name__ == '__main__':
    #Ros node inisital-
    #Create a publisher to publish topic for subscribe because we need to have rospy.spin in the subscrber
    rospy.init_node('pursuer', anonymous=True)
    add_thread = threading.Thread(target=thread_job)
    add_thread.start()
    #a tf listener
    listener = tf.TransformListener()
    #I remap evader cmd_vel to robot 0/cmd_vel in launch file.
    robot_1_vel = rospy.Publisher('robot_1/cmd_vel', Twist, queue_size=10)

    #Set the rate to 10
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('robot_1/base_footprint',
                                                    'robot_0/base_footprint',
                                                    rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            continue
	
	#Depend on the position of robot 0 and robot 1, the linear speed and angular speed needs to be calculated
	#And also we need to send command to make robot 1 move to robot 0.
        Pos = Twist()
        distance = 0
	#This is the distance between evader and the pursuer
        Pos.angular.z = float(5 * math.atan2(trans[1], trans[0]))
	#We dont want exactly the same angle of turning. Smooooooooth
        distance = float(abs(trans[0] + trans[1]))
	#Instead of abs, square then sqrt back.
        Pos.linear.x = 0.5 * distance
        print(Pos.angular.z)
        print(Pos.linear.x)
	#If distance between two robots smaller than 0.5
	#We not moving.
        if (distance < 0.5):
            Pos.linear.x = 0#-Pos.linear.x
	#If position angle greater than max angle, which here is PI, as the evader
	#Set it to 3.14, turning but not moving.
        if (Pos.angular.z > 3.14):
            Pos.angular.z = 3.14
            Pos.linear.x = 0
	#Set the max speed to 2m/s
        if (Pos.linear.x > 2):
            Pos.linear.x = 2
        robot_1_vel.publish(Pos)
        rate.sleep()
