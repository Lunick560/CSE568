#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from visualization_msgs.msg import Marker
import random
ranges = np.zeros((361, 2))
#publish the marker topic which will be shown in rviz
publisher = rospy.Publisher("visualization_marker", Marker, queue_size=10)


def publishRansacMarker(inner_point_x, inner_point_y):
    #Define the ransac marker in rviz so it will publish line
    #Give id to marker. Since here its only one marker so it should be fine to set it to 0 I guess
    marker = Marker()
    marker.id = 0
    marker.ns = "Ransac"
    marker.header.frame_id = "/base_link"
    marker.header.stamp = rospy.Time.now()
    
   
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    #Store data into marker
    for i in range(1, len(inner_point_x)):
        maker_point = Point()
        maker_point.x = inner_point_x[i]
        maker_point.y = -inner_point_y[i]
        marker.points.append(maker_point)

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    ##Scale is different from marker type. Which POINTS maker use x and y for width and height but LINE_STRIP and LINE_LIST maker only use x for line width in terms of meters.
    marker.scale.x = 0.1
    marker.scale.y = 0.0
    marker.scale.z = 0.0
    #Blue line
    marker.color.b = 1.0
    marker.color.a = 1.0 
    #Took me so long to realize the line lifetime
    marker.lifetime = rospy.Duration()
    publisher.publish(marker)

#process the laser scan data
def get_lidar_data(data):
    global ranges
    lidar_range = list(data.ranges)

    #Since the laser scan data is 180 degree which we set the limit between -90 degrees to 90 degrees
    #Turn laser scan data into numpy array
    #The range equal to 3 means within 3 meters, all the points without a obstacle will be set to 0.
    #Transfer the angle and distance of the laser scan into x y coordinate points
    theta = 1.57 #half of pi
    for i in range(361):
        if lidar_range[i] == 3:
            lidar_range[i] = 0
        theta = theta - 0.0087266
        ranges[i, 0] = lidar_range[i] * np.cos(theta)
        ranges[i, 1] = lidar_range[i] * np.sin(theta)


#The Ransac part
#iterations is the iteration times, distance_threshold is... well, threshold.   point_threshold are the points within the threshold.
def Ransac(iterations, distance_threshold, point_threshold):
    point_x = ranges[:, 0]
    point_y = ranges[:, 1]
    #p_sub is point subscribe, which are the number of the sensor data, which is the number of 360
    p_sub = range(len(point_x))
    rest_threshold = len(point_x) /100
    inner_point_x = [-1000]
    inner_point_y = [-1000]
    inner_point = []
    out_point = []
    #This loop is to find no more than 5 egdes
    for r in range(5):
        max = 0
        for i in range(iterations):
            count_i = 0
            count_o = 0
            #Generates two random points between 0 and 360
            ran1 = random.randint(0, len(p_sub) - 1)
            ran2 = random.randint(0, len(p_sub) - 1)
            #Define the list for inliers and outliers.
            tempt_point = np.zeros((361, 2), "int")
	    #If they are equal then we not consider it. We want two random points.
            if ran1 != ran2:
                p_ran1 = [point_x[p_sub[ran1]], point_y[p_sub[ran1]]]
                p_ran2 = [point_x[p_sub[ran2]], point_y[p_sub[ran2]]]
                #Now we iterates through two other random points. And we keep going until we hit all the points
                for j in (p_sub):
                    p_ran_x = point_x[j]
                    p_ran_y = point_y[j]
                    if p_ran_x != 0 and p_ran_y != 0 and (p_ran2[0] - p_ran1[0]) != 0:
                        distance = abs((p_ran2[1] - p_ran1[1]) * p_ran_x - (p_ran2[0] - p_ran1[0]) * p_ran_y +
                                   p_ran2[0] * p_ran1[1] - p_ran2[1] * p_ran1[0]) / math.sqrt(
                                       (p_ran2[1] - p_ran1[1]) * (p_ran2[1] - p_ran1[1]) +
                                       (p_ran2[0] - p_ran1[0]) * (p_ran2[0] - p_ran1[0]))
                        if distance < distance_threshold:
                            #Add the inliers
                            tempt_point[count_i, 0] = j
                            count_i = count_i + 1
                        elif distance >= distance_threshold:
                            #Add the outliers
                            tempt_point[count_o, 1] = j
                            count_o = count_o + 1
            #If there are more inliers than outliers we update
            if max < count_i:
                max = count_i
                inner_point = []
                out_point = []
                for i in range(count_i):
                    inner_point.append(int(tempt_point[i, 0]))
                for i in range(count_o):
                    out_point.append(int(tempt_point[i, 1]))
                    #Well here will return two coordinate points, which is the biggest and the smallest out of all the points. So that we might include the whole inliers area 
                max_x = ranges[inner_point[0], 0]
                min_x = ranges[inner_point[len(inner_point) - 1], 0]
                max_y = ranges[inner_point[0], 1]
                min_y = ranges[inner_point[len(inner_point) - 1], 1]

        #Keep this two points. The number of inliers should not less than point_threshold, which is the number of points within the threshold.
        if len(inner_point) > point_threshold:
            inner_point_x.append(max_x)
            inner_point_y.append(max_y)
            inner_point_x.append(min_x)
            inner_point_y.append(min_y)
            #Here we update the original p_sub value, and we get ride of the inliers that we already confir
            #print(len(inner_point))
            p_sub = out_point
        #We repeat that process until the outliers is less than one threshold.
        if len(p_sub) <= rest_threshold:
            break
    publishRansacMarker(inner_point_x, inner_point_y)


if __name__ == '__main__':
    try:
        rospy.init_node('Ransac', anonymous=True)
        #subscribe the laser scan data
        rospy.Subscriber("/robot_0/base_scan", LaserScan, get_lidar_data)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            #Ransac
            Ransac(200, 0.1, 3)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
