#!/usr/bin/env python
import rospy
import nav_msgs
from math import pi as phi
from geometry_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

#This file only handles the map-related data. Specificallly it is meant to control where would waypoints be considered.
#It should only return waypoints and possibly a path through those points.

class Mapper:
    def __init__(self,path):
        self.bestPath = Path()
        #self.bestPath.header = self.createHeader(0,frame)
        bestPath = path

    def createHeader(self,seq,frame):
        h = Header()
        h.seq = seq
        h.stamp = rospy.Time.now()
        h.frame_id = frame
        return h

    def checkforBestPath(self,path):
        myNewbestPath = Path()
        myPathArray = self.bestPath.poses
        for pose in myPathArray:
            for point in path.pointsWithPheromone:


                if (pose.position.x < point.point.x & pose.p point.distance ):
                    ptX = (pose.position.x + point.point.x)/2
                if (pose.position.y < point.point.y):
                    ptY = (pose.position.x + point.point.x)/2












    # def computePathLines(self):
    #     for point in managedPath.pose:
    #         vector = point
    #
    # def locatePlains(self):
    #     i = 1
    #     # Use scipy to figure out a way of locating points with the same category in them



#We'll figure out a way to export empty waypoints  and return them out to where they're needed
#self.managedPath.seq = pathArray.seq








#
#
# def measure_density(map_mymap, int_width, int_height, int_locRange,int_x, int_y):
#     for i in range (0, width):
#         for j in range (0, height):
#             point = (i+x) + width*(j+y)
#            # density = mymap.data.point
#
#
# #We're going to have to construct a map to virtualize the pheromone-enabled map. This map will be a copy over from whatever Hector/Gmapping is going to provide so as to enable consistent processing of the data.
# def locate_plains(map_mymap, int_width, int_height, int_locRange):
#     areaDensity = (locRange**2) * phi #the area to locate around a point
#     for i in range (0, width+height):
#         for j in range (0, height):
#             #The first loops set the points to locate point
#             point = i + width*j
#             density = mymap.data.point
#             self.measure_density(mymap, width, height, locRange, i, j)
#
#
