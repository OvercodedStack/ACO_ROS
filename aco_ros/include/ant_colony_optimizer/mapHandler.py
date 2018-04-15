#!/usr/bin/env python
import rospy
import nav_msgs
from ..PathACO import *
from math import pi as phi
from geometry_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
# This file only handles the map-related data. Specificallly it is meant to control where would waypoints be considered.
# It should only return waypoints and possibly a path through those points.

class Mapper:
    def __init__(self, path,frame): #Keep in mind we are expecting a PathACO data type as we cannot create a ROS path out of thin air.
        self.bestPath = Path()
        myHead = self.createHeader(0,frame)
        self.bestPath.header = myHead
        self.bestPath.poses = self.createFakePose(path,frame)


    def createHeader(self,seq,frame):
        h = Header()
        h.seq = seq
        h.stamp = rospy.Time.now()
        h.frame_id = frame
        return h

    def createFakePose(self,path,frame):
        poses = []
        tempPose = PoseStamped() #A temporary poseStampped message to write into before pasting into the poses array.
        count = 0
        h = self.createHeader(count,frame)
        for item in path.pointsWithPheromone:
            insidePose = Pose()
            insidePose.position = item.getPoint
            insidePose.orientation = item.getOrientation

            tempPose.header = h
            tempPose.pose = insidePose
            poses.append(tempPose)
        return poses

    def checkforBestPath(self, path):
        myNewbestPath = Path()
        tempH = myNewbestPath
        myPathArray = self.bestPath.poses

        skip = True
        prevPoint = Point()
        counter = 0
        if (bestDistance < path.getTotalDist):
            for pose in myPathArray:
                if (skip):
                    skip = false  # Skip the first point of the best array.
                else:  # For each point in the contested path, check each point.
                    point = path.pointsWithPheromone[counter]
                    
                    if (pose.position.x < point.point.x & bestPathDist < point.distance):  # Check if it's slightly better and the total distance is shorter
                        ptX = (pose.position.x + point.point.x) / 2
                        ptY = (pose.position.y + point.point.y) / 2
                if (not ptX):

                counter += 1
                prevPoint = pose  # Save the previous pose from the best array







                # for point in range( 1, len(path.pointsWithPheromone)-1):
                # if (pose.position.y < point.point.y & bestPathDist < point.distance):






                # def computePathLines(self):
                #     for point in managedPath.pose:
                #         vector = point
                #
                # def locatePlains(self):
                #     i = 1
                #     # Use scipy to figure out a way of locating points with the same category in them

# We'll figure out a way to export empty waypoints  and return them out to where they're needed
# self.managedPath.seq = pathArray.seq








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
