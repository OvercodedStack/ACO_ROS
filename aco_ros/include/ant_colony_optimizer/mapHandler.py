#!/usr/bin/env python
import rospy
import ros
import tf
import nav_msgs
#import scipy
from math import pi as phi
from geometry_msgs.msg import Path
#This file only handles the map-related data. Specificallly it is meant to control where would waypoints be considered.
#It should only return waypoints and possibly a path through those points.

class Mapper:
    self.storedPaths[]
    def __init__(self,pathArray):
        managedPath = Path()
        self.bestPath = Path()
        #self.managedPath.seq = pathArray.seq
        managedPath.pose = pathArray.pose

    def locatePlains(self):
        #Use scipy to figure out a way of locating points with the same category in them

    def locateBestPath(self):


        self.bestPath =

    def returnPlaints(self):
        return


    computePathLines(self):
        for point in managedPath.pose:
            vector = point





#We'll figure out a way to export empty waypoints  and return them out to where they're needed










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
