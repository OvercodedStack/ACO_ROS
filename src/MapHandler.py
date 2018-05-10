from PathACO import *
from math import pi as phi
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from copy import copy

# This file only handles the map-related data. Specificallly it is meant to control where would waypoints be considered.
# It should only return waypoints and possibly a path through those points.

class Mapper:
    def __init__(self):  # Keep in mind we are expecting a PathACO data type as we cannot create a ROS path out of thin air.
        self.pathArrays = []
        self.bestPath = PathACO()

    def setBestPath(self, path):
        self.bestPath = path  # Both are PathACO class types

    def checkforBestPath(self, path):
        # Funtion serves to compare between two paths, illeterate between them, and return the best path in PathACO format.
        myPathArray = self.bestPath
        ptX = 0
        PtY = 0
        myNewbestPath = PathACO()
        skip = True
        prevPoint = Point()
        counter = 0
        bestDistance = myPathArray.getTotalDist()
        otherDist = path.getTotalDist()
        if (bestDistance < otherDist):
            for pose in myPathArray.pointsWithPheromone:
                if (skip):
                    skip = False  # Skip the first point of the best array.
                else:  # For each point in the contested path, check each point.
                    point = path.pointsWithPheromone[
                        counter].getPoint()  # The IJ distance from one point to the next already calculated
                    posePoint = pose.getPoint()
                    bestPathDist = math.sqrt((posePoint.x - prevPoint.x) ** 2 + (posePoint.y - prevPoint.y) ** 2)
                    if (bestPathDist > point.distance):  # Compare IJ distance, if the distance between points of the old path is worse, average the distance for the new points.
                        ptX = (pose.position.x + point.point.x) / 2
                        ptY = (pose.position.y + point.point.y) / 2

                if (not ptX):  # Set the origin for the first point on the array
                    time = rospy.Time.now()
                    time = time.to_sec()
                    pt = myPathArray.pointsWithPheromone[0].getPoint()
                    ori = myPathArray.pointsWithPheromone[0].getOrientation()
                    myNewbestPath.insertPoint(pt, 1, time, ori)
                else:
                    time = rospy.Time.now()
                    time = time.to_sec()
                    pt = Point()
                    pt.x = ptX
                    pt.y = ptY
                    ori = myPathArray.pointsWithPheromone[counter].getOrientation()
                    myNewbestPath.insertPoint(pt,0.1,time,ori)
                counter += 1
                prevPoint = pose  # Save the previous pose from the best array
        if (myNewbestPath.distance != 0):
            self.bestPath = myNewbestPath
        return myNewbestPath



# myHead = self.createHeader(0,frame)
# self.bestPath.header = myHead
# self.bestPath.poses = self.createFakePose(path,frame)

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
