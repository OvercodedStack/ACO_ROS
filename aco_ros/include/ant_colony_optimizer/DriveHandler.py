#!/usr/bin/env python
import rospy
import nav_msgs
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
import sensor_msgs
from random import randint
from std_msgs import String


#The sole purpose of this file is to provide a means to drive the drone as close
#to the found waypoints without getting stuck in a corner. This would involve
#using a method to detect new paths out of a corner - pathfinding possibly.

#The path created by the drone would have to be tagged with its own things.

class DriveHandler:

    def __init__(self,Path):
        self.myNum = 0
        self.MAX_DIST = 8 #meters
        self.points = 180
        self.angles = -90 #Starting Degrees
        self.openAngleSpace = 41 #points that are free in a general direction
        self.randomizerGen = new randint()
        self.lastOrigins = [] #Collects Poses with position and quaterion values
        self.THETHAERRORMARGIN = 0.01 #In radians, adds a margin of error to drive towards.
        self.goalPoint = Point()


    #Functon that returns a point that the robot can go towards once it has a laserscan in view. The idea is that once it runs
    # a point is returned that can point towards a point that is sufficently clear to go to.
    def locateFrontierPt(self,laserscanIn, map, goalPt):
        myScan = LaserScan()
        myLocation = Pose()
        myScan = laserscanIn
        myLocation = map.info.origin
        width = map.info.width
        densityOccipation = []
        mapLocations = []
        densityClusters = []
        self.goalPoint = goalPt

        #Drop the last point on stack if reached
        #if (len(self.lastOrigins) >=4):
        #    self.lastOrigins.pop(0)
        #self.lastOrigins.append(myLocation)

        #Generate a radius of density around the robot
        for point in (0, self.points):
            x_other = myLocation.position.x + myScan[point]*cos(self.angles[point]+myLocation.Orientation.z)
            y_other = myLocation.position.y + myScan[point]*sin(self.angles[point]+myLocation.Orientation.z)
            point = x_other + width*y_other
            densityOccupation.append(int(point))

        #Locate a midpoint of density
        for mapPoint in densityOccipation:
            if (mapPoint > 0):
                densityClusters.append(0)
            else:
                densityClusters.append(1)

            #averageDensity += mapPoint
        checking = True
        myCluster = []
        clusters = []

        #Find the appropiate exit "lines" around the robot
        for point in densityClusters:
            if (point == 1):
                myCluster.append(densityClusters.index(point))
            if (len(myCluster) ==41):
                clusters.append(myCluster)
                myCluster = []
        myDirections = []

        #For each cluster of free points, find the midpoint.
        if clusters:
            for cluster in clusters:
                myDirections.append(cluster((len(cluster)//2))))

        #For the points found, select a random direction to go towards. Might change in the future to influence directionself.

            frontierPoint_raw = myDirections(self.randomizerGen(0,len(myDirections))) #The final point we want to go.
            x_mapPoint = frontierPoint_raw/width
            y_mapPoint = (frontierPoint_raw - x_mapPoint )/width
            finalFrontierPoint = Point()
            finalFrontierPoint.x = x_mapPoint
            finalFrontierPoint.y = y_mapPoint
            return finalFrontierPoint
        else:
            return self.lastOrigins[len(self.lastOrigins)-1]
            #If there's nothing in our array, we're stuck in a corner or it's impossible to move. So we must send back an old point to walk back to.

        #Determine the final points on the real map
        # point = x_other + width*y_other


    def getMeOut(self):
        clusters



    def generteTwist(self, secondPoint):
        myOrigin = self.lastOrigins[len(self.lastOrigins)-1]
        opposite = myOrigin.position.x - secondPoint.x
        adjacent = myOrigin.position.y - secondPoint.y
        thethaGoal = math.atan_2(adjacent,opposite) #The Z thetha goal that we're trying to turn towards.
        radThethaGoal = math.radians(thethaGoal)
        myAngle = myOrigin.orientation.z

        #Descision Plan for between two points
        #decisions = {0:stop,1:foward,2 :left,3:right,4:escape,5:rise,6:fall}
        if (myAngle > radThethaGoal): #turn right
            return 3
        elif(myAngle < radThethaGoal): #turn left
            return 2
        elif(radThethaGoal - self.THETHAERRORMARGIN >= myAngle <= radThethaGoal + self.THETHAERRORMARGIN )
            return 1
        else:
            return 1
