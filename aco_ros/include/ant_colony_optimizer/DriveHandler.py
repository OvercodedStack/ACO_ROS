#!/usr/bin/env python
import rospy
import nav_msgs
import sensor_msgs
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from random import randint
from std_msgs.msg import String


#The sole purpose of this file is to provide a means to drive the drone as close
#to the found waypoints without getting stuck in a corner. This would involve
#using a method to detect new paths out of a corner - pathfinding possibly.

#The path created by the drone would have to be tagged with its own things.

class DriveHandler:
    def __init__(self):
        self.DELAYSEC = 1 #Seconds
        self.MIN_DIST = 0.8 #meter
        self.MAX_DIST = 8 #meters
        self.points = 180 #Number of Points
        self.angles = -90 #Starting Degrees
        self.openAngleSpace = 41 #points that are free in a general direction
        self.randomizerGen = new randint()
        self.lastOrigins = [] #Collects Poses with position and quaterion values
        self.THETHAERRORMARGIN = 0.01 #In radians, adds a margin of error to drive towards.
        self.goalPoint = Point()
        self.oldTime = rospy.Time.now()

    def setGoal(self,goalPt):
        self.goalPoint = goalPt

    #Functon that returns a point that the robot can go towards once it has a laserscan in view. The idea is that once it runs
    # a point is returned that can point towards a point that is sufficently clear to go to.
    def locateFrontierPt(self,laserscanIn, map,escapePath,passArray): #escapePath is a bool that deternimes if this is a escape routine
        myScan = LaserScan()
        myLocation = Pose()
        myScan = laserscanIn
        myLocation = map.info.origin
        width = map.info.width
        densityOccipation = []
        mapLocations      = []
        densityClusters   = []

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
        clusters  = []

        #Find the appropiate exit "lines" around the robot
        for point in densityClusters:
            if (point == 1):
                myCluster.append(densityClusters.index(point))
            if (len(myCluster) ==41):
                clusters.append(myCluster)
                myCluster = []
        myDirections = []

        #For the points found, select a random direction to go towards. Might change in the future to influence directionself.
        #For each cluster of free points, find the midpoint.
        if clusters:
            for cluster in clusters:
                myDirections.append(cluster[(len(cluster)//2)])
            oldPt = Point()
            elif (passArray):
                return myDirections #Return an array of possible points to walk towards to.
        #Scroll the list of points possible to roll foward and find the closest point to the goal. Select this point to drive foward.
            for point in myDirections:
                finalFrontierPoint = self.convertMapArrayIntoXY(point,width)
                myClosestPt = math.sqrt((self.goalPoint.x - finalFrontierPoint.x)**2 + (self.goalPoint.y - finalFrontierPoint.x)**2)
                if (oldPt.x == 0):
                    oldPt = myClosestPt
                elif (myClosestPt < oldPt):
                    oldPt = myClosestPt
            return oldPt #Closest point so far to goal
        else:
            return self.lastOrigins[len(self.lastOrigins)-1]
            #If there's nothing in our array, we're stuck in a corner or it's impossible to move. So we must send back an old point to walk back to.

        #Determine the final points on the real map
        # point = x_other + width*y_other

    #Slow delay check the laserscan every DELAYSEC seconds
    def detecWallCrash(self,laserIn):
        if (rospy.Time.now()+self.DELAYSEC >= self.oldTime):
            for range in laserIn.ranges:
                if (range < self.MIN_DIST):
                    return true
            self.oldTime = rospy.Time.now()
        return False

    def run(self):
        detec = self.detecWallCrash()
        if (detec):
            return "crash"
        else:
            return self.locateFrontierPt(False)

    #Function to convert a map index into coordinate X, Y points
    def convertMapArrayIntoXY(self,point,width):
        x_mapPoint = point/width
        y_mapPoint = (point - x_mapPoint)/width
        finalPoint = Point()
        finalPoint.x = x_mapPoint
        finalPoint.y = y_mapPoint
        return finalPoint

    #Twist generator
    def generteTwist(self, secondPoint,Odometry):
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
            return "Do the rotate"





        #frontierPoint_raw = myDirections(self.randomizerGen(0,len(myDirections))) #The final point we want to go.
