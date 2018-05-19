import rospy
import nav_msgs
import sensor_msgs
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
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
        
        self.points = 667 #Number of points on the sim robot
        #self.points = 180 #Number of Points
        self.angles = -120.0
        #self.angles = -90 #Starting Degrees
        self.openAngleSpace = 41 #points that are free in a general direction
        #self.randomizerGen = randint()
        self.lastOrigins = [] #Collects Poses with position and quaterion values
        self.THETHAERRORMARGIN = 0.01 #In radians, adds a margin of error to drive towards.
        self.goalPoint = Point()
        self.oldTime = rospy.Time.now()

    def setGoal(self,goalPt):
        self.goalPoint = goalPt

    #Functon that returns a point that the robot can go towards once it has a laserscan in view. The idea is that once it runs
    # a point is returned that can point towards a point that is sufficently clear to go to.
    def locateFrontierPt(self,laserscanIn, mymap,escapePath): #escapePath is a bool that deternimes if this is a escape routine
        myScan = LaserScan()
        myLocation = Pose()
        myScan = laserscanIn
        myLocation = mymap.info.origin
        width = mymap.info.width
        densityOccupation = []
        densityClusters   = []
        #Generate a radius of density around the robot
        for point in (0, self.points -1):
            #rospy.loginfo(point)
            #rospy.loginfo(myScan.ranges) 
            x_other = myLocation.position.x + myScan.ranges[point]*math.cos(self.angles+myLocation.orientation.z)
            
            y_other = myLocation.position.y + myScan.ranges[point]*math.sin(self.angles+myLocation.orientation.z)
            point = x_other + width*y_other
            densityOccupation.append(int(point))
            self.angles += 0.35982
        #Locate a midpoint of density
        for mapPoint in densityOccupation:
            if (mapPoint > 0):
                densityClusters.append(0)
            else:
                densityClusters.append(1)
        points = []
        for point in densityClusters #Convert all possible "walkable" points into real points instead of map points. 
            if (point == 1): 
                myPoint = self.convertMapArrayIntoXY(point,width)
                points.append(myPoint)
        return points #Returns an array of points

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


        #myCluster = []
        #clusters  = []

        #Find the appropiate exit "lines" around the robot
        #for point in densityClusters:
        #    if (point == 1):
        #        myCluster.append(densityClusters.index(point))
        #    if (len(myCluster) ==41): #Points that can cross the drone with a midpoint point.
        #        clusters.append(myCluster)
        #        myCluster = []
        #myDirections = []
        #if clusters > 2: 
        #    for cluster in clusters:
        #        myDirections.append(cluster[(len(cluster)//2)])
        #    oldPt = Point()
             #   return myDirections #Return an array of possible points to walk towards to.
#Scroll the list of points possible to roll foward and find the closest point to the goal. Select this point to drive foward.
        #    for point in myDirections:
        #        finalFrontierPoint = self.convertMapArrayIntoXY(point,width)
        #        myClosestPt = math.sqrt((self.goalPoint.x - finalFrontierPoint.x)**2 + (self.goalPoint.y-finalFrontierPoint.x)**2)
        #        if (oldPt.x == 0):
        #            oldPt = myClosestPt
        #        elif (myClosestPt < oldPt):
        #            oldPt = myClosestPt
        #    return oldPt #Closest point so far to goal
        #else:
        #    return self.lastOrigins[len(self.lastOrigins)-1]
            #If there's nothing in our array, we're stuck in a corner or it's impossible to move. So we must send back an old point to walk back to.



        #Drop the last point on stack if reached
        #if (len(self.lastOrigins) >=4):
        #    self.lastOrigins.pop(0)
        #self.lastOrigins.append(myLocation)

        #Determine the final points on the real map
        # point = x_other + width*y_other

    #Slow delay check the laserscan every DELAYSEC seconds


        #frontierPoint_raw = myDirections(self.randomizerGen(0,len(myDirections))) #The final point we want to go.
