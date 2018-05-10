import rospy
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

# The sole purpose of this file is to provide a means to drive the drone as close
# to the found waypoints without getting stuck in a corner. This would involve
# using a method to detect new paths out of a corner - pathfinding possibly.
# The path created by the drone would have to be tagged with its own things.

class DriveHandler:
    def __init__(self):
        self.DELAYSEC = 1  # Seconds
        self.MIN_DIST = 0.8  # meter
        self.MAX_DIST = 8  # meters
        self.points = 667  # Number of points on the sim robot
        # self.points = 180 #Number of Points
        self.angles = -120.0
        # self.angles = -90 #Starting Degrees
        self.openAngleSpace = 41  # points that are free in a general direction
        # self.randomizerGen = randint()
        self.lastOrigins = []  # Collects Poses with position and quaterion values
        self.THETHAERRORMARGIN = 0.01  # In radians, adds a margin of error to drive towards.
        self.firstStart = True
        self.goalPoint = Point()
        self.oldTime = rospy.Time.now()

    def setGoal(self, goalPt):
        self.goalPoint = goalPt

    # Functon that returns a point that the robot can go towards once it has a laserscan in view. The idea is that once it runs
    # a point is returned that can point towards a point that is sufficently clear to go to.
    def locateFrontierPt(self, laserscanIn, mapIn):  # escapePath is a bool that deternimes if this is a escape routine
        myScan = LaserScan()
        myLocation = Pose()
        mymap = OccupancyGrid()
        myLocation = MapMetaData()
        mymap = mapIn
        myScan = laserscanIn
        myLocation = mymap.info.origin
        width = mymap.info.width
        densityOccupation = []
        densityClusters = []
        # Generate a radius of density around the robot
        for point in range (0, 667 - 1,1):
            rospy.loginfo(len(myScan.ranges))
            x_other = myLocation.position.x + myScan.ranges[point] * math.cos(self.angles + myLocation.orientation.z)
            y_other = myLocation.position.y + myScan.ranges[point] * math.sin(self.angles + myLocation.orientation.z)
            #rospy.loginfo(myScan.ranges[point])
            point = x_other + width * y_other
            #rospy.loginfo(point)


            if (not math.isinf(point) and not math.isnan(point)):
                densityOccupation.append(int(point))
            self.angles += 0.35982
        # Locate a midpoint of density
        #rospy.loginfo(densityOccupation)




        for mapPoint in densityOccupation:

            if (mymap.data[mapPoint] > 0):
                densityClusters.append(0)
            else:
                densityClusters.append(1)

        #rospy.loginfo(densityClusters)
        points = []
        for point in densityClusters:  # Convert all possible "walkable" points into real points instead of map points.
            if (point == 1):
                myPoint = self.convertMapArrayIntoXY(point, width)
                points.append(myPoint)
        #rospy.loginfo(len(points))
        if (len(points) < 100):  # We want to verify that we can walk foward if we can possibly can do so.
            return "Blocked", False
        else:
            return points, self.firstStart # Returns an array of points

    def firstOff(self):
        self.firstStart = False

    def run(self):
        detec = self.detecWallCrash()
        if (detec):
            return "crash"
        else:
            return self.locateFrontierPt(False)

    # Function to convert a map index into coordinate X, Y points
    def convertMapArrayIntoXY(self, point, width):
        x_mapPoint = point / width
        y_mapPoint = (point - x_mapPoint) / width
        finalPoint = Point()
        finalPoint.x = x_mapPoint
        finalPoint.y = y_mapPoint
        return finalPoint
