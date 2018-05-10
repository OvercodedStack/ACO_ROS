from random import randint
import rospy
import math
from geometry_msgs.msg import Point
from PathACO import *


class PointSelect:
    def __init__(self):
        self.bestPath = PathACO()
        self.robotSize = 0.5  # In meters - this is the "real" world size of the robot. Use it to calculate best path through a field of view.
        self.laserscanSize = 667  # In points
        self.robtSize = 60  # In points TODO: finish code to calculate size
        self.sequence = 0

    def setGoal(self, goal):
        self.goalPoint = Point()
        self.goalPoint = goal

    def locateBestPoint(self, pathChoice,firstDrive):
        # We'll define pathChoice as the array of free points around the arc of vision (Laserscan)
        if (isinstance(pathChoice, basestring)):
            return "Can't Move"
        if (len(self.bestPath.pointsWithPheromone) < 2 | firstDrive != True):
            goalX = self.goalPoint.x
            goalY = self.goalPoint.y
        else:
            pointFromBest = self.bestPath.pointsWithPheromone[self.sequence].getPoint()
            goalX = pointFromBest.X
            goalY = pointFromBest.Y
        distArray = []
        for point in pathChoice:  # We're dealing with an array of points, we want to locate the point that's closest to the "best path" at the moment. If there's none set however, we will try to point towards the goal in question.
            ptDist = math.sqrt((goalX - point.x) ** 2 + (goalY - point.y) ** 2)
            distArray.append(ptDist)
        closestPoint = min(distArray)
        pointInReference = distArray.index(closestPoint)  # We choose our point to go towards.
        direction = randint(int(pointInReference-pointInReference*0.5),int(pointInReference+pointInReference*0.5))
        #direction = randint(int(closestPoint - closestPoint*0.5),int( closestPoint + closestPoint*0.5))  # Simulate random choice behaviour
        rospy.loginfo(min(distArray))
        pointSelected = pathChoice[direction]
        self.sequence += 1
        return pointSelected

    def selectRandom(self,points):
        size = len(points)
        prevDist = 1
        pointCount = 0
        for p in range(0,size,1):
            if p+1 > size:
                break
            dist = math.sqrt((points[p].x-points[p+1].x)**2+(points[p].y-points[p+1].y)**2)
            if (dist/prevDist < 0.05):
                pointCount +=1
            else:
                pointCount = 0
            if (pointCount == self.robtSize):
                bestPoint = points[p]
                break
        return bestPoint


    def setBestPath(self, best):
        self.bestPath = best
