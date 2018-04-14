#!/usr/bin/env python
#Personalized path data type
import rospy
from geometry_msgs.msg import Point
import math
#The packaged point data
class packagedPt:
    def __init__(self,pt,pheromone,dist):
        self.point = Point()
        self.point = pt         #The point data
        self.phero = pheromone  #The pheromone influence
        self.distanceij = dist  #The distance data

    def getPoint(self):
        return self.point
    def getDistance(self):
        return self.distanceij
    def getPhero(self):
        return self.phero
#This is a data type that contains prepackaged information for the point, the distance from itself and the next, and
#an array that contains all the points from start to end.
class PathACO:
    #Initialize everything at zero, we don't want anything to mess with here.
    def __init__(self):
        self.pointsWithPheromone.append(Point())
        self.pointsWithPheromone = []
        self.distance = 0
    #Append a point to the array and calculate the distance from one to another
    def insertPoint(self,pt,pheromone,time):
        self.time = time
        ptDist = self.calcDist(pt)
        myPointPk = packagedPt(pt,pheromone,ptDist)
        self.pointsWithPheromone.append(myPointPk)
        self.distance += ptDist
    #Calculate distance
    def calcDist(self,pt):
        lastPt = self.pointsWithPheromone[len(self.pointsWithPheromone)-1]
        distance = math.sqrt((pt.x-lastPt.x)**2+(pt.y-lastPt.y)**2)
        return distance
