#!/usr/bin/env python
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


    def locateFrontierPts(self,laserscanIn, map):
        myScan = LaserScan()
        myLocation = Pose()
        myScan = laserscanIn
        myLocation = map.info.origin
        width = map.info.width
        densityOccipation = []

        #Generate a radius of density around the robot
        for point in (0, self.points):
            x_other = myLocation.x + myScan[point]*cos(self.angles[point])
            y_other = myLocation.y + myScan[point]*sin(self.angles[point])
            point = x_other + width*y_other
            densityOccupation.append(point)

        densityClusters = []
        #Locate a midpoint of density
        for mapPoint in densityOccipation:
            if (mapPoint > 0):
                densityClusters.append(0)
            else:
                densityClusters.append(1)

            #averageDensity += mapPoint
        checking = True
        while (checking):
            for point in densityClusters:
                if (point == 1):
                    

    #
