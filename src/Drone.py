#!/usr/bin/env python

import math
import rospy
import time as pyTime
from MapHandler import Mapper
from PathACO import *
from PointSelector import PointSelect
from DriveHandler import DriveHandler
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

# We'll concern ourselves with a 2D plane for now. We can't do much about laserscanning the bottom of the drone.

class Drone:
    def __init__(self, seqTp, droneName, odomTrue, imuTrue, frameID):  # Setup
        rospy.init_node('Ant_Colony_Wrapper')
        rospy.loginfo("Starting Drone " + droneName)
        self.SPEED = 0.40
        self.SIDE_DISTANCE = 0.50
        self.TURN_SPEED = 0.55
        self.MAX_DIST = 0.5
        self.MIN_DIST = 0.45
        self.LONG_DIST = 1.00
        self.AMOUNT_OF_POINTS = 180.0  # //Amount of points in scan_in laserscan
        self.MULTIPLIER = (self.AMOUNT_OF_POINTS / 2) / 90
        self.FRONT_SCAN_ANGLE = (math.atan(self.SIDE_DISTANCE / self.MIN_DIST) * (180 / math.pi))
        self.THETHAERRORMARGIN = 0.01
        #	//Calculate front cone angle
        self.SIDE_SCAN_ANGLE = 30
        self.FRONT_SCANS = self.MULTIPLIER * self.FRONT_SCAN_ANGLE  # Scanpoints in the front cone
        self.SIDE_SCANS = self.MULTIPLIER * self.SIDE_SCAN_ANGLE  # Scanpoints in the side cones
        self.AMOUNT_OF_POINTS = 180.0  # Amount of points in the laserscan
        self.RECORD_INTERVAL = 3.0  # Seconds
        self.goalPoint = Point()
        self.map = OccupancyGrid()
        self.driving = DriveHandler()
        time = rospy.Time.now()
        self.DELAYSEC = 1.5
        self.oldTime = time.to_sec()
        self.laserscan = LaserScan()
        # self.laserscan = geometry_msgs.LaserScan()
        self.imu = Imu()

        self.bestPath = PathACO()
        self.odom = Odometry()
        self.twistMsg = Twist()
        self.lin = Vector3()
        self.ang = Vector3()
        self.cmd_vel = Twist()

        self.path = Path()
        self.pathCount = 0
        self.shutdown = False

        self.childFrame = frameID  # String
        self.sequence = 0  # the drone own ID number in group
        self.cmdString = ""
        self.arrayOfPaths = []
        self.myName = droneName

        droneNameLaser = "/droneLaser_" + str(seqTp)
        if (odomTrue):
            droneNameOdom = "/droneOdom_" + str(seqTp)
            rospy.Subscriber(droneNameOdom, Odometry, self.getOdom)
        if (imuTrue):
            droneNameImu = "/droneImu_" + str(seqTp)
            rospy.Subscriber(droneNameImu, Imu, self.getImu)
        rospy.Subscriber(droneNameLaser, LaserScan, self.getLaser)
        rospy.Subscriber("/map", OccupancyGrid, self.getMap)
        rospy.Subscriber("/goal",Point,self.callPoint)

        self.pathPublisher = rospy.Publisher("/ACO_path",Path,queue_size=10)
        #self.dronePublisher = rospy.Publisher("/" + droneName, Twist, queue_size=10)  # Twist Publisher node
        self.dronePublisher = rospy.Publisher("/droneCmd_1", Twist, queue_size=10)  # Twist Publisher node

        self.strngPublisher = rospy.Publisher("/" + droneName, String, queue_size=10)
        ####################################### Set Topics##############
        self.sequence = seqTp
        self.reset = False


    ##########################################
    # Start protocol to wander towards goal.
    def start_wandering(self, illterations):
        self.reset = False
        for i in range(0, illterations, 1):
            self.createNewPath()
            pointSelec = PointSelect()
            self.newMapper = Mapper()
            wandering = True
            pointSelec.setGoal(self.goalPoint)
            self.driving.setGoal(self.goalPoint)
            counter = 0
            while (wandering & self.shutdown == False):
                choice = Point()
                self.cmd_vel = Twist()
                time = rospy.Time.now()
                points, firstDrive = self.driving.locateFrontierPt(self.laserscan,self.map,self.odom)
                if (firstDrive):
                    choice = pointSelec.selectRandom(points)
                    self.driving.firstOff()
                else:
                    choice = pointSelec.locateBestPoint(points, firstDrive)

                if (isinstance(choice,basestring)):#Choice used when the code cannot find an exit
                    counter -= 1
                    choice = self.newPath.pointsWithPheromone[counter].getPoint()
                    self.normalizeVectorTwist(choice)
                    self.removeLastPointPath()
                else:
                    self.normalizeVectorTwist(choice)
                    counter += 1
                if (self.arrivedAtGoal()):  # "Alive" condition to shutoff a drone if needed
                    wandering = False
            self.bestPath = self.newMapper.checkForBestPath(self.newPath)
            self.publish()
            self.reset()

    def arrivedAtGoal(self):
        myCheckX = self.odom.pose.pose.position.x
        myCheckY = self.odom.pose.pose.position.y
        if ((myCheckX / self.goalPoint.x <= 0.02) & (
                myCheckY / self.goalPoint.y <= 0.02)):  # Calculate percentage off from the origin.
            return True
        else:
            return False

    ############################Utility Functions##############################

    def reset(self):
        self.reset = True
        self.strngPublisher.publish(str(self.reset))
        # Publisher that publishes a string that resets drone.

    def detecWallCrash(self, laserIn):
        if (rospy.Time.now().to_sec() >= float(str(self.oldTime)) + self.DELAYSEC):
            for range in laserIn.ranges:
                if (range < self.MIN_DIST):
                    return True
            self.oldTime = rospy.Time.now()
        return False

    def shutDownDrone(self):
        self.shutdown = True

    ############################Path functions    ##############################
    def removeLastPointPath(self):
        last = len(self.newPath.pointsWithPheromone)
        dropDist = self.newPath.pointsWithPheromone[last-1].getDistance()
        self.newPath.removeDist(dropDist)
        self.newPath.pointsWithPheromone.pop(last-1)

    def createNewPath(self):
        self.newPath = PathACO()

    def convertPath(self,PathACOIn):
        size = len(PathACOIn.pointsWithPheromone)
        thePath = Path()
        head = self.createHeader(self.pathCount,self.myName)
        thePath.header = head
        for points in range(0,size,1):
            posePath = PoseStamped()
            h = self.createHeader(points,self.myName)
            posePath.header = h
            pose = Pose()
            pose.position = PathACOIn.pointsWithPheromone[points].getPoint()
            pose.orientation = PathACOIn.pointsWithPheromone[points].getOrientation()
            posePath.pose = pose
            thePath.poses.append(posePath)
        self.pathCount += 1
        return thePath

    def record_point(self, point, orientation):
        pheromone = 0.1  # TODO this
        self.newPath.insertPoint(point, pheromone, rospy.Time.now(), orientation)

    def savePath(self, last):
        self.arrayOfPaths.append(self.newPath)  # REMEMBER, THERE'S AN ARRAY OF PATHS HERE
        if (not last):
            self.creteNewPath()

    #################################Utilities###################################

    def createHeader(self, seq, frame):
        h = Header()
        h.seq = seq
        h.stamp = rospy.Time.now()
        h.frame_id = frame
        return h

    def createFakePose(self, path, frame):
        poses = []
        tempPose = PoseStamped()  # A temporary poseStampped message to write into before pasting into the poses array.
        count = 0
        h = self.createHeader(count, frame)
        for item in path.pointsWithPheromone:
            insidePose = Pose()
            insidePose.position = item.getPoint
            insidePose.orientation = item.getOrientation
            tempPose.header = h
            tempPose.pose = insidePose
            poses.append(tempPose)
        return poses

    #########################Publish best path so far###########################

    def publish(self):  # Set a publishing state to push data out
        self.path = self.convertPath(self.bestPath)
        ############################################Callbacks###################

    def callPoint(self, data):
        self.goalPoint = data
        output = "null"

    def getLaser(self, data):
        if data:
            self.laserscan = data
        else:
            return "Problem with laser in " + str(self.sequence) + "."

    def getMap(self, map):
        if map:
            self.map = map
        else:
            return "Problem with map"

    def getImu(self, imuIn):
        if imuIn:
            self.imu = imuIn
        else:
            return "No Imu provided in " + str(self.sequence) + "."

    def getOdom(self, odomIn):
        if odomIn:
            self.odom = odomIn
        else:
            return "Problem with odom in " + str(self.sequence) + "."


    ##################################Twist generator###########################
    def normalizeVectorTwist(self, secondPoint):
        margin = 0.05
        myOrigin = self.odom.pose.pose
        distCheckOld = math.sqrt((myOrigin.position.y - secondPoint.y )**2+( myOrigin.position.x - secondPoint.x)**2)
        rospy.loginfo(secondPoint)
        closeBy = False
        while (self.arrivedAtPoint(secondPoint)  == False):
            myFront = self.odom.pose.pose.orientation.z
            self.cmd_vel = Twist()
            opposite =  myOrigin.position.y - secondPoint.y
            adjacent =  myOrigin.position.x - secondPoint.x
            thethaGoal = math.atan2(adjacent, opposite)  # The Z thetha goal that we're trying to turn towards.
            radThethaGoal = math.radians(thethaGoal)
            rospy.loginfo(radThethaGoal)
            rospy.loginfo(myFront)
            myDistance = math.sqrt((opposite)**2+(adjacent)**2)
            #rospy.loginfo(self.odom.pose.pose.orientation)

            #if (round(radThethaGoal,4) == round(myFront,4)):
            if (radThethaGoal-margin  <= myFront <= radThethaGoal+margin):
                self.lin.x = self.SPEED
            else:
                self.direction(radThethaGoal,myFront)
            #rospy.loginfo(secondPoint)
            if (myDistance > distCheckOld+0.1):
                break
            self.cmd_vel.linear = self.lin
            self.cmd_vel.angular = self.ang
            self.dronePublisher.publish(self.cmd_vel)
        rospy.loginfo("Beep")
        self.stop()
        self.cmd_vel.linear = self.lin
        self.cmd_vel.angular = self.ang
        self.dronePublisher.publish(self.cmd_vel)

    def arrivedAtPoint(self,point):

        myCheckX = self.odom.pose.pose.position.x
        myCheckY = self.odom.pose.pose.position.y
        if ((myCheckX/point.x >= 0.95) and (myCheckY/point.y >= 0.95)):  # Calculate percentage off from the origin.
            return True
        else:
            return False
    ################################ROS Twist settings##########################

    def move_foward(self):
        self.lin.x = self.SPEED

    def stop(self):
        self.lin.x = 0
        self.lin.y = 0
        self.lin.z = 0
        self.ang.x = 0
        self.ang.y = 0
        self.ang.z = 0

    def direction(self,dir,dirOdom):
        speed = 1 * abs(self.odom.pose.pose.orientation.z / dir)
        if (dir > dirOdom ):
            self.ang.z = 0.1
        else:
            self.ang.z = -0.1
        self.lin.z = speed


    def escape(self):
        self.lin.x = 0
        #self.lin.y = self.TURN_SPEED
        self.ang.z = self.TURN_SPEED

    def left(self):
        #self.lin.y = self.TURN_SPEED
        self.ang.z = self.TURN_SPEED

    def right(self):
        #self.lin.y = -self.TURN_SPEED
        self.ang.z = -self.TURN_SPEED

    def drop(self):
        self.lin.z = 0.75 * -self.SPEED

    def rise(self):
        self.lin.z = 0.75 * self.SPEED

    ############################################################################

    def run(self):
        r = rospy.Rate(10)
        NUMBER_OF_ILLETERATIONS = 1000
        pyTime.sleep(2)
        while not rospy.is_shutdown():
            self.start_wandering(NUMBER_OF_ILLETERATIONS)
            self.pathPublisher.publish(self.path)
            rospy.loginfo("Publishing.")
            r.sleep()

    #############################################################################

# Trigger
if __name__ == '__main__':

    sequence = 1
    droneName = "droneACO_1"
    odomTrue = True
    IMUTrue = True
    frameID = "/Drone_1"
    try:
        main_prog = Drone(sequence,droneName,odomTrue,IMUTrue,frameID)
        main_prog.run()
    except rospy.ROSInterruptException:
        pass




        # if (myAngle > radThethaGoal):  # turn right
        #     self.move_foward()
        #     self.right()
        # elif (myAngle < radThethaGoal):  # turn left
        #     self.move_foward()
        #     self.left()
        # elif (radThethaGoal - self.THETHAERRORMARGIN >= myAngle <= radThethaGoal + self.THETHAERRORMARGIN):
        #     self.foward()
        # else:
        #     self.escape()