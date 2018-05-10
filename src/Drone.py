import math
import rospy
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
from geometry_msgs.msg import Quaternion


# We'll concern ourselves with a 2D plane for now. We can't do much about laserscanning the bottom of the drone.

class Drone:
    def __init__(self, seqTp, droneName, odomTrue, imuTrue, frameID):  # Setup
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


        self.pathCount = 0
        self.shutdown = False

        self.childFrame = frameID  # String
        self.sequence = 0  # the drone own ID number in group
        self.cmdString = ""
        self.arrayOfPaths = []
        self.myName = droneName

        self.dronePublisher = rospy.Publisher("/" + droneName, Twist, queue_size=10)  # Twist Publisher node
        self.strngPublisher = rospy.Publisher("/" + droneName, String, queue_size=10)
        self.pubPath = rospy.Publisher("/ACOpath", Path, queue_size=10)
        self.sendCommand = rospy.Publisher("/ACOcmd", String, queue_size=10)
        self.setValues(odomTrue, imuTrue)
        ####################################### Set Topics##############
        self.sequence = seqTp

    def setValues(self, odomTrue, imuTrue):  # Setup the subscribers
        droneNameLaser = "/droneLaser_" + str(self.sequence + 1)
        rospy.Subscriber(droneNameLaser, LaserScan, self.getLaser)
        if (odomTrue):
            droneNameOdom = "/droneOdom_" + str(self.sequence)
            rospy.Subscriber(droneNameOdom, Odometry, self.getOdom)
        if (imuTrue):
            droneNameImu = "/droneImu_" + str(self.sequence)
            rospy.Subscriber(droneNameImu, Imu, self.getImu)
        rospy.Subscriber("/map", OccupancyGrid, self.getMap)
        rospy.Subscriber("/goal",Point,self.callPoint)

        ############################################Callbacks###################

    def callPoint(self, data):
        self.goalPoint = data
        output = "null"

    def getLaser(self, data):
        if data:
            # rospy.loginfo(data.ranges)
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

    ##########################################
    # Start protocol to wander towards goal.
    def start_wandering(self, illterations):
        for i in range(0, illterations, 1):
            self.createNewPath()
            pointSelec = PointSelect()
            self.newMapper = Mapper()
            wandering = True
            pointSelec.setGoal(self.goalPoint)
            self.driving.setGoal(self.goalPoint)
            counter = 0
            while (wandering & self.shutdown == False):
                self.cmd_vel = Twist()
                time = rospy.Time.now()
                points, firstDrive = self.driving.locateFrontierPt(self.laserscan, self.map)
                #rospy.loginfo(points)
                if (firstDrive):
                    choice = pointSelec.selectRandom(points)
                    self.driving.firstOff()
                else:
                    choice = pointSelec.locateBestPoint(points, firstDrive)

                if (isinstance(choice,basestring)):#Choice used when the code cannot find an exit
                    counter -= 1
                    rospy.loginfo(len(self.newPath.pointsWithPheromone))
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
        #rospy.loginfo(self.goalPoint.x)
        #rospy.loginfo("help")

        if ((myCheckX / self.goalPoint.x <= 0.02) & (
                myCheckY / self.goalPoint.y <= 0.02)):  # Calculate percentage off from the origin.
            return True
        else:
            return False

    ############################Utility Functions##############################

    def reset(self):
        self.strngPublisher.publish("Reset")
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
        self.pubPath_ = False
        #self.pubCmd_ = False
        path = self.convertPath(self.bestPath)
        while not rospy.is_shutdown():
            if self.pubPath_:
                self.pubPath.publish(path)
                self.pubPath_ = False

    ##################################Twist generator###########################
    def normalizeVectorTwist(self, secondPoint):
        #myOrigin = self.lastOrigins[len(self.lastOrigins) - 1]
        #rospy.loginfo("Help")
        self.cmd_vel = Twist()
        myOrigin = self.odom.pose.pose
        opposite = myOrigin.position.y - secondPoint.y
        adjacent = myOrigin.position.x - secondPoint.x
        thethaGoal = math.atan2(adjacent, opposite)  # The Z thetha goal that we're trying to turn towards.
        radThethaGoal = math.radians(thethaGoal)
        myAngle = myOrigin.orientation.z

        if (myAngle > radThethaGoal):  # turn right
            self.right()
        elif (myAngle < radThethaGoal):  # turn left
            self.left()
        elif (radThethaGoal - self.THETHAERRORMARGIN >= myAngle <= radThethaGoal + self.THETHAERRORMARGIN):
            self.foward()
        else:
            self.escape()

        self.cmd_vel.linear = self.lin
        self.cmd_vel.angular = self.ang
        self.dronePublisher.publish(self.cmd_vel)


    ################################ROS Twist settings##########################

    def move_foward(self, dir):
        self.lin.x = self.SPEED

    def stop(self):
        self.lin.x = 0
        self.lin.y = 0
        self.lin.z = 0
        self.ang.x = 0
        self.ang.y = 0
        self.ang.z = 0

    def escape(self):
        self.lin.x = 0
        self.ang.y = self.TURN_SPEED

    def left(self):
        self.ang.y = self.TURN_SPEED

    def right(self):
        self.ang.y = -self.TURN_SPEED

    def drop(self):
        self.lin.z = 0.75 * -self.SPEED

    def rise(self):
        self.lin.z = 0.75 * self.SPEED

    ############################################################################

    #############################################################################


    # def controller_cmdList(self, CmdList):
    #     decisions = {0: self.stop, 1: self.foward, 2: self.left, 3: self.right, 4: self.escape, 5: self.rise,
    #                  6: self.fall}
    #     for cmd in CmdList:
    #         decisions[cmd]()
    #     self.cmd_vel.linear = self.lin
    #     self.cmd_vel.angular = self.ang
    #     self.dronePublisher.publish(self.cmd_vel)

    #############################Checkers for distance##########################
    #
    # def isStuck(self):
    #     if (self.lin.x == 0 & self.ang.y == 0):
    #         return True
    #     return False



    # def convertTwist(self, point):
    #     if (self.closeTo(In)):
    #         self.recalculate = True
    #     else:
    #         self.controller_cmdList(In)

    # def checkLocation(self, myPos):
    #     myCheckX = self.odom.pose.pose.position.x
    #     myCheckY = self.odom.pose.pose.position.y
    #     if ((myPos.x == myCheckX) & (myPos.y == myCheckY)):
    #         return True
    #     else:
    #         return False

    # def checkForGoal(self, myPos):
    #     myCheckX = self.goalPoint.x
    #     myCheckY = self.goalPoint.y
    #     if (myPos.x == myCheckX & myPos.y == myCheckY):
    #         return self.reset()
    #     else:
    #         return False

    # magnitude = math.sqrt((opposite* opposite)+(adjacent*adjacent))
    # xNorm = opposite/magnitude
    # yNorm = adjacent/magnitude
    # myMsg = Twist()
    # Descision Plan for between two points
    # decisions = {0:stop,1:foward,2 :left,3:right,4:escape,5:rise,6:fall}

    #                rospy.loginfo(str(self.oldTime))
    #               rospy.loginfo(str(time.to_sec())+"new") #Debug

    # if (time.to_sec() >= float(str(self.oldTime)) + self.RECORD_INTERVAL):
    #    self.newPath = PathACO()      #Create a new pathACO data type
    #    self.oldTime = time.to_sec()
    # Check if the robot hasn't crashed yet or is near the new position
    # if near or crashed, create a new point to walk towards
    # if(not self.detecWallCrash(self.laserscan) | self.checkLocation(newPt)):
    #    self.checkTwist(newPt)
    # else:
    #    newPt = self.driving.locateFrontierPt(self.laserscan, self.map,self.bestPath

    # def createNewPath(self):
    #     ##########Header for path
    #     h = std_msgs.msg.Header()
    #     h.stamp = rospy.Time.now()
    #     #h.frame_id = self.childFrame
    #     self.path.header = h
    #
    # def record_path(self):
    #     ########Data to make the pose
    #     p = PoseWithCovarianceStamped()
    #     p = self.odom.pose
    #     self.path.poses.append(p.pose)
    #     self.path.twist.append(self.cmd_vel)

    # def front_range(array_in):
    #     i = 45
    #     for i in range(0, 135):
    #         if (array_in > ranges[i] < 0.5):return False
    # 	return True
    #
    # def side_ranges(array_in):
    #     pointFound = 0
    #     j = self.MIN_DIST
    #     for j in range (0, self.MAX_DIST): #(int i = min; i < max; i++){
    #         if (array_in >ranges[j] > 1.25):pointFound+=1
    #     return pointFound

    # def controller_wanderer(self): #Controls to move the drone
    #     decisions = {0:self.stop,1:self.foward,2 :self.left,3:self.right,4:self.escape,5:self.rise,6:self.fall}
    #     if (front_range()): #if the whole array gets swampped, attempt reverse and escape.
    #         decisions[1]()
    #     if (self.stopMoving):
    #         decisions[0]()
    #     if(rangeOnSides(scan_in,0,90-SIDE_SCANS) > rangeOnSides(scan_in,SIDE_SCANS+90,180)):# printf("Turn Right");
    #         decisions[3]()
    #     if(rangeOnSides(scan_in,0,90-SIDE_SCANS) < rangeOnSides(scan_in,SIDE_SCANS+90,180)):#printf("Turn Left")
    #         decisions[2]()
    #     if(self.isStuck() && selt.stopMoving != True):#	printf("Stop");
    #         decisions[4]()
    #     self.cmd_vel.linear = self.lin
    #     self.cmd_vel.angular = self.ang
    #
    #     self.dronePublisher.publish(self.cmd_vel)
    #     #Set acceleration speeds and publish to topic

    #############################################################################
