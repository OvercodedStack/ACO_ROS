#!/usr/bin/env python
import rospy
import ros
import tf
import nav_msgs
from nav_msgs.msg import Map
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import sensor_msgs
import math
from std_msgs import String

#We'll concern ourselves with a 2D plane for now. We can't do much about laserscanning the bottom of the drone.
class Drone:
    def __init__(self,droneName,seqTp,odomTrue,imuTrue): #Setup
        self.SPEED              = 0.40
        self.TURN_SPEED         = 0.55
        self.MAX_DIST           = 0.5
        self.MIN_DIST           = 0.45
        self.LONG_DIST          = 1.00
        self.AMOUNT_OF_POINTS   = 180.0#             //Amount of points in scan_in laserscan
        self.MULTIPLIER 		= (AMOUNT_OF_POINTS/2)/90
        self.FRONT_SCAN_ANGLE	= (atan(SIDE_DISTANCE/MINIMUM_DISTANCE)*(180/M_PI))#	//Calculate front cone angle
        self.SIDE_SCAN_ANGLE	= 30
        self.FRONT_SCANS	    = self.MULTIPLIER * self.FRONT_SCAN_ANGLE#Scanpoints in the front cone
        self.SIDE_SCANS		    = self.MULTIPLIER * self.SIDE_SCAN_ANGLE #Scanpoints in the side cones
        self.AMOUNT_OF_POINTS   = 180.0
        self.RECORD_INTERVAL    = 3 #Seconds

        self.map = Map()
        self.driving  = DriveHandler()
        self.shutdown = False
        self.oldTime  = rospy.Time.to_sec()
        self.laserscan= geometry_msgs.LaserScan()
        self.imu      = geometry_msgs.Imu()
        self.sequence = 0           #the drone own ID number in group

        self.childFrame = frameID    #String
        self.path       = nav_msgs.Path()
        self.goalArray  = actionlib_msgs.GoalStatusArray()
        self.cmdString  = ""
        self.odom       = Odometry()
        self.twistMsg   = Twist()
        self.lin        = Vector3()
        self.ang        = Vector3()
        self.cmd_vel    = Twist()
        self.myName     = droneName
        self.dronePublisher = rospy.Publisher(myName,Twist,queue_size=10) #Twist Publisher node
        self.setValues(odomTrue,imuTrue)
        ####################################### Set Topics##############
        self.sequence = seqTp

    def setValues(self,odomTrue,imuTrue):#Setup the subscribers
        droneNameLaser = "/droneLaser_" + str(self.sequence)
        rospy.Subscriber(droneNameLaser,LaserScan, self.getLaser)
        if (odomTrue):
            droneNameOdom = "/droneOdom_" + str(self.sequence)
            rospy.Subscriber(droneNameOdom,Odometry, self.getOdom)
        if (imuTrue):
            droneNameImu = "/droneImu_" + str(self.sequence)
            rospy.Subscriber(droneNameImu,Imu,self.getImu)

    ############Callbacks###################
    def getLaser(laser):
        if laser != null:
            self.laserscan = LaserScan()
            self.laserscan = laser
        else:
            return "Problem with laser in " + str(self.sequence) + "."
    def getImu(imuIn):
        if imuIn != null:
            self.imu = imuIn
        else:
            return "No Imu provided in " + str(self.sequence)+ "."
    def getOdom(odomIn):
        if odomIn != null:
            self.odom = odomIn
        else:
            return "Problem with odom in " + str(self.sequence)+ "."
    ##########################################
    #Start protocol to wander towards goal.
    def start_wandering(self):
        wandering = True
        #self.driving - the DriveHandler object
        newPt = self.driving.locateFrontierPt(self.laserscan, self.map,self.path)
        while(wandering && self.shutdown == False):
            self.cmd_vel = Twist()
            if (rospy.Time.to_sec() >= oldTime + self.RECORD_INTERVAL):
                self.record_path()
                self.oldTime = rospy.Time.to_sec()

            #Check if the robot hasn't crashed yet or is near the new position
            #if near or crashed, create a new point to walk towards
            if(not self.detecWallCrash(self.laserscan) || self.checkLocation(newPt)):
                self.checkTwist(newPt)
            else:
                newPt = self.driving.locateFrontierPt(self.laserscan, self.map,self.path)

            #self.controller()
            if (self.shutdown == True): #"Alive" condition to shutoff a drone if needed
                wandering = False

    def checkTwist(self,In):
        if (in == "Crash"):
            self.left()
        else:
            if (self.closeTo(In)):
                self.recalculate = True
            else:
                self.controller_cmdList(In)

    def checkLocation(myPos):
        myCheckX = self.odom.pose.pose.position.x
        myCheckY = self.odom.pose.pose.position.y
        if (myPos.x == myCheckX && myPos.y == myCheckY):
            return True
        else:
            return False

    def detecWallCrash(self,laserIn):
        if (rospy.Time.now()+self.DELAYSEC >= self.oldTime):
            for range in laserIn.ranges:
                if (range < self.MIN_DIST):
                    return true
            self.oldTime = rospy.Time.now()
        return False


    ############################Path functions    ##############################
    def createNewPath(self):
        ##########Header for path
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        #h.frame_id = self.childFrame
        self.path.header = h

    def record_path(self):
        ########Data to make the pose
        p.pose = self.odom.pose()
        self.path.poses.append(p)
    def shutDownDrone(self):
        self.shutdown = True

    #########################Publish best path so far###########################

    def publish(self): #Set a publishing state to push data out
        self.pubPath_ = False
        self.pubCmd_ = False
        pubPath = rospy.Publisher("/ACOpath", Path, queue_size=10)
        sendCommand = rospy.Publisher("/ACOcmd", String, queue_size=10)
        while not rospy.is_shutdown():
            if self.pubPath_:
                pubPath.publish(self.path)
                self.pubPath_ = False
            if self.pubCmd_:
                sendCommand.publish(self.cmdString)
                self.pubCmd_ = False
    ##################################Twist generator###########################
    def normalizeVectorTwist(self, secondPoint,Odometry):
        myOrigin = self.lastOrigins[len(self.lastOrigins)-1]
        opposite = myOrigin.position.y - secondPoint.y
        adjacent = myOrigin.position.x - secondPoint.x
        thethaGoal = math.atan_2(adjacent,opposite) #The Z thetha goal that we're trying to turn towards.
        radThethaGoal = math.radians(thethaGoal)
        myAngle = myOrigin.orientation.z
        # magnitude = math.sqrt((opposite* opposite)+(adjacent*adjacent))
        # xNorm = opposite/magnitude
        # yNorm = adjacent/magnitude
        # myMsg = Twist()

        #Descision Plan for between two points
        #decisions = {0:stop,1:foward,2 :left,3:right,4:escape,5:rise,6:fall}
        if (myAngle > radThethaGoal): #turn right
            self.right()
        elif(myAngle < radThethaGoal): #turn left
            self.left()
        elif(radThethaGoal - self.THETHAERRORMARGIN >= myAngle <= radThethaGoal + self.THETHAERRORMARGIN )
            self.foward()
        else:
            self.escape()

        self.cmd_vel.linear = self.lin
        self.cmd_vel.angular = self.ang
        self.dronePublisher.publish(self.cmd_vel)


    def controller_cmdList(self,CmdList):
        decisions = {0:stop,1:foward,2 :left,3:right,4:escape,5:rise,6:fall}
        for cmd in CmdList:
            decisions[cmd]()
        self.cmd_vel.linear = self.lin
        self.cmd_vel.angular = self.ang
        self.dronePublisher.publish(self.cmd_vel)

    def controller_wanderer(self): #Controls to move the drone
        decisions = {0:stop,1:foward,2 :left,3:right,4:escape,5:rise,6:fall}
        if (front_range()): #if the whole array gets swampped, attempt reverse and escape.
            decisions[1]()
        if (self.stopMoving):
            decisions[0]()
        if(rangeOnSides(scan_in,0,90-SIDE_SCANS) > rangeOnSides(scan_in,SIDE_SCANS+90,180)):# printf("Turn Right");
            decisions[3]()
        if(rangeOnSides(scan_in,0,90-SIDE_SCANS) < rangeOnSides(scan_in,SIDE_SCANS+90,180)):#printf("Turn Left")
            decisions[2]()
        if(self.isStuck() && selt.stopMoving != True):#	printf("Stop");
            decisions[4]()
        self.cmd_vel.linear = self.lin
        self.cmd_vel.angular = self.ang

        self.dronePublisher.publish(self.cmd_vel)
        #Set acceleration speeds and publish to topic

    #############################Checkers for distance##########################

    def isStuck(self):
        if (self.lin.x == 0 && self.ang.y == 0)
            return True
        return False

    def front_range(self):
        i = 45
        for i in range(0, 135):
            if (array_in->ranges[i] < 0.5):return false
    	return true

    def side_ranges(self):
        pointFound = 0
        j = self.MIN_DIST
        for j in range (0, self.MAX_DIST): #(int i = min; i < max; i++){
            if (array_in->ranges[i] > 1.25):pointFound+=1
        return pointFound;

    ################################ROS Twist settings##########################

    def move_foward(self,dir):
        self.lin.x = SPEED

    def stop(self):
        self.lin.x = 0
        self.lin.y = 0
        self.lin.z = 0
        self.ang.x = 0
        self.ang.y = 0
        self.ang.z = 0

    def escape(self):
        self.lin.x = 0
        self.ang.y = TURN_SPEED

    def left(self):
        self.ang.y = TURN_SPEED

    def right(self):
        self.ang.y = -TURN_SPEED

    def drop(self):
        self.lin.z = 0.75*-SPEED

    def rise(self):
        self.lin.z = 0.75*SPEED

    ############################################################################
