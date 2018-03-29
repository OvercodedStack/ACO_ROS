#!/usr/bin/env python
import rospy
import ros
import tf
import nav_msgs
import geometry_msgs
import sensor_msgs

from std_msgs import String

class Drone:
    def __init__(self,seqTp): #Setup
        self.SPEED = 0.40
        self.TURN_SPEED = 0.55
        self.MAX_DIST = 0.5
        self.MIN_DIST = 0.45
        self.LONG_DIST = 1.00
        self.AMOUNT_OF_POINTS = 180.0



        self.laserscan = geometry_msgs.LaserScan()
        self.imu = geometry_msgs.Imu()
        self.sequence = 0           #the drone own ID number in group
        self.odom = Odometry()
        self.twistMsg = Twist()
        self.childFrame = frameID    #String
        self.path = nav_msgs.Path()
        self.goalArray = actionlib_msgs.GoalStatusArray()
        self.cmdString = ""
        ####################################### Set Topics##############
        self.sequence = seqTp

    def setValues(self):#Setup the subscribers
        droneNameLaser = "/droneLaser" + str(self.sequence)
        droneNameImu = "/droneImu" + str(self.sequence)
        droneNameOdom = "/droneOdom" + str(self.sequence)
        rospy.Subscriber(droneNameLaser,LaserScan, self.getLaser)
        rospy.Subscriber(droneNameImu,Imu,self.getImu)
        rospy.Subscriber(droneNameOdom,Odometry, self.getOdom)

    ############Callbacks###################
    def getLaser(laser):
        if laser != null:
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
    def startDrone(self):
        self.start_wandering()
        self.publish()

    def publish(self):
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

    def start_wandering(self): #Start wandering towards a location where the laserscan is furthest "empty"
        self.record_path()







    def move(self,dir):

    def escape(self):

    def moveFoward(self):

    def stop(self):

    def turn_left(self):

    def turn_right(self):

    def drop_down(self):


    def move_up(self):







    def record_path(self):
        ##########Header for path
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        #h.frame_id = self.childFrame
        self.path.header = h

        ########Data to make the pose
        p.pose = self.odom.pose()
        self.path.poses.append(p)






