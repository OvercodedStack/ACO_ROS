#!/usr/bin/env python
import rospy
import ros
import rospy
import tf
from nav_msgs import OccupancyGrid
from nav_msgs import MapMetaData
from geometry_msgs import PoseWithCovarianceStamped
from move_base_msgs import MoveBaseActionGoal
from actionlib_msgs import GoalID
from ant_colony_optimizer import mapHandler
from ant_colony_optimizer import Drone

#Independent multi-mesh drone/robot controller with ACO
#Will seek out a amount of drones to be used for


class ACO:
#Function to set the map
    def callForMap(myMap):
        self.map = myMap
        return "map loaded"
#Function to set Metadata
    def callForMetadata(data):
        self.metadata = data
        output = "null"
        if data.width == 0 and data.height == 0:
            output = "No data"
        else:
            output = "Data collected."
        return output
#Set mypose
    def callFinalPose(mypose):
        self.pose = mypose
        return "Done setting pose"
#Run main
    def main(self):
        running = True
        runTime = 0
        while (running):
            for drone in self.droneArray:
                drone.startDrone()
            if runTime == self.NUMBER_OF_ILLETERATIONS:
                running = False

        #point_array = mapHandler.locate_plains(self.map,self.metadata.width,self.metadata.height,5)

#Constructor
    def __init__(self):
        #Declare the variables required for the program
        self.map = OccupancyGrid()
        self.metadata = MapMetaData()
        self.pose = PoseWithCovarianceStamped()
        self.droneArray = []
        self.NUMBER_OF_ILLETERATIONS = 1000
        self.pheromonePath = []

        # self.pub_msg = MoveBaseActionGoal()
        # self.cancel = GoalID()
        # stampT = rospy.Time.now()
        # self.cancel.stamp = stampT
        # self.cancel.id = "cancel"

        ##########################Drone Objects#############################
        self.myTopics = [[]] #Expect a double array in rospyPubList
        self.myTopics = rospy.get_published_topics()
        matching = sort([element for element in self.myTopics if "/droneLaser" in element])
        droneNumber = len(matching)  #Match the amount of drones to the amount of laserscans we found
        i = 1 #counter
        while (i != droneNumber): #Instantciate an array of drone Objects
            self.droneArray[i] = Drone(i,"drone_"+str(i))
            i += 1

        #################Subscribers for maps ##############################
        rospy.Subscriber("/map",OccupancyGrid, self.callForMap)
        rospy.Subscriber("/map_metadata",MapMetaData,self.callForMetadata)
        rospy.Subscriber("/initialpose",PoseWithCovarianceStamped, self.callFinalPose)

        ####################################################Run calcuations
        self.main()

        # self.pubGoal_ = False
        # self.pubCancel_ = False
        # self.pubPath_ = False
        #Publishers
        #pubGoal = rospy.Publisher("/move_base_simple/goal",GoalStatusArray,queue_size=10)
        #pubCancel = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        #pubPath = rospy.Publisher("/ACOpath",Path,queue_size=10)

        #Publishing posting
        #while not rospy.is_shutdown():
            # if self.pubGoal_:
            #     pubGoal.publish(self.pub_msg)
            #     self.goal = False
            # if self.pubCancel_:
            #     pubCancel.publish(self.cancel)
            #     self.pubCancel_ = False
            # if self.pubPath_:
            #     pubPath.publish(self.)
            #     self.pubPath_ = False

#Trigger
if __name__=='__main__':
    rospy.init_node('Ant_Colony_Wrapper')
    try:
        main_prog = ACO()
    except rospy.ROSInterruptException:
        pass

