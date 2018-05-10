#!/usr/bin/env python
import rospy
from Drone import Drone
class ACO:
    # Constructor

    def __init__(self):
        # Declare the variables required for the program
        self.droneArray = []
        self.NUMBER_OF_ILLETERATIONS = 1000
        rospy.loginfo("Starting aco_ros...")
        myDrone = Drone(1, "drone_1", True, False, "/myDrone")
        myDrone.start_wandering(self.NUMBER_OF_ILLETERATIONS)

# Trigger
if __name__ == '__main__':
    rospy.init_node('Ant_Colony_Wrapper')
    try:
        main_prog = ACO()
    except rospy.ROSInterruptException:
        pass

        ##########################Drone Objects#############################
        # myTopics = rospy.get_published_topics()
        # droneNumber = self.locateDrones(myTopics)
        #################Subscribers for maps ##############################

        # rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.callFinalPose)
        # rospy.Subscriber("/goal", Point, self.callPoint)
        ####################################################Run calcuations

        # if (droneNumber != 0):
        #     i = 0  # counter
        #     while (i != droneNumber):  # Instantciate an array of drone Objects
        #         self.droneArray.append(Drone(i, "drone_" + str(i), True, False,
        #                                      "/myDrone"))  # Set drone name and set the sensors that work at the moment
        #         rospy.loginfo("Found drone" + str(i + 1))
        #         i += 1
        #     self.main()
        # else:
        #     rospy.loginfo("No drones to spawn!")

    # def locateDrones(self, myList):
    #     numDrones = 0
    #     for myArray in myList:
    #         s = myArray[0]
    #         if (s.find("/droneLaser") == 0):
    #             numDrones += 1
    #     return numDrones
