# ACO_ROS
This is a work in progress ACO algorithm built for use in automated driving using virtual or real robots. At the present moment this package contains a barebones framework for working with virtual drones and their control in a 2D plane (3D pathfinding will be a future planned feature. Or consider forking to implement in other projects).  

While there are some variants of ACO, this project is inspired by the book written by Marco Dorigo and Thomas Stützle. If you have access to th e book via your institution or other means, please give it a read to understand the concept. https://mitpress.mit.edu/books/ant-colony-optimization . While this package uses some parts and logic from ACO, it is important to realize that there are some parts still in development as the project becomes more avaliable.   


**---Driving a robot---**  
(coming soon in a future update)    

**---Virtual drones---** (HOLO-lens or ROS-Compatible AR devices ONLY. MUST create drones in a Unity space.)  

The way that this works is that ROS is able to pick up the data from a N amount of drones created in a Unity virtual game space. Each drone would then be driven around with this ROS package by sending twist messages and reading in data from each drone. Each drone will have a script install that will read in laserscan, IMU, and Odometry data. Each drone will then post/read data from unique individual subscriber and publisher nodes. The twist data is sent back to Unity which will drive the drone around in its virtual space.   

In addition to a virtual space, an active-map should be generated at the same time that the robot is being run in order to create a map 


#ID of the drone = Sequential increment of the drone number.   
e.g if you had 5 drones, you would name them 1, 2, 3, ... 5.   
(Virtual drones Setup)   

**Subscribers:**
/droneLaser_(#ID)  
/droneOdom_(#ID)   
/droneImu_(#ID)   
/map  

**Publishers:**
/ACOpath  
/ACOcmd  
/drone_(#ID)   

**Transforms:**   
/DRONE_(#ID)   




