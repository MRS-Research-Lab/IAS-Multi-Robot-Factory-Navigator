# IAS-Multi-Robot-Factory-Navigator
The project is a collaborative effort between the Multi-Robot Systems (MRS) research group in Egypt and the Institute of Automation and Software (IAS) lab at the University of Stuttgart, Germany. It aims to develop a ROS-based architecture for two TurtleBot3 units equipped with Open Manipulator setups designed to navigate the IAS factory.

The primary goal is to control the TurtleBots to follow a sequence of waypoints to perform pick-and-place operations efficiently while avoiding both static and dynamic obstacles utilizing onboard LIDAR sensors. Additionally, the onboard cameras enable the manipulators to identify and handle colored objects, streamlining the pick-and-place tasks. This project showcases advanced multi-robot coordination and navigation within a factory setting, emphasizing obstacle avoidance and precise object manipulation.

The project folders are divided into three groups:
- IAS Lab Simulation Environment 
- Turtlebot Navigation
- Open Manipulator Object Handling

# IAS Lab Simulation Environment
This simulation environment is designed on Gazebo simulator using a mobile phone 3D laser scanner application to scan the IAS lab sturcture and save the created file as world file to be launched on Gazebo simulator. In addition, a turtlebot3 robot equipped with open manipulator is spawned in the created environment for navigation purposes.  

## 1. Download Needed File
Download the [`ias_environment.zip`](ias_environment.zip) File

Open a new terminal using `CTRL` + `ALT` + `T` or from the applications pane and run this command
```bash
sudo apt install unzip
cd ~/Downloads
unzip ias.zip -d ~/Downloads
```

## 2. Move the Files
Run the following Commands
```bash
mkdir -p ~/.gazebo/models/IAS/meshes
cd ~/Downloads
mv model.config model.sdf ~/.gazebo/models/IAS
mv model.dae Image_0.jpg ~/.gazebo/models/IAS/meshes
mv turtlebot3_ias.launch turtlebot3_ias_gripper.launch ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch
mv IAS.world ias_empty.world ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
mv turtlebot3_manipulation_gazebo_ias.launch ~/catkin_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/launch
```

## 3. Run the Simulation
Running it with manipulator
```bash
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo_ias.launch
```

Running it without manipulator
```bash
roslaunch turtlebot3_gazebo turtlebot3_ias.launch
```
# Turtlebot Navigation
For turtlebot control and navigation, specific picking and depot locations are set for each robot. The control algorithm applied is the Point-to-Point control method to handle the robot's linear and angular velocities to reach the desired way point with accurate position and orientation. To avoid static and dynamic obstacles in the region, Artificial Potential Field (APF) method is adopted. Further details on the setup and launching the control and navigation architecture are discussed in the upcoming points.

## 1. PC and SBC Setup 
In order to build the turtlebot navigation framework, PC setup and Raspberry Pi (SBC) setup should be performed first through ['PC setup'](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/). It is worth mentionning that the project's architecture is built using ROS Noetic version, Ubuntu 20.04. 

# Acknowledgments
Gratitude is extended to Prof. Dr. Ing Andrey Morozov and Eng. Philipp Grimmeisen, M.Sc.; IAS director and member, and Assoc. Prof. Dr. Omar M. Shehata; MRS director for providing this incredible opportunity. Appreciation is also conveyed to the MRS team for their dedication and hard work in meeting the project requirements. Their efforts were crucial to the project's success.

![IAS-MRS_Team](https://github.com/user-attachments/assets/483af2e4-7ee1-4e0d-8afc-b3a7519784ad)
