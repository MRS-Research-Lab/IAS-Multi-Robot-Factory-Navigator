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
For turtlebot control and navigation, specific picking and depot locations are set for each robot. The control algorithm applied is the **Point-to-Point control** method to handle the robot's linear and angular velocities to reach the desired way point with accurate position and orientation. To avoid static and dynamic obstacles in the region, **Artificial Potential Field (APF)** method is adopted. Further details on the setup and launching the control and navigation architecture are discussed in the upcoming points.

## 1. Setup 
It is worth mentionning that the project's architecture is built using **ROS Noetic version, Ubuntu 20.04**. 

In order to build the turtlebot navigation framework, PC setup, Raspberry Pi (SBC) and OpenCR setup should be performed first through [`PC, SBC and OpenCR Setup`](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/). 

For manipulation, ensure that the setup steps included through [`Manipulation Setup`](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation) are performed.

## 2. Launching the System Architecture
Download the [`turtlebot_architecture.zip`](turtlebot_architecture.zip) File

**PC Terminal**: Open a new terminal using `CTRL` + `ALT` + `T` and run the following to call the ROS Master on your machine:
```bash
roscore
```

**SBC Terminal**: Open a new terminal using `CTRL` + `ALT` + `T` and run the following:
```bash
ssh@ip_address_of_the_turtlebot
```
This line is used to access the turtlebot Raspberry Pi. Change `ip_address_of_the_turtlebot` to the ip address number of the desired turtlebot to navigate. Make sure that the `ROS_MASTER_URI` and `ROS_HOSTNAME` are set correctly on both the PC and SBC through `~/.bashrc`.

In this terminal, run the following lines to bringup the turtlebot:
```bash
export TURTLEBOT3_MODEL=${TB3_MODEL}
export LDS_MODEL=${LDS_MODEL}
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
Change `TB3_MODEL` to the model we are using; in our case `waffle_pi`. Change `LDS_MODEL` to the type of LIDAR sensor used; in our case `LD-02`.

**PC Terminal**: Open a new terminal using `CTRL` + `ALT` + `T` and run the following lines to move the turtlebot to the desirable goal positions:
```bash
export TURTLEBOT3_MODEL=${TB3_MODEL}
export LDS_MODEL=${LDS_MODEL}
roslaunch turtlebot_architecture Turtlecontrol.launch
```

`Turtlecontrol` is a launch file which includes `commands.py` and `Bot.py` which are python scripts that facilitates the communication between the factory and the robots which is guaranteed using MQTT communication network. The goal points of each robot are set in 2D array inside the node script indicating the X, Y positions of the robot and the desirable orientation of the robot assigned in `Bot.py` file. These points are measured from the robot initial frame. For each robot there exist an array indicating its goal positions. 

Moreover, the launch file includes the `P2P.py` node which is responsible to control the robot's motion using **Point-to-Point control**. It subscribes the goal points sent by the MQTT signals and the actual pose of the robot from the `odom` topic and publishes the linear and angular controlled velocities for the robot to move. 

Finally, the launch file includes the `APF.py` node which is responsible for obstacle avoidance method using the **Artificial Potential Field (APF)** approach. The static obstacles locations are obtained by gmapping in Gazebo prebuilt environment, while the dynamic obstacle locations are acquired by the onboard LIDAR sensor. The APF updates the location and orientation of the robot based on computed attractive (towards the goal) and repulsive (away from obstacle) forces. For further illustration of the algorithm, kindly visit the following link: [`APF Algorithm`](https://drive.google.com/open?id=1TWZQeathktpwSYaO6owYp2_ohNy_oGNd).

A color detection algorithm is present in the src folder which depends on HSV color scale to detect the color of the handled object without being affected by the light.

# Open Manipulator Object Handling
To be able to control the robotic manipulator to handle the object, pre-defined commands for the open manipulator are used utilizing the move_group commands [`move_group`](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation)

## 1. Setup
For manipulation, ensure that the setup steps included through [`Manipulation Setup`](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation) are performed.

## 2. Manipulator Control Architecture
Download the [`manipulator_control.zip`](manipulator_control.zip) File

**PC Terminal**: Open a new terminal using `CTRL` + `ALT` + `T` and run the following lines to control the manipulator:
```bash
roslaunch manipulator_control Manipulator_Controller_Launcher.launch
rosrun manipulator_control Manipulator_Goal_Points_Turtlebot1
```
`Manipulator_Controller_Launcher` is a launch file that includes the bringup file to access the manipulator. 
`Manipulator_Goal_Points_Turtlebot1` or `Manipulator_Goal_Points_Turtlebot2` are python nodes that are used to send the required joint angles and gripper's motion for the manipulator. The nodes subscribe from `/Camera_msg` and `MQTT_msg` to subscribe the color of the object and the current status of the manipulator. The MQTT script is used to send the manipulator the commands for grasping and releasing the object. The nodes publishes to  `/Manipulator_JointSpace_Goal` to pulish the required joint angles and `/Manipulator_Gripper_Open_Close' to publish the required boolean expression for closing and opening the gripper. 

# Extra Material 
Several videos testing the architecture are recorded and present through [`Media`](https://drive.google.com/file/d/1wLJwq8hGhBpHKh4v2OiO7iuZHSlS__et/view?usp=sharing). In addition, a presentation illustrating the whole process is present though ['Final_Presentation.ppsx'](final_presentation.ppsx).

# Acknowledgments
Gratitude is extended to Prof. Dr. Ing Andrey Morozov and Eng. Philipp Grimmeisen, M.Sc.; IAS team, and Assoc. Prof. Dr. Omar M. Shehata; MRS director for providing this incredible opportunity. Appreciation is also conveyed to the MRS team for their dedication and hard work in meeting the project requirements. Their efforts were crucial to the project's success.

![IAS-MRS_Team](https://github.com/user-attachments/assets/483af2e4-7ee1-4e0d-8afc-b3a7519784ad)
