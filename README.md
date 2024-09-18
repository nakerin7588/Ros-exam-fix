# FRA501 Amazing turtle

In this project we will develop a little fun turtle project using simple libraly on ubuntu  So Ubuntu (22.04 LTS is recommend) with Ros humble 

* [Ubuntu](https://releases.ubuntu.com/jammy/)
* [ROS_Humble](https://docs.ros.org/en/humble/Installation.html)

## Basic knowlegde and How to use 
first you need to clone your ptoject to your computer using command 

```
git clone https://github.com/Saifa36622/Ros_exam.git -b saifa_main_2
```
paste this in the terminal or powershell of your prefer and then use this command to navigate to the working directory

```
cd Ros_exam
```
then do hard reset for this workspace to change commit version to previous
```
git reset --hard 33c3e935743a18b7e47cc7b2e5cdf2c9ad8c65b3
```
after this you need to build the workspace by this command
```
source /opt/ros/humble/setup.bash
```
to source setup-file from ros 

then `colcon build` to build up the project 

```
colcon build
```
then
```
source install/setup.bash
```
if you dont have the colcon yet you can watch how to install and use it at [ros humble tutorials](https://docs.ros.org/en/humble/Installation.html)

<br>

and for the most of the project the commadline that we going to use is 
```
ros2 launch saifa_pack turtle_1.launch.py
```
to open the most of the project file and 
```
ros2 run saifa_pack teleop_turtle.py
```
to open tele-operation file to control the turtle in this project 

<br>

then you might have too install some additional libraly for this project 
* pyyaml for read or edit or etc .yaml

```
pip install pyyaml
```
* RQT for visualize and modify ROS parameters or watch the graph of ros communication in real-time.
```
sudo apt-get update
sudo apt-get install ros-humble-rqt*
```
To launch RQT, open a new terminal and run the following command:
```
rqt
```

# System Architect

![image](https://github.com/user-attachments/assets/710f584a-7f18-49fc-8189-dd216e7c09ee)

- **Tele-op Architect**

<img src="https://github.com/user-attachments/assets/40e46043-d574-466f-8cc4-b8a72b2b6877" alt="alt text" width="500" height="500">

![image](https://github.com/user-attachments/assets/9b6d481a-03e4-4ed1-af22-388dda36b0be)

Tha main point of this node is to be manual controller for users by input via keyboard and control the velocity and the acceleration of the 
turtle from point to point .There is going to be 2 node for this operation that are <br>
- teleop_turtle -> this node is use to recieve input from the user keyboard and move turtle in the ways user inteded
- teleop_controller -> this node is use in special occasion that is when user press clear ,we are going to ignore the input from the keyboard and use position and way point from our code calculation
  to eat all remain of the pizza
<br>
and from the upper imagge it going to indicate that these node going  to be the one that recieve input from the user ,So we internde that we are going to save the input from user to .yaml file via these node


<br>

<br>

- **Yaml converter**
  
![image](https://github.com/user-attachments/assets/76d5617f-eefb-40b6-958b-13d2e37d9c6c)

this node we create it to use to convert the pos or position input that we recieve from the user via the tele-op node that parse to this node ,this node will convert all those data to .yaml file (one of the kind of config file that easy to use)
to be use in the future
##### (Normally .yaml file will automatically generate with the folder to source this file on your desktop->ros2_yaml_files->pizza_pose.yaml) ##### 

<br>

- **Copy or cp Architect**

![image](https://github.com/user-attachments/assets/3ef36d7c-4c90-4e24-aa7c-765f96d9d33a)

All this cp having the same architect cuase it came from the same node call cp_controller than been duplicate in to 4 node by launch file ,These 4 node is use to mimick the user Behavior when press the key and duplicate that behavior so in this node is going to have the main component simiilar to the teleop node but the cmd_vel or the pose is going to be fully auto ,and the sum_callback is use to signal all the duplicate node when all of the duplicae nodeis finish doing they task 
<br>

![image](https://github.com/user-attachments/assets/ae5c9a38-7f66-40e0-ba61-3a046253ff60)

and obviosly all those node have the param in the node from the same constuct so all those param is going to connect with the topic parameter event

# Fuction available
* Teleoperation
  So how to use it you need to run this command first
  ```
  ros2 launch saifa_pack turtle_1.launch.py
  ```
  to open the turtle bot in the terminal like this

![image](https://github.com/user-attachments/assets/e99c7a4f-2d3a-4205-af78-d8b9226d213c)

there is going to be 2 terminal appear on the screen one is the on we going to use in this Tele-operation function

Then you have to open another terminal and run this command line 
```
ros2 run saifa_pack teleop_turtle.py
```
to open another terminal like this
<br>

![image](https://github.com/user-attachments/assets/78d8a584-178c-4ec0-a389-433fcce5a2c8)

So this termianl is going to be use to control the turtle on the termminal ,

### Keyblind 
- w to go forward
- s to go backward
- a to rotate left
- d to rotate right
- e to stop moving
 ### warning: If you want to control the turtle you should left a click at the Teleoperation window all the time ###
 ##### (if you cannot control the turtle try to click at the terminal agian) ##### 
  ![image](https://github.com/user-attachments/assets/2ef03283-721a-4e66-956e-099f733f3c49)
  So we done the Tele-op base on that 
![image](https://github.com/user-attachments/assets/76410962-07cf-4850-90f3-2f5e63005003)
this Tele-op are going to be a node that going to input the pos or posiotion in the present of the t_name (turtle name)
and then control the velocity or acceletion of the turtle when it going to move to another pos  and we add a little help from 
the control parameter ,So the code that control the spped is going to look like tihs 

![image](https://github.com/user-attachments/assets/010c5b30-ad02-40df-afa3-d3f077fb1fa2)

and we use a little bit pythagoras theory to find a Distance betweeen point and robot 

* Spawn pizza
  Just like when you using tele-op you need to run this command first
  ```
  ros2 launch saifa_pack turtle_1.launch.py
  ```
  and then this command 
  ```
  ros2 run saifa_pack teleop_turtle.py
  ```
  #### Keyblind 
  - r to spawn a pizza
  
  
![image](https://github.com/user-attachments/assets/dfe7a7c3-281c-4960-ab8d-965bb2e7e4f2)


So to spawn a pizza from the turtle position we need get the live position of the turtle and then call the service /spawn_pizza to spawn the pizza like the image below

  ![image](https://github.com/user-attachments/assets/9fc5204a-9db9-425e-963f-5cabef103b8c)

* Save and Clear <br>

   Just like when you using some above, you need to run this command first
  ```
  ros2 launch saifa_pack turtle_1.launch.py
  ```
  and then this command 
  ```
  ros2 run saifa_pack teleop_turtle.py
  ```
  #### Keyblind 
  - u to save the pizza position (to .yaml file) 
  - c to clear all the pizza( the turtle will eat all the pizza that not been save)

  when you successly save this log going to show up to indicate that save have been complete 
  
  ![image](https://github.com/user-attachments/assets/1f360022-06da-47ec-b175-7f4b5e70ee7b)
  
  <br>

   when you press clear to clear all the unnessecerry pizza

  ![image](https://github.com/user-attachments/assets/512ed4b3-0df9-4aff-8122-d6d55e992c91)

  the turtle going to start eating all the pizza
* Modify ROS Parameters Using RQT. In this project we have main parameter is for pizza node call max_pizza that tell we how much pizza can generate on 1 turtle ana other parameter that we can also use are controller gain of turtles.

  Once RQT is launched, follow these steps to visualize and modify the parameters:

  1.Open the Parameter Plugin:
  In RQT, go to the Plugins menu at the top, navigate to Configuration → Dynamic Reconfigure or Parameter Reconfigure. This will open the Parameter window, where you can view and modify the active parameters in your system.

![image](https://github.com/user-attachments/assets/961c00e3-1b6a-46a7-b8ca-d31a41fc7593)
![image](https://github.com/user-attachments/assets/843fe3f6-618d-462b-bf7b-92f50000a681)

  2.Select the Node:
  On the left panel of the Parameter window, you will see a list of active nodes. Select the node whose parameters you wish to modify.

![image](https://github.com/user-attachments/assets/b741c3b5-3194-48f9-9cd6-237be911a5ec)

  3.Edit Parameters:
  Once you select the node, its parameters will be displayed. To modify a parameter, simply click on the value field next to the parameter name and enter the new value.

![image](https://github.com/user-attachments/assets/33f5bdc6-c8af-476e-bd97-95b30e314980)

  4.Apply Changes:
  After modifying the parameter, press Enter to apply the change. This will update the parameter in real-time on the running node.

  
  5.Check parameter:
  if you want to check the parameters that you are modify are update.
  ```
  ros2 param list
  ```
  to check the list of parameters
  ```
  ros2 param get <node_name> <parameter_name>
  ```
  to check the value of parameter
* Launch file and Luanch file argument <br>
  ```
  ros2 launch saifa_pack turtle_1.launch.py
  ```
  for change the name of turtle teleop via launch file use this command instead.
  ```
  ros2 launch saifa_pack turtle_1.launch.py turtle1_name:=<turtle_teleop_name>
  ```
  Example:
  ```
  ros2 launch saifa_pack turtle_1.launch.py turtle1_name:="InwZa"
  ```
  with this command above you will get the turtle teleop name "InwZa".
  <br>
   <br>
  ![image](https://github.com/user-attachments/assets/9b35b9e4-46b5-4389-82a9-3c767b5e162d)

  launch file is use to run a lot of file in one comand line such as in this project we use lauch file name `turtle_1.launch.py` in the saifa_pack (package) and in this lauch file consisting of file or executeing file such as

  - turtlesim_plus_node -> the turtle sim we use ( the black background when terminal open)
  - execute command line "spawn" to spawn turtle
  - teleop_controller -> to control the teleop turltle when user want to clear
  - pizza_viapoint_script -> to convert the pose data to .ymal file
  - turtlesim_plus_node that have a namespace cp -> to be another terminal for cp bot
  - for loop that going to loop 4 time to spawn turtle and create node controller for turtle_name "Foxy","Noetic","Humble","Iron"
  - sum_controller -> to be a signal node for all cp bot to move when finish at the same time
  - start_copy_script -> to be the node that going to read from .yaml file that been save and send send the right data to all cp bot 

 




