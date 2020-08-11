 Robots_Simulator
 ==
 The copyright of this repository and simulator belongs to Networked Robotic Systems Laboratory, The Pennsylvania State University.
 <br>
 **All Rights Reserved**
 <br>
 
 Install The Package
 --
 First, you need create a ROS simulation workspace:(You can name it whatever you want. Here, we call it _robots_simulator_) 
 <br>
 You are highly recommended to call it _robots_simulator_, since all the following steps are based on the assumption that the workspace is named _robots_simulator_
 ```
 $ mkdir robots_simulator
 ```
 Then, we need to install this package into your simulation workspace:
 ```
 $ cd robots_simulator
 $ git clone https://github.com/Shicheng-Liu/Robots_Simulator.git
 ```
Since this repo is private, you can download this repo directly from this website if you have trouble running _git clone_ command.
<br>
After you download this repo, you can find a new folder named _Robots_Simulator_ in your simulation workspace _robots_simulator_. 
<br>
**IMPORTANT**: You need to change the name of this folder from _robots_simulator to _src_ 
<br>
After that, our terminal is still in this simulation workspace. If your terminal is not in the workspace, please cd to this workspace: _cd robots_simulator_. Then, run:
```
$ catkin_make
```
Now, we need to add the path of this workspace into your environment:
```
$ gedit ~/.bashrc
```
Please scroll down to the bottom and add this line:
```
source ~/robots_simulator/devel/setup.bash
```
Save and exit. 
<br>
Now, all the installation steps are over.

Use the simulator
-
In this package, I have written four launches to launch different numbers of robots. If you want to spawn more robots, please go to _robots_simulator/src/iRobot_create/launch_ to see how launch files are wriiten. You can write a launch file satisfying your requirements quickly after you read these already existing launch files.
<br>
<br>
Here, we have four launch files to spawn 1~4 robots. You can choose one of the following four commands depending on how many robots you want to spawn:
```
$ roslaunch iRobot_create one_robot.launch
```
```
$ roslaunch iRobot_create two_robots.launch
```
```
$ roslaunch iRobot_create three_robots.launch
```
```
$ roslaunch iRobot_create four_robots.launch
```
In _robots_simulator/src/iRobot_create/src_, I have already written the basic controller of all the four robots in _controller.py_.
<br>
To write a python script to implement your algorithm, you can use this controller by adding
```
from controller import BasicRobotController
```
at the top in your code.
<br> 
<br> 
I will introduce two functions of the controller to help you start:
<br>
```
Function: robot1(), robot2(), robot3(), robot4()
This kind of functions can get the pose of the robots. 
Take robot1() as an example:
robot1() will return 7 values: x,y,z coordinate and the quaternion of robot1. 
```

```
Funtion: SetCommand1(), SetCommand2(), SetCommand3(), SetCommand4()
This kind of functions can controll the movement of the robots
Take SetCommand1() as an example:
This function has two inputs: SetCommand1(velocity, angle)
Since this kind of robots can only move forward or backword, you need to change its angle if you want it 
to move right or left or other directions
```
I know it is kind of confusing to you because of my poor expression skill.
<br> You can go to the file _irobot.py_ which is also in _robots_simulator/src/iRobot_create/src_ to see how to exactly controll the robots.
<br>

Simple Presentation
-
Here are two videos to help to get an idea of how to use it quickly.
<br>
First, we need to spawn four robots:
```
$ roslaunch iRobot_create four_robots.launch
```
Then, we can run the template just mentioned:
```
$ rosrun iRobot_create irobot.py
```
and you can see the four robots following their own trajectory simultaneously.

![image](https://github.com/Shicheng-Liu/Robots_Simulator/blob/master/move_simultaneously.gif)

They are controller by the function _move()_ in _irobot.py_
<br>
<br>
Or, they can also move one by one.

![image](https://github.com/Shicheng-Liu/Robots_Simulator/blob/master/move_respectively.gif)

They are controller by the functions _move1()  move2()  move3()  move4()_ respectively.


Expectations
--
The first version of this simulator is created in September, 2019 with only the most fundamental functions. Hope this simulator will be cooler and fancier in the future by all you guys.
<br>
And great thanks to [Networked Robotic Systems Laboratory](http://php.scripts.psu.edu/muz16/index.php) to give me this opportunity to do what I am interested in.

