# End to end motion planner using Deep Deterministic Policy Gradient (DDPG) in gazebo

The goal is to use deep reinforcement learning algorithms: Deep Deterministic Policy Gradient (DDPG) to control mobile robot(turtlebot) to avoid obstacles while trying to arrive a target.

Goal: Let robot(turtlebot) navigate to the target(enter green circle)

![image](https://github.com/m5823779/MotionPlannerUsingDDPG/blob/master/demo/demo.gif)
Demo video (Speed up ten times )

### Introduce 

With the rapid development of technology, more and more service robots appear in our daily lives. The key technologies of service robots span many fields. The key technologies include the following: mobile navigation, system control, mechanism modules, vision modules, voice modules, artificial intelligence, and other related technical fields. Among them, navigation is the core technology of service robots. Therefore, this research will focus on the development of indoor robot navigation.

In this project, we present a learning-based mapless motion planner by taking the sparse laser single and the target position in the robot frame (relative distance and relative angles) as input and the continuous steering commands as output. This saves us from using traditional methods such as "SLAM" to have maps and can also do the navigation. The trained motion planner can also be directly applied in environments which it never seen before. 

Input(State):
1) Laser finding (10 Dimensions)
2) Past action (Linear velocity & Angular velocity) (2 Dimensions)
3) Target position in robot frame (2 Dimensions)
      a. Relative distance
      b. Relative angle (Polar coordinates)
4) Robot yaw angular (1 Dimensions)
5) The degrees to face the target i.e.|Yaw - Relative angle| (1 Dimensions)
  
     Total: 16 Dimensions


Normalize input(state):
1) Laser finding / Maximum laser finding range
2) Past action (Orignal)
3) Target position in robot frame    
    - Relative distance / Diagonal length in the map
    - Relative angle / 360
4) Robot yaw angular / 360 
5) The degrees to face the target / 180


Output(Action):
1) Linear velocity (0~0.25 m/s) (1 Dimensions)
2) Angular velocity (-0.5~0.5 rad/s) (1 Dimensions)


Reward:
- Arrive the target: +120
- Hit the wall: -100
- Else: 500*(Past relative distance - current relative distance)


Algorithm: DDPG (Actor with batch normlization Critic without batch normlization)


Training env: gazebo

### Installation Dependencies:

Python3

Tensorflow

``
pip3 install tensorflow-gpu
``
ROS Kinetic

> http://wiki.ros.org/kinetic/Installation/Ubuntu

Gazebo7 (When you install ros kinetic it also install gazebo7)

> http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0


### How to Run?
```
cd
mkdir catkin_ws && mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/m5823779/MotionPlannerUsingDDPG.git project
git clone https://github.com/m5823779/turtlebot3
git clone https://github.com/m5823779/turtlebot3_msgs
git clone https://github.com/m5823779/turtlebot3_simulations
cd ..
catkin_make
```
And add following line in ~/.bashrc
```
export TURTLEBOT3_MODEL=burger
source /home/"Enter your user name"/catkin_ws/devel/setup.bash
```

Then enter following command in terminal
```
source ~/.bashrc
```


Demo:
First run:
```
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch
```
In another terminal run:
```
roslaunch project ddpg_stage_1.launch
```
_______________________________________________________

Train:
If you want to retrain yourself change the setting

```
is_training = True   # In: project/src/ddpg_stage_1.py
```

### Reference:

Idea:

https://arxiv.org/pdf/1703.00420.pdf

Network structure:

https://github.com/floodsung/DDPG

Ros workspace:

https://github.com/ROBOTIS-GIT/turtlebot3

https://github.com/ROBOTIS-GIT/turtlebot3_msgs


https://github.com/ROBOTIS-GIT/turtlebot3_simulations

https://github.com/dranaju/project
