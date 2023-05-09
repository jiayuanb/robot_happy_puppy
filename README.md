# Happy Puppy

## Team members

| Name  |
| ------------- | 
| Yijie Du  | 
| Zhengwei Yu   | 
| Yiping Shi  | 
| Jiayuan Bi   | 

## Project Description

The development of quadruped robots, is driven by the goal of creating machines that can effectively traverse through intricate and difficult terrains.

### Task 1: Walking

The walking task's goal is to design a controller that allows the robot to walk forward and backward with a speed of  0.5m/s.

Walking sideways with a speed of  0.5m/s.

Turning in place with the yaw speed of 0.5 rad/s.

### Task 2: Running on flat ground

The running task’s goal is to control the robot to run on flat ground for 10 meters.

### Task 3: Stair Climbing

This task’s goal is to let the robot walk/run on 5 consecutive stairs with a stair rise of 10cm, a stair run of 20 cm, and a stair width of 2 m.

### Task 4: Obstacle course

This task’s goal is to control the robot to navigate through the obstacle course(shown below)without collision between the robot's body, legs, obstacles, and terrain. 

![alt text](https://github.com/jiayuanb/robot_happy_puppy/blob/main/task4_obstacle.png?raw=true)

## General Approach 

The general approach for the project involves the dynamic model of the quadruped robot, the MPC controller for the rigid body dynamics, the PD controller for leg swing, and the gait controller for different movement phases of the robot. These will all be executed by a function called the coordinator, which counts the running time and specifies the execution time for each function. The listed are the general steps approaches for this project:

1. Set up Simulink and its environment, ensure all the contact forces are added, and create ports for input and output.

2. Design the PD control for standing up, allowing the robot to stand back up from the ground. (QP control was implemented for this function to improve performance and accuracy)

3. Design the MPC control for the rigid body dynamics controlling the robot trunk of the robot to desired states.

4. Design the PD controller for leg swing, which will be used to control the leg’s motion by initially giving a predicted trajectory, which will translate to desired positions and velocity for PD control.

5. Design a coordinator. The coordinator will be implemented after the robot has completed the standing phase. A world clock will be used as the coordinator to arrange all functions. Different controllers will be executed depending on the current time to perform the tasks.

6. Debug and tuning, after establishing all the required functions, they will be merged to debug, ensuring that everything is working relatively, as many function calls are performed.  Then, tuning of the controller and tuning of the desired system state will be done, in order to achieve optimal performance. 
