Learning Skills
==========================
[**Project Website**](https://egalbally.github.io/LearningRobotSkills/)

This project contains the code required for a 7-DOF Panda robot to autonomously perform tasks involving 6 primitives: free space motion (FSM), make contact, align two surfaces, engage threads, screw, tighten. If a failure occurs, a user can intervene and haptically control the robot. This achieves two goals: (a) enables task completion, (b) allows us to collect data from the recovery strategies that can then be used to enhance the autonomous capabilities of the robot.

![projectDiagram](https://github.com/egalbally/LearningRobotSkills/blob/master/readme_imgs/learningSkills_diagram.png)


## How to run the code:
Inside launch_scripts, there are scritps to run each of the elements in the system: 

### redis communication
- server
- client

### hardware drivers
- optitrack motion capture
- ATI F/T sensor
- haptic device driver
- allegro hand 
- panda arm 

### controllers
- robot controller 
- haptic device controller 

### data collection
- logger and plotter (interface)
