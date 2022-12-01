# **RockyRobotics - Object Detection and Sorting Simulation**


## Introduction 
The purpose of the project is to be able to detect different colored blocks and sort them using a robotic simulation in descending order of color wavelength also known as rainbow format. The code developed in this repository uses a variety of different software and hardware components which are detailed below:

### Hardware required:
* RGB Camera with Intel Real Sense capabilities
* USB-C to USB cable to connect camera to computer
* Colored Blocks to sort 
* Camera Stand (optional) 

### Software Required:
* Most of the required software imports have been imported on the top of all the required files. However, make sure to pip install any of the packages such as pybullet, opencv, and pyrealsense2. More documentation of installation can be found online

Now that we understand the required components and a generalized function of the code. Let's get into more detail on what each component means.

### Block Distance Detection 
#code explanation
#hardware used
#hardware set up

### Pyrobot code 
code explanation
Limitations

Robot Simulation:
This code simulates a robot that hovers over a block, reaches down for it, touches it, and moves it over to a specified position in rainbow order.

Robot Grasp:
This code simulates a robot that hovers over a block, reaches down for it, and grips it in between the grippers. It then sets it back down in its original location.

Limitations:
We were forced to hard code the positions of the blocks for the Robot simulation because we were not able to get the robot to grab blocks at different specified locations. The positions and colors of the blocks are defined in the color_dict dictionary. We also hard coded the movements of the robot and blocks as they travel to the sorted position. The simulated robot touches down on the blocks, the block disappears and reappears at the specified location that will put the blocks in rainbow order along with the robot gripper.

Instructions:
Run Robot_Sim, Robot_Sim_Grasp_Block

### Conclusion

