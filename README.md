# **RockyRobotics - Object Detection and Sorting Simulation**


## Introduction 
The purpose of the project is to be able to detect different colored blocks and sort them using a robotic simulation in descending order of color wavelength also known as rainbow format. The code developed in this repository uses a variety of different software and hardware components which are detailed below:

### Hardware required:
* RGB Camera with Intel Real Sense capabilities
* USB-C to USB cable to connect camera to computer
* Colored Blocks to sort 
* Camera Stand  

### Software Required:
* Most of the required software imports have been imported on the top of all the required files. However, make sure to pip install any of the packages such as pybullet, opencv, and pyrealsense2. More documentation of installation can be found online

Now that we understand the required components and a generalized function of the code. Let's get into more detail on what each component means.

### **Block Distance Detection** 
The code for block detection uses the following files to detect the distance between each block from where the camera is and uses a mathematical approch to figure out the x,y,z distance that has been inputted in. One important note here is that the z value id harcoded in at a certain value depending on the camera height as we had access to only one camera that could determine 2 axes with good accuracy.

The files used are as follows:

* Color_dictionary_text
* detect_distance.py
* distance_with_colordict.py
* realsense_depth.cpython-38.pyc
* realsense_depth.py
* **All of the files are stored in detection_files_2 and can be downloaded directly**

In this folder the main file to run is the detect_distance file which contains the imports and functions needed. 

**NOTE: on the top of this file there is a variable for z_val, change that as needed in mm depending on the camera height**

**Additional Note: Place the camera on the first block and make sure all blocks are within the frame**

Connect the camera to the computer and run the detect_distance file. It should pull up the camera with a black dot and varying distances. 

**NOTE: Due to the fluctuations in vision, the mathematical output expects an increasing distance as each block is scanned in order. If the detected distance is less than the previous one, then it does not detect the distances**



### **Robot Arm Simulation** 
There are two different files for the robot arm simulation:

* Robot_Sim
* Robot_Sim_Grasp_Block

#### Robot_Sim:
This code simulates a robot that hovers over a block, reaches down for it, touches it, and moves it over to a specified position in rainbow order.

#### Robot_Sim_Grasp_Block:
This code simulates a robot that hovers over a block, reaches down for it, and grips it in between the grippers. It then sets it back down in its original location.

#### Limitations:
For testing purposes, the file `color_dicitionary_text.py` with a predermined position of the blocks was created. The robot is coded to organize in rainbow order 5 blocks of colors: Red, green, blue, yellow, and orange.
The simulated robot touches down on the blocks, the block disappears and reappears at the specified location that will put the blocks in rainbow order along with the robot gripper.
Instructions:
Run Robot_Sim, Robot_Sim_Grasp_Block



