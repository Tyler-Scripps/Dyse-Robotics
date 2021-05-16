# Rofous
This repo is dedicated to the development of an autonomous quadcopter.

The goal is to design a stealth companion that follows instructions to survey and track objects. This would be great for filming outdoor activities of all sorts, the drone must be able to move with speed and accuracy as to not loose its target. In addition the drone must have advanced CV recognition abilities and robust control/planning features. This repository is organized to help develop and test the avionics of the aerial device as well as resources for design.

### Requirements
  
  ROS, Gazebo, Python3.8=<, Ubuntu 18.04

# Install

First clone the repo: 

    git clone https://github.com/mithellscott/Rofous

Then cd into the project and run the setup script

    cd Rofous
    ./setup.bash
        	Usage and Hints:

      [Option]	 Description

	-e	 specifies the path to the python3 environment for this project
	-n	 decides the name of the python env
	-p	 declares the name of the project (required)
	-g	 specifies the extension of a git repo to attach to the project's root (make sure to update .gitignore before)
	-o	 specifies file path to load the above parameters from
	-b	 will build the project with catkin build
	-h	 prints this help message (program will exit after)
    
Now the project has an alias to load the tools in .bashrc run those like 

    source ~/.bashrc
 
    
Now your python enviroment is configured and your catkin_ws should be initialized you can check that this was successful by 

    echo $PYTHON_PATH
    catkin build
    
The simulation is runable with roslaunch
    
    roslaunch rofous_gazebo main.launch
    
    
You can run the python version with 

    cd Python3_Notebooks
    jupyter notebook
