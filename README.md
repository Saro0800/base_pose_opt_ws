# Mobile manipulator base placement optimization using an ellipsoidal reachability model

## Table of contents
+ [Introduction](#introduciton)
+ [Installation](#installation)
+ [Documentation](#documentation)
+ [Simulation tutorial](#simulation_tutorial)
+ [Cite us](#cite_us)

## Introduction
This repository provides tools and resources for **optimal base placement of mobile manipulators** using a compact **ellipsoidal model of their reachability space**. 

A first tool allows to **compute the reachability map of a manipulator** starting from its kinematic model. Actually, this first tool is an extension of the one available at https://github.com/Saro0800/Robotic-manipulators-reachability-space-modeling. \
For each reachable point, a **reachability measure** is assigned by evaluating multiple end-effector orientations through inverse kinematics and collision checking. This process yields a dense representation of the robot’s dexterity across its workspace.

Starting from this reachability distribution, **a mathematical model based on two concentric ellipsoids is obtained**. These ellipsoid equations describe the region of the workspace where the manipulator exhibits the **highest level of dexterity**, and their parameters are identified through a dedicated **optimization problem**. The reachaiblity distribution and the corresponding ellipsoids are visualized below.

<div align="center">
    <img src="./images/reachability_map.png">
</div>

This ellipsoidal model is then exploited in a **second optimization problem**, designed to autonomously determine an optimal **mobile base pose** to reach a target end-effector position. The approach simultaneously ensures:
* high manipulability of the end-effector,
* correct orientation of the robot toward the target,
* and **reduced collision risk** with the surrounding environment.

<div align="center">
    <img src="./images/opt_base_poses.png">
</div>


Different optimization algorithms can be used to solve the optimization problems, allowing users to balance convergence time and optimality depending on application requirements.

<!-- **Documentation of each module can be found inside each specific folder.** -->

The code contained in this repository has been used for the experimental evaluation in the paper *“Mobile Manipulator Base Placement Optimization Using an Ellipsoidal Reachability Model.”*

**Keywords**: Mobile Manipulators, Base Placement Optimization, Reachability Space, Ellipsoid Modeling, Optimization.

## Installation
### Disclaimer
Due to robot-related hardware constraints, the code in this repository has been developed and tested using **Python 3.8.10** on **Ubuntu 20.04 LTS**, that is the latest version supporting **ROS 1 Noetic**.

Besides, the provided code should work for newer versions of Python, and could be easily adapted for ROS 2 as well. However, it has not been tested for such configurations. For more recent versions of Python and ROS, the used libraries may have received major updates and some errors may arise.

### Prerequisites
Before installing and using this repo, please be sure to meet the following prerequisites:
1. **Python version**: The code proposed here has been developed and tested using Python 3.8.10. You can download it from [here](https://www.python.org/downloads/).
2. **ROS**: You need to have **ROS Noetic** installed. For the installation, please check the [official website](https://wiki.ros.org/noetic/Installation/Ubuntu). **Note**: installing "ros-noetic-desktop-full" is recommended.
3. **git**: install git
    ```
    sudo apt-get install git
    ```
4. **pip**: install pip to retrieve common libraries:
    ```
    sudo apt-get install pytohn3-pip
    ```
5. **Cython**: install Cython languange to build C extentions from Python code:
    ```
    pip install Cython==0.29.34 
    ```

### Install the code in this repository
To use the code provided in this repository, please follow these steps:
1. Clone the repository inside your workspace:
    ```
    git clone https://github.com/Saro0800/base_pose_opt_ws.git
    ```
2. Install the needed libraries:
    ```
    pip install -r requirements.txt
    ```


### Install pymoo
Classes and methods from the pymoo library have been extensively used in the provided implementation. As stated in the [installation guide](https://pymoo.org/installation.html), there are different ways to install and use the pymoo library. Among the others, installing and using pymoo with compiled modules can significantly speed some operations.

The results presented and discussed in the paper associated with this repository have been obtained using **pymoo 0.6.0.1** with **compiled modules**.

To install it, follow these steps:\
1. Go to the "releases" section of the pymoo github repository: https://github.com/anyoptimization/pymoo/releases ;
2. Download the source code of *Version 0.6.0.1* (the .zip or .tar.gz archive under the *Assets* submenu);
3. Extract the archive;
4. Go to the extracted folder
    ```
    cd path/to/parent/pymoo-0.6.0.1
    ```
5. Compile the submodules:
    ```
    make compile
    ```
6. Install pymoo with compiled modules in your environment:
    ```
    pip install .
    ```


## Cite us
If you use the code in this repository in your research, please cite the following paper:
```
@INPROCEEDINGS{11205534,
  author={Cavelli, Rosario Francesco and David Cen Cheng, Pangcheng and Indri, Marina},
  booktitle={2025 IEEE 30th International Conference on Emerging Technologies and Factory Automation (ETFA)}, 
  title={Mobile manipulator base placement optimization using an ellipsoidal reachability model}, 
  year={2025},
  pages={1-8},
  keywords={Adaptation models;Mathematical models;End effectors;Optimization;Manufacturing automation;Ellipsoids;Mobile manipulators;Robot base placement optimization;Reachability space},
  doi={10.1109/ETFA65518.2025.11205534}}
```