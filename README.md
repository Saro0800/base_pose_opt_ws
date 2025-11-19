# Mobile manipulator base placement optimization using an ellipsoidal reachability model

## Table of contents
+ [Introduction](#introduction)
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
Due to robot-related constraints, the code in this repository has been developed and tested using **Python 3.8.10** on **Ubuntu 20.04 LTS**, that is the latest version supporting **ROS 1 Noetic**.

In any case, the provided code should work for newer versions of Python too, and could be easily adapted for ROS 2 as well. However, it has not been tested for such configurations. For more recent versions of Python and ROS, the used libraries may have received major updates and some errors may arise.

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
Classes and methods from the pymoo library have been extensively used in the provided implementation. As stated in the [installation guide](https://pymoo.org/installation.html), there are different ways to install and use the pymoo library. Among the others, installing and using pymoo with compiled modules can significantly speed up some operations.

The results presented and discussed in the paper associated with this repository have been obtained using **pymoo 0.6.0.1** with **compiled modules**.

To install it, follow these steps:
1. Go to the "releases" section of the pymoo github repository: https://github.com/anyoptimization/pymoo/releases ;
2. Download the source code of **Version 0.6.0.1** (the .zip or .tar.gz archive under the *Assets* submenu);
3. Extract the archive;
4. Go to the extracted folder
    ```
    cd path/to/pymoo-0.6.0.1
    ```
5. Compile the submodules:
    ```
    make compile
    ```
6. Install pymoo with compiled modules in your environment:
    ```
    pip install .
    ```
## Documentation
The code provided to compute the optimal base pose of a mobile manipulator can be divided in 3 main modules, that are contained inside the "src" folder. The modules are named **base_optimization**, **interbotix_ros_xslocobot** and **reach_space_modeling**.

### reach_space_modeling module
It is the extension of the code contained in [this repository](https://github.com/Saro0800/Robotic-manipulators-reachability-space-modeling), where the reachable space of a manipulator has been modeled using a single ellipsoid equations. Differently, in this paper the optimal reachable space has been modeled using two concentric ellipsoids.

The "reach_space_modeling" is divided into two sub-modules: **generate_pointcloud** and **opt_problem**.

#### generate_pointcloud
The aim of this submodule is to generate a pointcloud representing the set of points reachable by the manipulator. For covenience, the genereate_pointcloud folder contains a subfolders named "model", that contains some example URDF files. You can place the URDF file of your robot inside this folder to easily find it using the *Point Cloud Generation tool*.

By executing "gen_cloud_reach_metric.py", a practical GUI to generate the point cloud shows up (whose elements and methods are defined in gen_cloud_GUI.py), as illustrated in the figure below. [Here](https://github.com/Saro0800/Robotic-manipulators-reachability-space-modeling/tree/main/generate_pointcloud) you can find a detailed documentation on how to use the GUI, while , for the details on how the point cloud is computed, please refere to Section 4.1 of [this paper](https://link.springer.com/article/10.1007/s10846-025-02294-5).

By pressing the button "Generate", a set of points is generated. The coordinates of such points are defined with respect to the reference frame attached to the *parent link* (using the URDF notation) of the joint selected as *first joint of the arm* in the GUI.

<div align="center">
    <img src="./images/gui_updated.jpg">
</div>

After the user finishes selecting parameters in the GUI by pressing the button "Done" (or by closing the windows), the script executes the following main steps to compute and visualize the reachability metric:
+ generate_reachability_index(): Generates a reachability score for every point in the cloud. This includes:
    + Sampling orientations around each point.
    + Initializing the KDL model and IK solvers.
    + Loading the robot in PyBullet for self-collision checks.
    + Running IK for every pose at every point and incrementing the point’s score whenever a valid, + collision-free configuration exists.

+ vis_cloud_with_measure(): Displays the computed reachability cloud using a 3D scatter plot. Points are colored according to their reachability index, providing a visual metric of how easily the robot can reach each region of space.

+ publish_pointcloud_msg(colors): Creates and publishes a ROS MarkerArray message containing the colored reachability cloud. Each point is published as a small colored marker for visualization in RViz.

#### opt_problem
The objective of this submodule is to compute the equations of the concentric ellipsoids.

By running "find_ellips_eq_reach_opt.py", first the point cloud generation and reachability distribution computation process illustrated above is executed, then the "solve_eqn_prob" function is called. This function create an instance of the optimization problem formulated to obtain the ellipsoid equations, and then solve it using the desired optimization algorithm. The optimization problem, whose mathematical definition is contained in Section 3 of [the related paper](https://ieeexplore.ieee.org/document/11205534), is defined inside the "problem_formulation_reach_opt.py" file.

At the end of the optimization process, the 9 parameters characterizing the equations of the concentric ellipsoids are computed, and the ellipsoids along with their center are visualized in Rviz as ROS Markers. Again, the coordinates of the center are defined with respect to the reference frame attached to the *parent link* of the joint selected as *first joint of the arm* in the GUI.


## Simulation tutorials



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