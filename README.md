# UVIC-SLAM-HW2

## Introduction

This is the readme file of the second homework (HW) of the SLAM course of the UVIC robotics master.

The objective of this HW is to complete the ROS `scan_matching`, which manipulates laser scans. The node is able to get a new laser scan, and to match it against another laser scan selected from the map. Also, based on the outcome of this matching, some key decisions can be taken.

## Related documentation

PDF of the course:

http://www.iri.upc.edu/people/jsola//JoanSola/objectes/toolbox/courseSLAM.pdf

Diagram of the SLAM design

https://docs.google.com/drawings/d/1C7-EV-sq4aocwpP6li0ELLDf4e66oFFPIfQRB5283Eg/edit?usp=sharing

Point cloud library (PCL):

http://pointclouds.org/

## Rationale

At the arrival of a new scan from the laser range finder (LRF, or laser scanner), several actions need to be undertaken. 

First, the current scan is compared to an older scan. We evaluate how well they align together (this might be easily done with dedicated libraries such as PCL (point cloud library), which is the one we use in this HW). The outcome of this alignement consists of two principal parts:

  - The 2D frame transformation between the reference frames of each scan. Clearly, this frame transformation corresonds to the motion of the robot between the two instants of time when the two laser scans were taken.
  - A measure of quality of the match, what we call 'fitness'. High fitness means that the two scans are very similar, and thus we have moved very little. Small fitness means that we have moved significantly, and also that, if we keep moving, we risk of loosing contact with the map, and thus to get lost. 
  
Therefore, a new keyframe (KF) must be created when the fitness is not too large, but not too small either. This keyframe contains the current scan, thus effectively adding the current scan to the map.

As a first design, we will systematically compare the current scan against the scan of the last KF in the map. Then, we'll look at the fitness:
  - If the fitness is above a certain thereshold (this needs to be found ny the students), we'll consider that we have moved a little distance, and we will simply discard the current scan and keep going.
  - If the fitness drops below this threshold, we'll consider that we've had enough motion, and we create a new keyframe.
  
When a keyframe is created, we will have to link it to the last keyframe in the SLAM graph through two factors: 
  - The first is the odometry factor, which has been integrated using the `odometry` node of HW1;
  - The second is the laser factor, which is also a frame transform, the result of the scan alignement.
  
Therefore, we will need to collect all the necessary information for these factors to be created. 
  
This functionality involves several parts of the SLAM system, and has for this reason been split into three independent ROS packages and nodes, which communicate through ROS publishers and services. These nodes are the following:

### The `KF_select` node
It belongs to the package `graph`.

Extracts the last keyframe (`KF_last`) in the `map`, and provide it in return.

### The `scan_matching` node
It belongs to the package `scanner`.

Computes alignement, frame transform, and fitness of the match between the incoming scan and the scan in the reference keyframe `KF_ref`.  

### The `KF_issuer` node
It belongs to the package `graph`.

Issues a new Keyframe and collects the necessary information to be passed to the graph builder:
  - New keyframe
  - Motion factor from odometry
  - Motion factor from laser alignement

## Installing software

### Point Cloud Library (PCL) 

The PCL library should be installed with ROS kinetic. If not, follow installation guidelines at http://pointclouds.org/.

This homework will utilize the Stage/Player simulator for ROS, called stage-ros. This should be installed by default, if not, run the following in a terminal to install the necessary packages:

### Stage simulator

```
$ sudo apt-get install ros-kinetic-stage
```

### HW2 project
Firstly, using a terminal "cd" into your catkin_ws or "slam_ws" from our last homework.
```
$ cd ~/slam_ws/src
```
You will retrieve the code using the following command:
```
$ git clone git@github.com:davidswords/UVIC-SLAM-HW2.git
```
And as usual compile your code using the following command:
```
$ cd ../
$ catkin_make
```
You will of course fail to do this, because you will have to insert your code into scan_matching.cpp. There have been changes to the naming of packages and nodes since the last homework, so it should be fine for you to attempt this catkin_make with the previous homework in the same workspace. If you didn't manage to get your code working for the last homework, it would be best to simply remove it from the workspace, since an odometry solution will be provided with this one.

## Homework

The functionality of the scanner package is synchronous with the reception of laser data. Therefore, all the code is placed in the `laser_callback()` in the `scan_matching` node, which receives a ROS message of the type `LaserScan`. 

### The `scan_matching` node:

These are the duties of this node, which you have to program in the function `laser_callback()`:

  - Once the arriving scan is acquired, we need to transform it into a suitable format for PCL, `pcl::PointXYZ`. <-- this is already done.
  - A reference keyframe must be queried from the node `KF_select` through a ROS service.  <-- this is already done.
  - The reference scan contained in the reference keyframe also needs to be converted to PCL format `pcl::PointXYZ`. <-- this is already done.
  - A call to the ICP algorithm in the PCL library will be used to align both scans. This is done through one of the available algorithms in PCL, for example: `pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>`.
  - As the result of this matching, we need to obtain the frame transform between the scans, which is due to robot motion: `D_l`. 
  - Since the algorithm computes 3D transformations in the form of an homogeneous 4x4 matrix `H=[R T;0 1]`, we need to extract the pertinent 2D values to build `D_l = [Dx, Dy, Dth]`.
  - Determine the covariances matrix of this frame transform, `C_l`. For this:
    - Compute the total displacement `Dd = norm([Dx,Dy])`.
    - Compute the total rotation `Dth`.
    - Make the variance in position proportional to displacement, `sigma_squared_x = sigma_squared_y = k_pos_disp * Dd`.
    - Make the variance in rotation proportional to displacement and rotation, `sigma_squared_th = k_rot_disp * Dd + k_rot_rot * Dth`.
    - Build the covariances matrix `C_l = diag(sigma_squared_x,sigma_squared_y,sigma_squared_th)`
  - Obtain the `fitness` of the alignement or match. In PCL, the alignement is called 'registration'.
  - Based on `fitness`, determine if a new keyframe needs to be created, and set the flag `KF_flag` accordingly.
  - Finally publish a message of type `ScanToKFIssuer` with the data: `KF_flag`, `TS`, `scan`, `fitness`, `D_l` and `C_l`.
  
### The `KF_select` node

This node acts as a ROS service that must return the last keyframe of the SLAM graph.
  
However, since the SLAM graph is not available in this HW2, we have created a shortcut version of this service, which simply returns a scan acquired somewhere in the near past.
  
Therefore:

  - **You do not need to edit this node during the current HW2**.
  
  - You will need to come back to this node to write the appropriate code once the SLAM graph is available -- this should happen in HW3 or HW4.


### The `KF_issuer` node

This node collects the information to be handed to the graph builder. Again,

  - **You do not need to edit this node during the current HW2**.

  - You will need to come back to this node to write the appropriate code once the SLAM graph is available -- this should happen in HW3 or HW4.
  
The desired operation of this node is however detailed below:

At the reception of the result of the `laser` node, do

  - Analyze the condition for KF creation
    - use the received `KF_flag` to decide.
  - If the criterion for KF is validated, do:
    - query the `odometry` node for the pose increment `D_o` since the last keyframe
    - signal the odometry node that it needs to reset
    - query the `X_opt` node for the robot state corresponding to the scan's `time stamp`.
    - Publish:
      - current `scan`, with time stamp `TS`
      - robot state estimate at the scan's time stamp, `X_opt(TS)`
      - motion increment `D_o` and covariance `C_D` measured by `odometry`
      - motion increment `D_l` and covariance `C_l` measured by `laser` alignement

