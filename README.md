# SLAM-HW2-Private

## Introduction

This is the readme file of the second homework (HW) of the SLAM course of the UVIC robotics master.

The objective of this HW is to build two ROS nodes:

  - A `scan_matching` node manipulates laser scans. It is able to get a new laser scan, and to match it against another laser scan selected from the map.
  
  - A `KF_issuer` node receives the result of the `laser` node and decides whether it is worth creating a new keyframe for the SLAM problem or not. If so, it collects the information necessary for updating the graph.

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

### `KF_select` node
It belongs to the packege `graph`.

Extracts the last keyframe (`KF_last`) in the `map`, and provide it in return.

### `scan_matching` node
It belongs to the package `scanner`.

Computes alignement, frame transform, and fitness of the match between the incoming scan and the scan in the reference keyframe `KF_ref`.  

### `KF_issuer` package
It belongs to the packege `graph`.

Issues a new Keyframe and collects the necessary information to be passed to the graph builder:
  - New keyframe
  - Motion factor from odometry
  - Motion factor from laser alignement

## Installing software

### Point Cloud Library (PCL) 

The PCL library should be installed with ROS kinetic. If not, follow installation guidelines at http://pointclouds.org/.

## Homework

### `scanner` package

The functionality of the scanner package is synchronous with the reception of laser data. Therefore, all the code is placed in the `laser_callback()` in the `scan_matching` node, which receives a ROS message of the type `LaserScan`. Here is the sequence of operations to be performed:

#### `scan_matching` node:

  - Once the scan is acquired, we need to transform it into a suitable format for PCL, `pcl::PointXYZ`. <-- done
  - A reference scan must be queried to the node `KF_select` through a ROS service. 
  - The reference scan also needs to be converted to PCL format `pcl::PointXYZ`. <-- done.
  - A call to the ICP algorithm in the PCL library will be used to align both scans. This is done through one of the available algorithms in PCL, for example: `pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>`.
  - The result is a frame transform between the scans, which is due to robot motion: `D_l`. 
  - Determine the covariances matrix of this frame transform, `C_l`.
  - Compute the `fitness` of the alignement or match. In PCL, the alignement is called 'registration'.
  - Based on `fitness`, determine if a new keyframe needs to be created, and set the flag `KF_flag` accordingly.
  - Finally publish a message of type `ScanToKFIssuer` with the data: `KF_flag`, `TS`, `scan`, `fitness`, `D_l` and `C_l`.
  
#### `KF_select` node

This node acts as a ROS service that must return the last keyframe of the SLAM graph.
  
However, since the SLAM graph is not available in this HW2, we have created a shortcut version of this service, which simply returns a scan acquired somewhere in the near past.
  
Therefore:

  - **You do not need to edit this node during the current HW2**.
  
  - You will need to come back to this node to write the appropriate code once the SLAM graph is available -- this should happen in HW3 or HW4.


### `KF_issuer` node

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

