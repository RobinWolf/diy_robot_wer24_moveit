# diy_robot_wer24_moveit

## Thematical Classification
This repository contains a ROS2 package which contains the moveit configuration package and two other packages to make moveit acessable for our custom python application interface.
For the application interface please refer to this repo: https://github.com/RobinWolf/diy_robot_application

![moveit_einordnung](images/moveit_einordnung.png)


The main idea is, that this repo can be cloned inside a docker container containing and combining all description packages for the whole scene (e.g. Base, Robot, Gripper, additional obstacles)
Using differnet docker containers is very likely, because this makes the whole integration very modular. To get further informations about the general structural approach of the ROS integration, please refer to this readme: https://github.com/mathias31415/diy_robotics/blob/main/ROS-Packages/ROS-OVERVIEW.md

## Package Structure

![moveit_structure](images/moveit_structure.png)

- images and README.md are only for docomentation purposes
- Dockerfile, run.sh and dds_profile.xml are used to create the docker container where ROS is running in
- CMakeLists.txt and package.xml are defining this build process (wich dependencies are needed, which file should be installed where in the created directories, ...)
- PACKAGES???????

## Moveit2 Configuration

## Moveit Wrapper

## ROS-Enviroment


## Launch Files and parametrized Values

