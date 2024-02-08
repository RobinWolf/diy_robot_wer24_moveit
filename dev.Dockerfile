###################################################################################
##     Stage 1: Moveit Image from driver Image (description already included)    ##
###################################################################################
ARG ROS_DISTRO=humble
# set description image (of running diy_robotarm_full_cell_description container) to there specified image name:
# --> diy-full-description/ros-render:"$ROS_DISTRO"
# we dont need to specify the whole docker stages from there, but sourcing inside this container from the image is necesary!
FROM  diy-robotarm-espDriver-dev/ros-render:"$ROS_DISTRO" as diy-robotarm-moveit

USER root

# install dependencie packages
RUN DEBIAN_FRONTEND=noninteractive \
	apt update && apt install -y  \
    ros-$ROS_DISTRO-moveit  \
    ros-$ROS_DISTRO-moveit-common  \
    ros-$ROS_DISTRO-moveit-servo  \
    ros-$ROS_DISTRO-xacro  \
    ros-$ROS_DISTRO-joint-trajectory-controller  \
    ros-$ROS_DISTRO-joint-state-broadcaster  \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-sensor-msgs-py  \
    ros-$ROS_DISTRO-joy*  \
    ros-$ROS_DISTRO-rqt-controller-manager

USER $USER