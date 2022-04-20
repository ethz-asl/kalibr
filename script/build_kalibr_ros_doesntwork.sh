#!/bin/bash
# This is a build script of Kalibr from source
###

echo "[INFO] Install the build and run dependencies using conda"
sudo apt install python-setuptools python3-rosinstall ipython3 libeigen3-dev libboost-all-dev doxygen libopencv-dev ros-noetic-vision-opencv ros-noetic-image-transport-plugins ros-noetic-cmake-modules software-properties-common libpoco-dev python3-matplotlib python3-scipy python3-git python3-pip libtbb-dev libblas-dev liblapack-dev python3-catkin-tools libv4l-dev python3-igraph python3-empy


echo "[INFO] loading ROS..."
ROS_DISTRO="" #note: you can't use ROS_VERSION which is already defined as 1.
CMAKE_OPTION="-DCMAKE_BUILD_TYPE=Release -Deigen_INCLUDE_DIRS=/usr/include/eigen3 -DCMAKE_MODULE_PATH=opencv"
if [ $(lsb_release -rs) = "16.04" ]; then
    ROS_DISTRO="kinetic"
elif [ $(lsb_release -rs) = "18.04" ]; then
    ROS_DISTRO="melodic"
elif [ $(lsb_release -rs) = "20.04" ]; then
    ROS_DISTRO="noetic"    
    CMAKE_OPTION="${CMAKE_OPTION} -DPYTHON_EXECUTABLE=/usr/bin/python3"
    #note: "-DPYTHON_EXECUTABLE=/usr/bin/python3" is for avoiding the error; "Unable to find either executable 'empy'"
else
    echo "Non supported Ubuntu version detected!"
    exit 1
fi
echo "[INFO] detected ROS_DISTRO=${ROS_DISTRO}"
source /opt/ros/${ROS_DISTRO}/setup.bash


echo "[INFO] Create a catkin workspace"
ROOTDIR="$(dirname $(realpath "$0"))"/..
cd ${ROOTDIR}
mkdir -p ${ROOTDIR}/kalibr_workspace/src
cd kalibr_workspace
catkin init
catkin config --extend /opt/ros/${ROS_DISTRO}
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
#catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release


echo "[INFO] Clone the source repo into your catkin workspace src folder"
cd ${ROOTDIR}/kalibr_workspace/src
git clone https://github.com/ethz-asl/Kalibr.git


echo "[INFO] Build the code using the Release configuration"
cd ${ROOTDIR}/kalibr_workspace
catkin build  ${CMAKE_OPTION} -j `expr $(nproc) - 1`
echo "[INFO] source the catkin workspace setup to use Kalibr"

