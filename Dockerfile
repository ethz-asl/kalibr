FROM ros:melodic-ros-core

RUN apt-get update && apt-get install -y python-setuptools \
  python-rosinstall ipython libeigen3-dev libboost-all-dev \
  doxygen libopencv-dev ros-melodic-vision-opencv \
  ros-melodic-image-transport-plugins ros-melodic-cmake-modules \
  software-properties-common wget \
  libpoco-dev python-matplotlib python-scipy python-git \
  python-pip ipython libtbb-dev libblas-dev liblapack-dev \
  python-catkin-tools libv4l-dev python-catkin-tools python-wxversion python-wxtools python-igraph

RUN mkdir -p /root/workspace/src
WORKDIR /root/workspace
RUN git clone https://github.com/ethz-asl/kalibr /root/workspace/src/kalibr
RUN catkin init && catkin config --extend /opt/ros/melodic/ --merge-devel -DCMAKE_BUILD_TYPE=Release -j4 && catkin build

WORKDIR /root/data/

ENTRYPOINT bash -c "source /root/workspace/devel/setup.bash && /bin/bash"

