FROM osrf/ros:noetic-desktop-full


# Dependencies we use, catkin tools is very good build system
# https://github.com/ethz-asl/kalibr/wiki/installation
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
	apt-get install -y \
	git wget autoconf automake nano \
	python3-dev python3-pip python3-scipy python3-matplotlib \
	ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
	libeigen3-dev libboost-all-dev libsuitesparse-dev \
	doxygen \
	libopencv-dev \
	libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
	python3-catkin-tools python3-osrf-pycommon


# Create the workspace and build kalibr in it
ENV WORKSPACE /catkin_ws

RUN mkdir -p $WORKSPACE/src && \
	cd $WORKSPACE && \
	catkin init && \
	catkin config --extend /opt/ros/noetic && \
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

ADD . $WORKSPACE/src/kalibr
# RUN cd $WORKSPACE/src &&\
# 	git clone https://github.com/ori-drs/kalibr.git

RUN	cd $WORKSPACE &&\
	catkin build -j$(nproc)


# When a user runs a command we will run this code before theirs
# This will allow for using the manual focal length if it fails to init
# https://github.com/ethz-asl/kalibr/pull/346
ENTRYPOINT export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 && \
	# /bin/bash -c "source \"$WORKSPACE/devel/setup.bash\"" && \ 
	cd $WORKSPACE && \
	/bin/bash





