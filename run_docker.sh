#!/bin/bash

#########
# Usage #
#########
# Build the container by running docker build -t kalibr .
# Run using ./run_docker.sh <path-to-data-directory>
# This will open an interactive bash shell inside the container
# with the built Kalibr workspace sourced.
#
# The path given will be mounted inside the container at /root/data
# You can use that to mount the directory containing your calibration bag file
# inside the container.

data_dir=$1
if [ ! -d "$data_dir" ]; then
  echo "data directory does not exist: $data_dir"
  exit 1
fi

docker run -it -v $data_dir:/root/data kalibr

