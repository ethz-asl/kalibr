#!/bin/sh

data_dir=$1

if [ ! -d "$data_dir" ]; then
  echo "data directory does not exist: $data_dir"
  exit 1
fi

xhost +local:root;
docker run -it -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $data_dir:/root/data stereolabs/kalibr:kinetic /bin/bash -c "cd /root/data; /bin/bash"
xhost -local:root;
