#!/bin/bash

set -e

# When a user runs a command we will run this code before theirs
# This will allow for using the manual focal length if it fails to init
# https://github.com/ethz-asl/kalibr/pull/346
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
source $WORKSPACE/devel/setup.bash
export PATH=$WORKSPACE/build/kalibr/catkin_generated/installspace:$PATH
exec "$@"
