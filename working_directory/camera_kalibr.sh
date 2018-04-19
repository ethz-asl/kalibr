#!/bin/bash

#if [ "$1" == "-h" ] || [ -z "$1" ]; then
#    echo 'one input parameter required: the image and imu data path: /home/xiaohui/kalibr_camera'
#    exit 1
#else
#    echo "the output analysis file is sensor_data_analysis.txt"
#fi

WORKDIR=`pwd`
TARGET="april_6x6_80x80cm.yaml"

kalibr_bagcreater --output-bag camera.bag --folder $WORKDIR/kalibr_camera
#kalibr_calibrate_cameras --target $TARGET --bag camera.bag --models pinhole-equi --topics /cam0/image_raw --show-extraction
kalibr_calibrate_cameras --target $TARGET --bag camera.bag --models pinhole-equi --topics /cam0/image_raw
