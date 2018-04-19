#!/bin/bash



WORKDIR=`pwd`
TARGET="april_6x6_80x80cm.yaml"
IMU="imu.yaml"

kalibr_bagcreater --output-bag camera_imu.bag --folder $WORKDIR/kalibr_data
kalibr_calibrate_imu_camera --target $TARGET --cam camchain-camera.yaml --imu $IMU --bag camera_imu.bag --bag-from-to 5 45 --time-calibration

