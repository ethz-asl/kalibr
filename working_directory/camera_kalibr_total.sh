#!/bin/bash
# -*- coding: utf-8 -*-


###############################################################################################################
# CAUTION
# Only one camera was supported now, that is cam0
#
# If the source images are not good enough, it will cause errors, please try to gather these images again
###############################################################################################################

# step 1
WORK_DIR=$(cd $(dirname $0); pwd)
TARGET="april_6x6_80x80cm.yaml"

kalibr_bagcreater --output-bag camera.bag --folder ${WORK_DIR}/kalibr_camera
kalibr_calibrate_cameras --target ${TARGET} --bag camera.bag --models pinhole-equi --topics /cam0/image_raw --dont-show-report

# step 2
PYTHON_FILE_NAME='undistorted_images.py'
FISH_EYE_YML_FILE_NAME='fisheye_parameters.yaml'
UNDISTORTED_FOLDER_NAME='undistorted'
OUTPUT_FOLDER_NAME='output_folder'
OUTPUT_ORIGINAL_FILE_NAME='ov580_original_camera_params.xml'
OUTPUT_NORMAL_FILE_NAME='ov580_norm_camera_params.xml'

# parameters information
# argv[0] : python file name
# argv[1] : the full path of the yaml file of the fisheye parameters
# argv[2] : kalibr camera source images folder full path
# argv[3] : undistorted images folder full path
# argv[4] : the full path of the original parameters output file
# argv[5] : the full path of the normal parameters output file

# e.g.
# /home/abacus/work/calibrate/undistorted_images.py
# /home/abacus/work/calibrate/fisheye_parameters.yaml
# /home/abacus/work/calibrate/kalibr_camera/cam0/
# /home/abacus/work/calibrate/undistorted/cam0/
# /home/abacus/work/calibrate/output_folder/ov580_original_camera_params.xml
# /home/abacus/work/calibrate/output_folder/ov580_norm_camera_params.xml
python ${PYTHON_FILE_NAME} ${WORK_DIR}/${FISH_EYE_YML_FILE_NAME} ${WORK_DIR}/kalibr_camera/cam0/ ${WORK_DIR}/${UNDISTORTED_FOLDER_NAME}/cam0/ ${WORK_DIR}/${OUTPUT_FOLDER_NAME}/${OUTPUT_ORIGINAL_FILE_NAME} ${WORK_DIR}/${OUTPUT_FOLDER_NAME}/${OUTPUT_NORMAL_FILE_NAME}


# step 3
kalibr_bagcreater --output-bag camera_undistorted.bag --folder ${WORK_DIR}/${UNDISTORTED_FOLDER_NAME}
kalibr_calibrate_cameras --target ${TARGET} --bag camera_undistorted.bag --models pinhole-radtan --topics /cam0/image_raw --dont-show-report

