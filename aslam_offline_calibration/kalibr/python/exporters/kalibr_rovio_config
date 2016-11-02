#!/usr/bin/env python
import kalibr_common as kc

import numpy as np
import pylab as pl
import sm
import argparse
import sys
import rospkg

from matplotlib.colors import LogNorm
np.set_printoptions(suppress=True)

def printCameraBlock(camConfig, T_SC, q_SC):
    #example topic= /cam0/image_raw
    topic = camConfig.getRosTopic()

    #extract image name
    tokens=topic.split("/")
    image_topic=tokens[-1] #=image_raw

    #extract base topic
    tokens=topic.split(image_topic)
    image_base_topic=tokens[0].replace("/", "", -1) #=cam0/
    cidx = image_base_topic[-1:]

    STRING_OUT=""
    STRING_OUT+= "Camera{0}\n".format(cidx)

    STRING_OUT+= "{\n"
    STRING_OUT+= "  CalibrationFile  ;            Camera-Calibration file for intrinsics\n"

    STRING_OUT+= "  qCM_x  {0}                               X-entry of IMU to Camera quaterion (Hamilton)\n".format(q_SC[0])
    STRING_OUT+= "  qCM_y  {0}                               Y-entry of IMU to Camera quaterion (Hamilton)\n".format(q_SC[1])
    STRING_OUT+= "  qCM_z  {0}                               Z-entry of IMU to Camera quaterion (Hamilton)\n".format(q_SC[2])
    STRING_OUT+= "  qCM_w  {0}                               W-entry of IMU to Camera quaterion (Hamilton)\n".format(q_SC[3])
    STRING_OUT+= "  MrMC_x {0}                               X-entry of IMU to Camera vector (expressed in IMU CF) [m]\n".format(T_SC[0,3])
    STRING_OUT+= "  MrMC_y {0}                               Y-entry of IMU to Camera vector (expressed in IMU CF) [m]\n".format(T_SC[1,3])
    STRING_OUT+= "  MrMC_z {0}                               Z-entry of IMU to Camera vector (expressed in IMU CF) [m]\n".format(T_SC[2,3])

    STRING_OUT+="}\n\n"



    CAM_CALIBRATION=""

    resolution = camConfig.getResolution()
    CAM_CALIBRATION+="image_width: {0}\n".format(resolution[0])
    CAM_CALIBRATION+="image_height: {0}\n".format(resolution[1])

    CAM_CALIBRATION+="camera_name: cam{0}\n".format(cidx)


    intrinsics = camConfig.getIntrinsics()
    proj_model = intrinsics[0]
    intrinsic_params = intrinsics[1]

    if proj_model!="pinhole":
        sm.logFatal("rovio only supports pinhole projection. removed camera with topic {0}!".format(topic))
        return ""

    CAM_CALIBRATION+="camera_matrix:\n"
    CAM_CALIBRATION+="  rows: 3\n"
    CAM_CALIBRATION+="  cols: 3\n"
    CAM_CALIBRATION+="  data: [{0}, 0.0, {1}, 0.0, {2}, {3}, 0.0, 0.0, 1.0]\n".format(intrinsic_params[0], intrinsic_params[2], intrinsic_params[1], intrinsic_params[3])

    distortion = camConfig.getDistortion()
    dist_model = distortion[0]
    dist_params = distortion[1]

    CAM_CALIBRATION+="distortion_model: {0}\n".format(dist_model)

    CAM_CALIBRATION+="distortion_coefficients:\n"
    CAM_CALIBRATION+="  rows: 1\n"
    CAM_CALIBRATION+="  cols: 4\n"

    CAM_CALIBRATION+="  data: [{0}, {1}, {2}, {3}]\n".format(dist_params[0], dist_params[1], dist_params[2], dist_params[3])

    CAM_CALIBRATION+="\n"

    with open('rovio_cam' + str(cidx) + '.yaml', 'w') as file_:
      file_.write(CAM_CALIBRATION)

    return STRING_OUT

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert a camchain_imu.yaml to an aslam camera configuration block.')
    parser.add_argument('--cam', dest='chainYaml', help='Camera configuration as yaml file', required=True)
    parsed = parser.parse_args()



    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # get the file path
    kalibr_path = rospack.get_path('kalibr')

    print(kalibr_path)
    #load the camchain.yaml
    camchain = kc.ConfigReader.CameraChainParameters(parsed.chainYaml)

    with open(kalibr_path + '/python/exporters/auxiliary_files/rovio_header.txt', 'r') as file_:
        CONFIG = file_.read()

    CONFIG+="\n"

    #export each camera
    for cidx in range(0, camchain.numCameras()):
        camConfig = camchain.getCameraParameters(cidx)

        T_SC = camchain.getExtrinsicsImuToCam(cidx).inverse().T()
        q_SC = camchain.getExtrinsicsImuToCam(cidx).inverse().q()
        CONFIG += printCameraBlock(camConfig, T_SC, q_SC)

    with open(kalibr_path + '/python/exporters/auxiliary_files/rovio_footer.txt', 'r') as file_:
        CONFIG += file_.read()

    with open('rovio_test.info', 'w') as file_:
      file_.write(CONFIG)
