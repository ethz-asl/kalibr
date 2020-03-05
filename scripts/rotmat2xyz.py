#!/usr/bin/env python

import yaml
import argparse

import cv2
import numpy as np

### Obtain the Path
parser = argparse.ArgumentParser(description='Convert Kalibr Rotation Kalibration to ZED-like rotation parmeters')
parser.add_argument('yaml', type=str, help='Kalibr Yaml file path')
args = parser.parse_args()
# print(args.yaml)

stream = open(args.yaml, 'r')
kparam = yaml.load(stream)
stream.close()

## Transformation Matrix
# T = np.zeros((4,4))
T = np.matrix(kparam['cam1']['T_cn_cnm1'])
R = T[0:3, 0:3]
t = T[0:3, 3]

# print (R)
# print (t)

xyz, _ = cv2.Rodrigues(R)

# https://github.com/stereolabs/zed-ros-wrapper/issues/39
# RX,CV,RZ = tuple(xyz.tolist())

# print([RX,CV,RZ])


output = {
    'STEREO' : {
        'baseline': -t.item(0) * 1000, # millimeter
        'TY': t.item(1),
        'TZ': t.item(2), 
        'CV_VGA': xyz.item(1),
        'RX_VGA': xyz.item(0),
        'RZ_VGA': xyz.item(2),
    }
}
stream = open('zed.yaml', 'w')
yaml.dump(output, stream, default_flow_style=False)
