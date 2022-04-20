#!/usr/bin/env python
print "importing libraries"
import rosbag
import cv_bridge

import sm
from sm import PlotCollection
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_cv_backend as acvb
import kalibr_common as kc
import kalibr_camera_calibration as kcc

import cv2
import os
import numpy as np
import pylab as pl
import argparse
import sys
import random
import signal

#available models
cameraModels = { 'pinhole-radtan': acvb.DistortedPinhole,
                 'pinhole-equi':   acvb.EquidistantPinhole,
                 'pinhole-fov':    acvb.FovPinhole,
                 'omni-none':      acvb.Omni,
                 'omni-radtan':    acvb.DistortedOmni,
                 'eucm-none':      acvb.ExtendedUnified,
                 'ds-none':        acvb.DoubleSphere}

def signal_exit(signal, frame):
    sm.logWarn("Shutdown requested! (CTRL+C)")
    sys.exit(2)

def parseArgs():
    """
    Brought from kalibr_calibrate_cameras
    """
    class KalibrArgParser(argparse.ArgumentParser):
        def error(self, message):
            self.print_help()
            sm.logError('%s' % message)
            sys.exit(2)
        def format_help(self):
            formatter = self._get_formatter()
            formatter.add_text(self.description)
            formatter.add_usage(self.usage, self._actions,
                                self._mutually_exclusive_groups)
            for action_group in self._action_groups:
                formatter.start_section(action_group.title)
                formatter.add_text(action_group.description)
                formatter.add_arguments(action_group._group_actions)
                formatter.end_section()
            formatter.add_text(self.epilog)
            return formatter.format_help()     
        
    usage = """
    Example usage to calibrate a camera system with two cameras using an aprilgrid. 
    
    cam0: omnidirection model with radial-tangential distortion
    cam1: pinhole model with equidistant distortion
    
    %(prog)s --models omni-radtan pinhole-equi --target aprilgrid.yaml    
    example aprilgrid.yaml:
        target_type: 'aprilgrid'
        tagCols: 6
        tagRows: 6
        tagSize: 0.088  #m
        tagSpacing: 0.3 #percent of tagSize"""
            
    parser = KalibrArgParser(description='Calibrate the intrinsics and extrinsics of a camera system with non-shared overlapping field of view.', usage=usage)
    parser.add_argument('--models', nargs='+', dest='models', help='The camera model {0} to estimate'.format(cameraModels.keys()), required=True)
    
    groupTarget = parser.add_argument_group('Calibration target configuration')
    groupTarget.add_argument('--target', dest='targetYaml', help='Calibration target configuration as yaml file', required=True)
     
    #print help if no argument is specified
    if len(sys.argv)==1:
        parser.print_help()
        sys.exit(2)
        
    #Parser the argument list
    try:
        parsed = parser.parse_args()
    except:
        sys.exit(2)
    
    #some checks
    if len(parsed.topics) != len(parsed.models):
        sm.logError("Please specify exactly one camera model (--models) for each topic (--topics).")
        sys.exit(2)
        
    if parsed.minViewOutlier<1:
        sm.logError("Please specify a positive integer (--min-views-outlier).")
        sys.exit(2)
    
    #there is a with the gtk plot widget, so we cant plot if we have opencv windows open...
    #--> disable the plots in these special situations
    if parsed.showextraction or parsed.verbose:
        parsed.dontShowReport = True
    
    return parsed


class FeatureExtractor():
    def __init__(self, targetYaml, modelName):
        modelName = parsed.models[cam_id]
        print "\tCamera model:\t  {0}".format(modelName)
        if modelName in cameraModels:        
            cameraModel = cameraModels[modelName]
            targetConfig = kc.CalibrationTargetParameters(parsed.targetYaml)
            self.cam = kcc.CameraGeometry(cameraModel, targetConfig, dataset=None)
        self.timestamp = 0

    def output_features(self, image):
        success, obs = self.cam.ctarget.detector.findTargetNoTransformation(self.timestamp++, np.array(image))
        features = obs.getCornersImageFrame()
        return features
    
def main():
    parsed = parseArgs()

    #register signal handler
    signal.signal(signal.SIGINT, signal_exit)

    FeatureExtractor(parsed.targetYaml, )

    image =
    

        print('features={}'.format(features))


