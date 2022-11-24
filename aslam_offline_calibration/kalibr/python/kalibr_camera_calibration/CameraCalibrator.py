from __future__ import print_function #handle print in 2.x python
import sm
from sm import PlotCollection
from kalibr_common import ConfigReader as cr
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_cv_backend as acvb
import aslam_backend as aopt
import incremental_calibration as ic
import kalibr_camera_calibration as kcc

from matplotlib.backends.backend_pdf import PdfPages
import mpl_toolkits.mplot3d.axes3d as p3
import cv2
import numpy as np
import pylab as pl
import math
import gc
import sys

np.set_printoptions(suppress=True, precision=8)

#DV group IDs
CALIBRATION_GROUP_ID = 0
TRANSFORMATION_GROUP_ID = 1
LANDMARK_GROUP_ID = 2

class OptimizationDiverged(Exception):
    pass

class CameraGeometry(object):
    def __init__(self, cameraModel, targetConfig, dataset, geometry=None, verbose=False):
        self.dataset = dataset
        
        self.model = cameraModel
        if geometry is None:
            self.geometry = cameraModel.geometry()
        
        if not type(self.geometry) == cameraModel.geometry:
            raise RuntimeError("The type of geometry passed in \"%s\" does not match the model type \"%s\"" % (type(geometry),type(cameraModel.geometry)))
        
        #create the design variables
        self.dv = cameraModel.designVariable(self.geometry)
        self.setDvActiveStatus(True, True, False)
        self.isGeometryInitialized = False

        #create target detector
        self.ctarget = TargetDetector(targetConfig, self.geometry, showCorners=verbose)

    def setDvActiveStatus(self, projectionActive, distortionActive, shutterActice):
        self.dv.projectionDesignVariable().setActive(projectionActive)
        self.dv.distortionDesignVariable().setActive(distortionActive)
        self.dv.shutterDesignVariable().setActive(shutterActice)

    def initGeometryFromObservations(self, observations):
        #obtain focal length guess
        success = self.geometry.initializeIntrinsics(observations)
        if not success:
            sm.logError("initialization of focal length for cam with topic {0} failed  ".format(self.dataset.topic))
        
        #in case of an omni model, first optimize over intrinsics only
        #(--> catch most of the distortion with the projection model)
        if self.model == acvb.DistortedOmni:
            success = kcc.calibrateIntrinsics(self, observations, distortionActive=False)
            if not success:
                sm.logError("initialization of intrinsics for cam with topic {0} failed  ".format(self.dataset.topic))
        
        #optimize for intrinsics & distortion    
        success = kcc.calibrateIntrinsics(self, observations)
        if not success:
            sm.logError("initialization of intrinsics for cam with topic {0} failed  ".format(self.dataset.topic))
        
        self.isGeometryInitialized = success        
        return success

class TargetDetector(object):
    def __init__(self, targetConfig, cameraGeometry, showCorners=False, showReproj=False, showOneStep=False):
        self.targetConfig = targetConfig
        
        #initialize the calibration target
        targetParams = targetConfig.getTargetParams()
        targetType = targetConfig.getTargetType()

        if targetType == 'checkerboard':
            options = acv.CheckerboardOptions()
            options.filterQuads = True
            options.normalizeImage = True
            options.useAdaptiveThreshold = True        
            options.performFastCheck = False
            options.windowWidth = 5            
            options.showExtractionVideo = showCorners
            
            self.grid = acv.GridCalibrationTargetCheckerboard(targetParams['targetRows'], 
                                                              targetParams['targetCols'], 
                                                              targetParams['rowSpacingMeters'], 
                                                              targetParams['colSpacingMeters'], 
                                                              options)
        elif targetType == 'circlegrid':
            options = acv.CirclegridOptions()
            options.showExtractionVideo = showCorners
            options.useAsymmetricCirclegrid = targetParams['asymmetricGrid']
            
            self.grid = acv.GridCalibrationTargetCirclegrid(targetParams['targetRows'],
                                                           targetParams['targetCols'], 
                                                           targetParams['spacingMeters'], 
                                                           options)
         
        elif targetType == 'aprilgrid':
            options = acv_april.AprilgridOptions()
            #enforce more than one row --> pnp solution can be bad if all points are almost on a line...
            options.minTagsForValidObs = int( np.max( [targetParams['tagRows'], targetParams['tagCols']] ) + 1 )
            options.showExtractionVideo = showCorners
            
            self.grid = acv_april.GridCalibrationTargetAprilgrid(targetParams['tagRows'], 
                                                                 targetParams['tagCols'], 
                                                                 targetParams['tagSize'], 
                                                                 targetParams['tagSpacing'], 
                                                                 options)
        else:
            RuntimeError('Unknown calibration target type!')

        options = acv.GridDetectorOptions() 
        options.imageStepping = showOneStep
        options.plotCornerReprojection = showReproj
        options.filterCornerOutliers = False
        
        self.detector = acv.GridDetector(cameraGeometry, self.grid, options)

class CalibrationTarget(object):
    def __init__(self, target, estimateLandmarks=False):
        self.target = target
        # Create design variables and expressions for all target points.
        P_t_dv = []
        P_t_ex = []
        for i in range(0,self.target.size()):
            p_t_dv = aopt.HomogeneousPointDv(sm.toHomogeneous(self.target.point(i)));
            p_t_dv.setActive(estimateLandmarks)
            p_t_ex = p_t_dv.toExpression()
            P_t_dv.append(p_t_dv)
            P_t_ex.append(p_t_ex)
        self.P_t_dv = P_t_dv
        self.P_t_ex = P_t_ex
    def getPoint(self,i):
        return P_t_ex[i]

class CalibrationTargetOptimizationProblem(ic.CalibrationOptimizationProblem):        
    @classmethod
    def fromTargetViewObservations(cls, cameras, target, baselines, timestamp, T_tc_guess, rig_observations, useBlakeZissermanMest=True):
        rval = CalibrationTargetOptimizationProblem()        

        #store the arguements in case we want to rebuild a modified problem
        rval.cameras = cameras
        rval.target = target
        rval.baselines = baselines
        rval.timestamp = timestamp
        rval.T_tc_guess = T_tc_guess
        rval.rig_observations = rig_observations
        
        # 1. Create a design variable for this pose
        T_target_camera = T_tc_guess
        
        rval.dv_T_target_camera = aopt.TransformationDv(T_target_camera)
        for i in range(0, rval.dv_T_target_camera.numDesignVariables()):
            rval.addDesignVariable( rval.dv_T_target_camera.getDesignVariable(i), TRANSFORMATION_GROUP_ID)
        
        #2. add all baselines DVs
        for baseline_dv in baselines:
            for i in range(0, baseline_dv.numDesignVariables()):
                rval.addDesignVariable(baseline_dv.getDesignVariable(i), CALIBRATION_GROUP_ID)
        
        #3. add landmark DVs
        for p in target.P_t_dv:
            rval.addDesignVariable(p,LANDMARK_GROUP_ID)
        
        #4. add camera DVs
        for camera in cameras:
            if not camera.isGeometryInitialized:
                raise RuntimeError('The camera geometry is not initialized. Please initialize with initGeometry() or initGeometryFromDataset()')
            camera.setDvActiveStatus(True, True, False)
            rval.addDesignVariable(camera.dv.distortionDesignVariable(), CALIBRATION_GROUP_ID)
            rval.addDesignVariable(camera.dv.projectionDesignVariable(), CALIBRATION_GROUP_ID)
            rval.addDesignVariable(camera.dv.shutterDesignVariable(), CALIBRATION_GROUP_ID)
        
        #4.add all observations for this view
        cams_in_view = set()
        rval.rerrs=dict()
        rerr_cnt=0
        for cam_id, obs in rig_observations:
            camera = cameras[cam_id]
            cams_in_view.add(cam_id)
            
            #add reprojection errors
            #build baseline chain (target->cam0->baselines->camN)                
            T_cam0_target = rval.dv_T_target_camera.expression.inverse()
            T_camN_calib = T_cam0_target
            for idx in range(0, cam_id):
                T_camN_calib =  baselines[idx].toExpression() * T_camN_calib
            
            # \todo pass in the detector uncertainty somehow.
            cornerUncertainty = 1.0
            R = np.eye(2) * cornerUncertainty * cornerUncertainty
            invR = np.linalg.inv(R)
            
            rval.rerrs[cam_id] = list()
            for i in range(0,len(target.P_t_ex)):
                p_target = target.P_t_ex[i]
                valid, y = obs.imagePoint(i)
                if valid:
                    rerr_cnt+=1
                    # Create an error term.
                    rerr = camera.model.reprojectionError(y, invR, T_camN_calib * p_target, camera.dv)
                    rerr.idx = i
                    
                    #add blake-zisserman mest
                    if useBlakeZissermanMest:
                        mest = aopt.BlakeZissermanMEstimator( 2.0 )
                        rerr.setMEstimatorPolicy(mest)
                    rval.addErrorTerm(rerr)
                    rval.rerrs[cam_id].append(rerr)
                else:
                    rval.rerrs[cam_id].append(None)

        sm.logDebug("Adding a view with {0} cameras and {1} error terms".format(len(cams_in_view), rerr_cnt))
        return rval

def removeCornersFromBatch(batch, camId_cornerIdList_tuples, useBlakeZissermanMest=True):
    #translate (camid,obs) tuple to dict
    obsdict=dict()
    for cidx, obs in batch.rig_observations:
        obsdict[cidx]=obs
       
    #disable the corners
    hasCornerRemoved=False
    for cidx, removelist in camId_cornerIdList_tuples:
        for corner_id in removelist: 
            obsdict[cidx].removeImagePoint(corner_id)
            hasCornerRemoved=True
    assert hasCornerRemoved, "need to remove at least one corner..."
    
    #rebuild problem
    new_problem = CalibrationTargetOptimizationProblem.fromTargetViewObservations(batch.cameras, 
                                                                                  batch.target, 
                                                                                  batch.baselines, 
                                                                                  batch.timestamp, 
                                                                                  batch.T_tc_guess, 
                                                                                  batch.rig_observations,
                                                                                  useBlakeZissermanMest=useBlakeZissermanMest)

    return new_problem
        
class CameraCalibration(object):
    def __init__(self, cameras, baseline_guesses, estimateLandmarks=False, verbose=False, useBlakeZissermanMest=True):
        self.cameras = cameras
        self.useBlakeZissermanMest = useBlakeZissermanMest
        #create the incremental estimator
        self.estimator = ic.IncrementalEstimator(CALIBRATION_GROUP_ID)
        self.linearSolverOptions = self.estimator.getLinearSolverOptions()
        self.optimizerOptions = self.estimator.getOptimizerOptions()
        self.target = CalibrationTarget(cameras[0].ctarget.detector.target(), estimateLandmarks=estimateLandmarks)
        self.initializeBaselineDVs(baseline_guesses)
        #storage for the used views
        self.views = list()
        
    def initializeBaselineDVs(self, baseline_guesses):
        self.baselines = list()
        for baseline_idx in range(0, len(self.cameras)-1): 
            self.baselines.append( aopt.TransformationDv(baseline_guesses[baseline_idx]) )
            
    def getBaseline(self, i):
        return self.baselines[i]
    
    def addTargetView(self, timestamp, rig_observations, T_tc_guess, force=False):
        #create the problem for this batch and try to add it 
        batch_problem = CalibrationTargetOptimizationProblem.fromTargetViewObservations(self.cameras, self.target, self.baselines, timestamp, T_tc_guess, rig_observations, useBlakeZissermanMest=self.useBlakeZissermanMest)
        self.estimator_return_value = self.estimator.addBatch(batch_problem, force)
        
        if self.estimator_return_value.numIterations >= self.optimizerOptions.maxIterations:
            sm.logError("Did not converge in maxIterations... restarting...")
            raise OptimizationDiverged
        
        success = self.estimator_return_value.batchAccepted
        if success:
            sm.logDebug("The estimator accepted this batch")
            self.views.append(batch_problem)
        else:
            sm.logDebug("The estimator did not accept this batch")
        return success

