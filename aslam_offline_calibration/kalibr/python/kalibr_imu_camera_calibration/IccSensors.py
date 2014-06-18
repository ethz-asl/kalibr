import sm
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_splines as asp
import aslam_backend as aopt
import bsplines
import kalibr_common as kc
import kalibr_errorterms as ket
import IccCalibrator as ic

import cv2
import sys
import math
import numpy as np
import pylab as pl


def initCameraBagDataset(bagfile, topic, from_to=None):
    print "Initializing camera rosbag dataset reader:"
    print "\tDataset:          {0}".format(bagfile)
    print "\tTopic:            {0}".format(topic)
    reader = kc.BagImageDatasetReader(bagfile, topic, bag_from_to=from_to)
    print "\tNumber of images: {0}".format(len(reader.index))
    return reader

def initImuBagDataset(bagfile, topic, from_to=None):
    print "Initializing imu rosbag dataset reader:"
    print "\tDataset:          {0}".format(bagfile)
    print "\tTopic:            {0}".format(topic)
    reader = kc.BagImuDatasetReader(bagfile, topic, bag_from_to=from_to)
    print "\tNumber of messages: {0}".format(len(reader.index))
    return reader


#mono camera
class IccCamera():
    def __init__(self, camConfig, targetConfig, dataset, showCorners=True, showReproj=True, showOneStep=False):
        
        #store the configuration
        self.dataset = dataset
        self.camConfig = camConfig
        self.targetConfig = targetConfig
        
        # Corner uncertainty
        self.cornerUncertainty = 1.0

        #set the extrinsic prior to default
        self.T_extrinsic = sm.Transformation()
         
        #initialize timeshift prior to zero
        self.timeshiftCamToImuPrior = 0.0
        
        #initialize the camera data
        self.camera = kc.AslamCamera.fromParameters( camConfig )
        
        #extract corners
        self.setupCalibrationTarget( targetConfig, showExtraction=showCorners, showReproj=showReproj, imageStepping=showOneStep )
        multithreading = not (showCorners or showReproj or showOneStep)
        self.targetObservations = kc.extractCornersFromDataset(self.dataset, self.detector, multithreading=multithreading)
        
    def setupCalibrationTarget(self, targetConfig, showExtraction=False, showReproj=False, imageStepping=False):
        
        #load the calibration target configuration
        targetParams = targetConfig.getTargetParams()
        targetType = targetConfig.getTargetType()
    
        if targetType == 'checkerboard':
            options = acv.CheckerboardOptions(); 
            options.showExtractionVideo = showExtraction;
            grid = acv.GridCalibrationTargetCheckerboard(targetParams['targetRows'], 
                                                            targetParams['targetCols'], 
                                                            targetParams['rowSpacingMeters'], 
                                                            targetParams['colSpacingMeters'],
                                                            options)
        elif targetType == 'circlegrid':
            options = acv.CirclegridOptions(); 
            options.showExtractionVideo = showExtraction;
            options.useAsymmetricCirclegrid = targetParams['asymmetricGrid']
            grid = acv.GridCalibrationTargetCirclegrid(targetParams['targetRows'],
                                                          targetParams['targetCols'], 
                                                          targetParams['spacingMeters'], 
                                                          options)
        elif targetType == 'aprilgrid':
            options = acv_april.AprilgridOptions(); 
            options.showExtractionVideo = showExtraction;
            options.minTagsForValidObs = int( np.max( [targetParams['tagRows'], targetParams['tagCols']] ) + 1 )
            
            grid = acv_april.GridCalibrationTargetAprilgrid(targetParams['tagRows'],
                                                            targetParams['tagCols'], 
                                                            targetParams['tagSize'], 
                                                            targetParams['tagSpacing'], 
                                                            options)
        else:
            raise RuntimeError( "Unknown calibration target." )
                          
        options = acv.GridDetectorOptions() 
        options.imageStepping = imageStepping
        options.plotCornerReprojection = showReproj
        options.filterCornerOutliers = True
        #options.filterCornerSigmaThreshold = 2.0
        #options.filterCornerMinReprojError = 0.2
        self.detector = acv.GridDetector(self.camera.geometry, grid, options)        

    def findOrientationPriorCameraToImu(self, imu):
        print
        print "Estimating imu-camera rotation prior"
        
        # build the problem
        problem = aopt.OptimizationProblem()

        # Add the rotation as design variable
        q_i_c_Dv = aopt.RotationQuaternionDv(  self.T_extrinsic.q() )
        q_i_c_Dv.setActive( True )
        problem.addDesignVariable(q_i_c_Dv)

        # Add the gyro bias as design variable
        gyroBiasDv = aopt.EuclideanPointDv( np.zeros(3) )
        gyroBiasDv.setActive( True )
        problem.addDesignVariable(gyroBiasDv)
        
        #initialize a pose spline using the camera poses
        poseSpline = self.initPoseSplineFromCamera( timeOffsetPadding=0.0 )
        
        for im in imu.imuData:
            tk = im.stamp.toSec()
            if tk > poseSpline.t_min() and tk < poseSpline.t_max():        
                #DV expressions
                R_i_c = q_i_c_Dv.toExpression()
                bias = gyroBiasDv.toExpression()   
                
                #get the vision predicted omega and measured omega (IMU)
                omega_predicted = R_i_c * aopt.EuclideanExpression( np.matrix( poseSpline.angularVelocityBodyFrame( tk ) ).transpose() )
                omega_measured = im.omega
                
                #error term
                gerr = ket.GyroscopeError(omega_measured, im.omegaInvR, omega_predicted, bias)
                problem.addErrorTerm(gerr)
        
        #define the optimization 
        options = aopt.Optimizer2Options()
        options.verbose = False
        options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()
        options.nThreads = 2
        options.convergenceDeltaX = 1e-4
        options.convergenceDeltaJ = 1
        options.maxIterations = 50

        #run the optimization
        optimizer = aopt.Optimizer2(options)
        optimizer.setProblem(problem)
        
        #get the prior
        try:
            optimizer.optimize()
        except:
            sm.logFatal("Failed to obtain orientation prior!")
            sys.exit(-1)

        #overwrite the external rotation prior (keep the external translation prior)
        R_i_c = q_i_c_Dv.toRotationMatrix().transpose()
        self.T_extrinsic = sm.Transformation( sm.rt2Transform( R_i_c, self.T_extrinsic.t() ) )

        #set the gyro bias prior (if we have more than 1 cameras use recursive average)
        b_gyro = bias.toEuclidean() 
        imu.GyroBiasPriorCount += 1
        imu.GyroBiasPrior = (imu.GyroBiasPriorCount-1.0)/imu.GyroBiasPriorCount * imu.GyroBiasPrior + 1.0/imu.GyroBiasPriorCount*b_gyro

        #print result
        print "  Orientation prior camera-imu found as: (T_i_c)"
        print R_i_c
        print "  Gyro bias prior found as: (b_gyro)"
        print b_gyro        
        
    #estimates the timeshift between the camearas and the imu using a crosscorrelation approach
    #
    #approach: angular rates are constant on a fixed body independent of location
    #          using only the norm of the gyro outputs and assuming that the biases are small
    #          we can estimate the timeshift between the cameras and the imu by calculating
    #          the angular rates of the cameras by fitting a spline and evaluating the derivatives
    #          then computing the cross correlating between the "predicted" angular rates (camera)
    #          and imu, the maximum corresponds to the timeshift...
    #          in a next step we can use the time shift to estimate the rotation between camera and imu
    def findTimeshiftCameraImuPrior(self, imu, verbose=False):
        print "Estimating time shift camera to imu:"
        
        #fit a spline to the camera observations
        poseSpline = self.initPoseSplineFromCamera( timeOffsetPadding=0.0 )
        
        #predict time shift prior 
        t=[]
        omega_measured_norm = []
        omega_predicted_norm = []
        
        for im in imu.imuData:
            tk = im.stamp.toSec()
            if tk > poseSpline.t_min() and tk < poseSpline.t_max():
                
                #get imu measurements and spline from camera
                omega_measured = im.omega
                omega_predicted = aopt.EuclideanExpression( np.matrix( poseSpline.angularVelocityBodyFrame( tk ) ).transpose() )

                #calc norm
                t = np.hstack( (t, tk) )
                omega_measured_norm = np.hstack( (omega_measured_norm, np.linalg.norm( omega_measured ) ))
                omega_predicted_norm = np.hstack( (omega_predicted_norm, np.linalg.norm( omega_predicted.toEuclidean() )) )
        
        #get the time shift
        corr = np.correlate(omega_predicted_norm, omega_measured_norm, "full")
        discrete_shift = corr.argmax() - (np.size(omega_measured_norm) - 1)
        
        #get cont. time shift
        times = [im.stamp.toSec() for im in imu.imuData]
        dT = np.mean(np.diff( times ))
        shift = -discrete_shift*dT
        
        #Create plots
        if verbose:
            pl.plot(t, omega_measured_norm, label="measured_raw")
            pl.plot(t, omega_predicted_norm, label="predicted")
            pl.plot(t-shift, omega_measured_norm, label="measured_corrected")
            pl.legend()
            pl.title("Time shift prior camera-imu estimation")
            pl.figure()
            pl.plot(corr)
            pl.title("Cross-correlation ||omega_predicted||, ||omega_measured||")
            pl.show()
            sm.logDebug("discrete time shift: {0}".format(discrete_shift))
            sm.logDebug("cont. time shift: {0}".format(shift))
            sm.logDebug("dT: {0}".format(dT))
        
        #store the timeshift (t_imu = t_cam + timeshiftCamToImuPrior)
        self.timeshiftCamToImuPrior = shift
        
        print "  Time shift camera to imu (t_imu = t_cam + shift):"
        print self.timeshiftCamToImuPrior
        
    #initialize a pose spline using camera poses (pose spline = T_wb)
    def initPoseSplineFromCamera(self, splineOrder=6, poseKnotsPerSecond=100, timeOffsetPadding=0.02):
        T_c_b = self.T_extrinsic.T()        
        pose = bsplines.BSplinePose(splineOrder, sm.RotationVector() )
                
        # Get the checkerboard times.
        times = np.array([obs.time().toSec()+self.timeshiftCamToImuPrior for obs in self.targetObservations ])                 
        curve = np.matrix([ pose.transformationToCurveValue( np.dot(obs.T_t_c().T(), T_c_b) ) for obs in self.targetObservations]).T
        
        if np.isnan(curve).any():
            raise RuntimeError("Nans in curve values")
            sys.exit(0)
        
        # Add 2 seconds on either end to allow the spline to slide during optimization
        times = np.hstack((times[0] - (timeOffsetPadding * 2.0), times, times[-1] + (timeOffsetPadding * 2.0)))
        curve = np.hstack((curve[:,0], curve, curve[:,-1]))
        
        # Make sure the rotation vector doesn't flip
        for i in range(1,curve.shape[1]):
            previousRotationVector = curve[3:6,i-1]
            r = curve[3:6,i]
            angle = np.linalg.norm(r)
            axis = r/angle
            best_r = r
            best_dist = np.linalg.norm( best_r - previousRotationVector)
            
            for s in range(-3,4):
                aa = axis * (angle + math.pi * 2.0 * s)
                dist = np.linalg.norm( aa - previousRotationVector )
                if dist < best_dist:
                    best_r = aa
                    best_dist = dist
            curve[3:6,i] = best_r;
            
        seconds = times[-1] - times[0]
        knots = int(round(seconds * poseKnotsPerSecond))
        
        print
        print "Initializing a pose spline with %d knots (%f knots per second over %f seconds)" % ( knots, poseKnotsPerSecond, seconds)
        pose.initPoseSplineSparse(times, curve, knots, 1e-4)
        return pose
    
    def addDesignVariables(self, problem, noExtrinsics=True, noTimeCalibration=True, baselinedv_group_id=ic.HELPER_GROUP_ID):
        # Add the calibration design variables.
        active = not noExtrinsics
        self.T_c_b_Dv = aopt.TransformationDv(self.T_extrinsic, rotationActive=active, translationActive=active)
        for i in range(0, self.T_c_b_Dv.numDesignVariables()):
            problem.addDesignVariable(self.T_c_b_Dv.getDesignVariable(i), baselinedv_group_id)
        
        # Add the time delay design variable.
        self.cameraTimeToImuTimeDv = aopt.Scalar(0.0)
        self.cameraTimeToImuTimeDv.setActive( not noTimeCalibration )
        problem.addDesignVariable(self.cameraTimeToImuTimeDv, ic.CALIBRATION_GROUP_ID)
        
    def addCameraErrorTerms(self, problem, poseSplineDv, T_cN_b, blakeZissermanDf=0.0, timeOffsetPadding=0.0):
        print
        print "Adding camera error terms ({0})".format(self.dataset.topic)
        
        #progress bar
        iProgress = sm.Progress2( len(self.targetObservations) )
        iProgress.sample()

        allReprojectionErrors = list()
        error_t = self.camera.reprojectionErrorType
        
        for obs in self.targetObservations:
            # Build a transformation expression for the time.
            frameTime = self.cameraTimeToImuTimeDv.toExpression() + obs.time().toSec() + self.timeshiftCamToImuPrior
            frameTimeScalar = frameTime.toScalar()
            
            #as we are applying an initial time shift outside the optimization so 
            #we need to make sure that we dont add data outside the spline definition
            if frameTimeScalar <= poseSplineDv.spline().t_min() or frameTimeScalar >= poseSplineDv.spline().t_max():
                continue
            
            T_w_b = poseSplineDv.transformationAtTime(frameTime, timeOffsetPadding, timeOffsetPadding)
            T_b_w = T_w_b.inverse()

            #calibration target coords to camera N coords
            #T_b_w: from world to imu coords
            #T_cN_b: from imu to camera N coords
            T_c_w = T_cN_b  * T_b_w
            
            #get the image and target points corresponding to the frame
            imageCornerPoints =  np.array( obs.getCornersImageFrame() ).T
            targetCornerPoints = np.array( obs.getCornersTargetFrame() ).T
            
            #setup an aslam frame (handles the distortion)
            frame = self.camera.frameType()
            frame.setGeometry(self.camera.geometry)
            
            #corner uncertainty
            R = np.eye(2) * self.cornerUncertainty * self.cornerUncertainty
            invR = np.linalg.inv(R)
            
            for pidx in range(0,imageCornerPoints.shape[1]):
                #add all image points
                k = self.camera.keypointType()
                k.setMeasurement( imageCornerPoints[:,pidx] )
                k.setInverseMeasurementCovariance(invR)
                frame.addKeypoint(k)
            
            reprojectionErrors=list()
            for pidx in range(0,imageCornerPoints.shape[1]):
                #add all target points
                targetPoint = np.insert( targetCornerPoints.transpose()[pidx], 3, 1)
                p = T_c_w *  aopt.HomogeneousExpression( targetPoint )
             
                #build and append the error term
                rerr = error_t(frame, pidx, p)
                
                #add blake-zisserman m-estimator
                if blakeZissermanDf>0.0:
                    mest = aopt.BlakeZissermanMEstimator( blakeZissermanDf )
                    rerr.setMEstimatorPolicy(mest)
                
                problem.addErrorTerm(rerr)  
                reprojectionErrors.append(rerr)
            
            allReprojectionErrors.append(reprojectionErrors)
                        
            #update progress bar
            iProgress.sample()
            
        print "\r  Added {0} camera error terms                      ".format( len(self.targetObservations) )           
        self.allReprojectionErrors = allReprojectionErrors

#pair of cameras with overlapping field of view (perfectly synced cams required!!)
#
#     Sensor "chain"                    R_C1C0 source: *fixed as input from stereo calib
#                                                      *optimized using stereo error terms
#         R_C1C0(R,t)   C1   R_C2C1(R,t)    C2         Cn
# C0  o------------------o------------------o    ...    o 
#     |
#     | R_C0I (R,t)
#     |
#     o (IMU)
#
#imu is need to initialize an orientation prior between imu and camera chain
class IccCameraChain():
    def __init__(self, chainConfig, targetConfig, parsed):

        #create all camera in the chain
        self.camList = []
        for camNr in range(0, chainConfig.numCameras()):
            camConfig = chainConfig.getCameraParameters(camNr)
            dataset = initCameraBagDataset(parsed.bagfile[0], camConfig.getRosTopic(), parsed.bag_from_to)               
            
            #create the camera
            self.camList.append( IccCamera( camConfig, 
                                            targetConfig, 
                                            dataset, 
                                            showCorners=parsed.showextraction,
                                            showReproj=parsed.showextraction, 
                                            showOneStep=parsed.extractionstepping) )  
                
        self.chainConfig = chainConfig
        
        #find and store time between first and last image over all cameras
        self.findCameraTimespan()
        
        #use stereo calibration guess if no baselines are provided
        self.initializeBaselines()
        

    def initializeBaselines(self):
        #estimate baseline prior if no external guess is provided           
        for camNr in range(1, len(self.camList)):
            self.camList[camNr].T_extrinsic = self.chainConfig.getExtrinsicsLastCamToHere(camNr)

            print "Baseline between cam{0} and cam{1} set to:".format(camNr-1,camNr)
            print "T= ", self.camList[camNr].T_extrinsic.T()
            print "Baseline: ", np.linalg.norm(self.camList[camNr].T_extrinsic.t()), " [m]"
   
    #initialize a pose spline for the chain
    def initializePoseSplineFromCameraChain(self, splineOrder=6, poseKnotsPerSecond=100, timeOffsetPadding=0.02):
        #use the main camera for the spline to initialize the poses
        return self.camList[0].initPoseSplineFromCamera()

    #find the timestamp for the first and last image considering all cameras in the chain
    def findCameraTimespan(self):
        tStart = acv.Time( 0.0 )
        tEnd = acv.Time( 0.0 )
        
        for cam in self.camList:
            if len(cam.targetObservations)>0:
                tStartCam = cam.targetObservations[0].time()
                tEndCam   = cam.targetObservations[-1].time()
                
                if tStart.toSec() > tStartCam.toSec():
                    tStart = tStartCam
                    
                if tEndCam.toSec() > tEnd.toSec():
                    tEnd = tEndCam
        
        self.timeStart = tStart
        self.timeStart = tEnd
         
    #find/set orientation prior between first camera in chain and main IMU (imu0) 
    def findOrientationPriorCameraChainToImu(self, imu):
        cam = self.camList[0]
        self.camList[0].findOrientationPriorCameraToImu( imu )

    #return the baseline transformation from camA to camB
    def getResultBaseline(self, fromCamANr, toCamBNr):
        #transformation from cam a to b is always stored in the higer ID cam
        idx = np.max([fromCamANr, toCamBNr])
        
        #get the transform from camNrmin to camNrmax
        T_cB_cA = sm.Transformation( self.camList[idx].T_c_b_Dv.T() )
        
        #get the transformation direction right
        if fromCamANr > toCamBNr:
            T_cB_cA = T_cB_cA.inverse()
        
        #calculate the metric baseline
        baseline = np.linalg.norm( T_cB_cA.t() )
    
        return T_cB_cA, baseline
    
    def getResultTrafoImuToCam(self, camNr):
        #trafo from imu to cam0 in the chain
        T_c0_i = sm.Transformation( self.camList[0].T_c_b_Dv.T() )
        
        #add all the baselines along the chain up to our camera N
        T_cN_imu = T_c0_i
        
        #now add all baselines starting from the second camera up to the desired one
        for cam in self.camList[1:camNr+1]:
            T_cNplus1_cN = sm.Transformation( cam.T_c_b_Dv.T() )
            T_cN_imu = T_cNplus1_cN*T_cN_imu

        return T_cN_imu
    
    def getResultTimeShift(self, camNr):
        return self.camList[camNr].cameraTimeToImuTimeDv.toScalar() + self.camList[camNr].timeshiftCamToImuPrior
    
    def addDesignVariables(self, problem, noTimeCalibration=True ):
        #add the design variables (T(R,t) & time)  for all induvidual cameras
        for camNr, cam in enumerate( self.camList ):
            #the first "baseline" dv is between the imu and cam0
            if camNr==0:
                noExtrinsics = False
                baselinedv_group_id = ic.CALIBRATION_GROUP_ID
            else:
                noExtrinsics = True
                baselinedv_group_id = ic.HELPER_GROUP_ID
            cam.addDesignVariables(problem, noExtrinsics, noTimeCalibration, baselinedv_group_id=baselinedv_group_id)
    
    #add the reprojection error terms for all cameras in the chain
    def addCameraChainErrorTerms(self, problem, poseSplineDv, blakeZissermanDf=-1, timeOffsetPadding=0.0):
        
        #add the induviduak error terms for all cameras
        for camNr, cam in enumerate(self.camList):
            #add error terms for the first chain element
            if camNr == 0:
                #initialize the chain with first camerea ( imu to cam0)
                T_chain = cam.T_c_b_Dv.toExpression()
            else:
                T_chain = cam.T_c_b_Dv.toExpression() * T_chain
            
            #from imu coords to camerea N coords (as DVs)
            T_cN_b = T_chain
            
            #add the error terms
            cam.addCameraErrorTerms( problem, poseSplineDv, T_cN_b, blakeZissermanDf, timeOffsetPadding )

#IMU
class IccImu():
    def __init__(self, imuConfig, parsed):
        #store input
        self.imuConfig = imuConfig
        
        #load dataset
        self.dataset = initImuBagDataset(parsed.bagfile[0], imuConfig.getRosTopic(), parsed.bag_from_to)
        
        #statistics
        self.accelUncertaintyDiscrete, self.accelRandomWalk, self.accelUncertainty = self.imuConfig.getAccelerometerStatistics()
        self.gyroUncertaintyDiscrete, self.gyroRandomWalk, self.gyroUncertainty = self.imuConfig.getGyroStatistics()
        
        #init GyroBiasPrior (+ count for recursive averaging if we have more than 1 measurement = >1 cameras)
        self.GyroBiasPrior = np.array([0,0,0])
        self.GyroBiasPriorCount = 0
        
        #imu position
        self.T_ib = sm.Transformation()
        
        #load the imu dataset
        self.loadImuData()
        
    class ImuMeasurement(object):
        def __init__(self, stamp, omega, alpha, Rgyro, Raccel):
            self.omega = omega
            self.alpha = alpha
            self.omegaR = Rgyro
            self.omegaInvR = np.linalg.inv(Rgyro)
            self.alphaR = Raccel
            self.alphaInvR = np.linalg.inv(Raccel)
            self.stamp = stamp
        
    def loadImuData(self):
        print "Reading IMU data ({0})".format(self.dataset.topic)
            
        # prepare progess bar
        iProgress = sm.Progress2( self.dataset.numMessages() )
        iProgress.sample()
        
        Rgyro = np.eye(3) * self.gyroUncertaintyDiscrete * self.gyroUncertaintyDiscrete
        Raccel = np.eye(3) * self.accelUncertaintyDiscrete * self.accelUncertaintyDiscrete
        
        # Now read the imu measurements.
        imu = []
        for timestamp, omega, alpha in self.dataset:
            timestamp = acv.Time( timestamp.toSec() ) 
            imu.append( self.ImuMeasurement(timestamp, omega, alpha, Rgyro, Raccel) )
            iProgress.sample()
        
        self.imuData = imu
        
        if len(self.imuData)>1:
            print "\r  Read %d imu readings over %.1f seconds                   " % (len(imu), imu[-1].stamp.toSec() - imu[0].stamp.toSec())
        else:
            sm.logFatal("Could not find any IMU messages. Please check the dataset.")
            sys.exit(-1)
            
    def addAccelerometerErrorTerms(self, problem, poseSplineDv, gravityExpression, mSigma=0.0, accelNoiseScale=1.0):
        print
        print "Adding accelerometer error terms ({0})".format(self.dataset.topic)
        
        #progress bar
        iProgress = sm.Progress2( len(self.imuData) )
        iProgress.sample()
        
        # AccelerometerError(measurement,  invR,  C_b_w,  acceleration_w,  bias,  g_w)
        w = 1.0/accelNoiseScale
        accelErrors = []
        num_skipped = 0
        
        if mSigma > 0.0:
            mest = aopt.HuberMEstimator(math.sqrt(mSigma))
        else:
            mest = aopt.NoMEstimator()
            
        for im in self.imuData:
            tk = im.stamp.toSec()
            if tk > poseSplineDv.spline().t_min() and tk < poseSplineDv.spline().t_max():
                C_b_w = poseSplineDv.orientation(tk).inverse()
                ak = poseSplineDv.linearAcceleration(tk)
                bk = self.accelBiasDv.toEuclideanExpression(tk,0)
                aerr = ket.AccelerometerError(im.alpha, im.alphaInvR * w, C_b_w, ak, bk, gravityExpression)
                aerr.setMEstimatorPolicy(mest)
                accelErrors.append(aerr)
                problem.addErrorTerm(aerr)
            else:
                num_skipped = num_skipped + 1

            #update progress bar
            iProgress.sample()

        print "\r  Added {0} of {1} accelerometer error terms (skipped {2} out-of-bounds measurements)".format( len(self.imuData)-num_skipped, len(self.imuData), num_skipped )
        self.accelErrors = accelErrors

    def addGyroscopeErrorTerms(self, problem, poseSplineDv, mSigma=0.0, gyroNoiseScale=1.0):
        print
        print "Adding gyroscope error terms ({0})".format(self.dataset.topic)
        
        #progress bar
        iProgress = sm.Progress2( len(self.imuData) )
        iProgress.sample()

        num_skipped = 0
        gyroErrors = []
        w = 1.0/gyroNoiseScale
        if mSigma > 0.0:
            mest = aopt.HuberMEstimator(math.sqrt(mSigma))
        else:
            mest = aopt.NoMEstimator()
            
        for im in self.imuData:
            tk = im.stamp.toSec()
            if tk > poseSplineDv.spline().t_min() and tk < poseSplineDv.spline().t_max():
                # GyroscopeError(measurement, invR, angularVelocity, bias)
                wk = poseSplineDv.angularVelocityBodyFrame(tk)
                bk = self.gyroBiasDv.toEuclideanExpression(tk,0)
                gerr = ket.GyroscopeError(im.omega, im.omegaInvR * w, wk, bk)
                #gerr.setMEstimatorPolicy(mest)
                gyroErrors.append(gerr)
                problem.addErrorTerm(gerr)
            else:
                num_skipped = num_skipped + 1

            #update progress bar
            iProgress.sample()

        print "\r  Added {0} of {1} gyroscope error terms (skipped {2} out-of-bounds measurements)".format( len(self.imuData)-num_skipped, len(self.imuData), num_skipped )           
        self.gyroErrors = gyroErrors

    def initBiasSplines(self, poseSpline, splineOrder, biasKnotsPerSecond):
        start = poseSpline.t_min();
        end = poseSpline.t_max();
        seconds = end - start;
        knots = int(round(seconds * biasKnotsPerSecond))
        
        print
        print "Initializing the bias splines with %d knots" % (knots)
        
        #initialize the bias splines
        self.gyroBias = bsplines.BSpline(splineOrder)
        self.gyroBias.initConstantSpline(start,end,knots, self.GyroBiasPrior )
        
        self.accelBias = bsplines.BSpline(splineOrder)
        self.accelBias.initConstantSpline(start,end,knots, np.zeros(3))
        
    def addDesignVariables(self, problem):
        #create design variables
        self.gyroBiasDv = asp.EuclideanBSplineDesignVariable( self.gyroBias )       
        self.accelBiasDv = asp.EuclideanBSplineDesignVariable( self.accelBias )
        
        ic.addSplineDesignVariables(problem, self.gyroBiasDv, ic.HELPER_GROUP_ID)
        ic.addSplineDesignVariables(problem, self.accelBiasDv, ic.HELPER_GROUP_ID)       

    def addBiasMotionTerms(self, problem):
        Wgyro = np.eye(3) / (self.gyroRandomWalk * self.gyroRandomWalk)
        Waccel =  np.eye(3) / (self.accelRandomWalk * self.accelRandomWalk)
        order = 1
        asp.addMotionErrorTerms(problem, self.gyroBiasDv, Wgyro, order)
        asp.addMotionErrorTerms(problem, self.accelBiasDv, Waccel, order)
