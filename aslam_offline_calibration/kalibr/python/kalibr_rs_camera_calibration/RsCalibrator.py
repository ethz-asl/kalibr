#encoding:UTF-8
import sm
import aslam_backend as aopt
import aslam_cv_backend as acvb
import aslam_cv as acv
import aslam_splines as asp
import incremental_calibration as inc
import bsplines
import numpy as np
import multiprocessing
import sys
import gc
import math
from ReprojectionErrorKnotSequenceUpdateStrategy import *
from RsPlot import plotSpline
from RsPlot import plotSplineValues
import pylab as pl
import pdb

# make numpy print prettier
np.set_printoptions(suppress=True)

CALIBRATION_GROUP_ID = 0

class RsCalibratorConfiguration(object):
    deltaX = 1e-8
    """Stopping criterion for the optimizer on x"""

    deltaJ = 1e-4
    """Stopping criterion for the optimizer on J"""

    maxNumberOfIterations = 20
    """Maximum number of iterations of the batch optimizer"""

    maxKnotPlacementIterations = 10
    """Maximum number of iterations to take in the adaptive knot-placement step"""

    adaptiveKnotPlacement = True
    """Whether to enable adaptive knot placement"""

    knotUpdateStrategy = ReprojectionErrorKnotSequenceUpdateStrategy
    """The adaptive knot placement strategy to use"""

    timeOffsetConstantSparsityPattern = 0.08
    """A time offset to pad the blocks generated in the hessian/jacobian to ensure a constant symbolic representation
    of the batch estimation problem, even when a change in the shutter timing shifts the capture time to another
    spline segment.
    """

    inverseFeatureCovariance = 1/0.26
    """The inverse covariance of the feature detector. Used to standardize the error terms."""

    estimateParameters = {'shutter': True, 'intrinsics': True, 'distortion': True, 'pose': True, 'landmarks': False}
    """Which parameters to estimate. Dictionary with shutter, intrinsics, distortion, pose, landmarks as bool"""

    splineOrder = 4
    """Order of the spline to use for ct-parametrization"""

    timeOffsetPadding = 0.05
    """Time offset to add to the beginning and end of the spline to ensure we remain
    in-bounds while estimating time-depending parameters that shift the spline.
    """

    numberOfKnots = None
    """Set to an integer to start with a fixed number of uniformly distributed knots on the spline."""

    W = None
    """6x6 diagonal matrix with a weak motion prior"""

    framerate = 30
    """The approximate framerate of the camera. Required as approximate threshold in adaptive
    knot placement and for initializing a knot sequence if no number of knots is given.
    """

    def validate(self, isRollingShutter):
        """Validate the configuration."""
        # only rolling shutters can be estimated
        if (not isRollingShutter):
            self.estimateParameters['shutter'] = False
            self.adaptiveKnotPlacement = False

class RsCalibrator(object):

    __observations = None
    """Store the list of observations"""

    __cameraGeometry = None
    """The geometry container of which the calibration is performed."""

    __camera = None
    """The camera geometry itself."""

    __camera_dv = None
    """The camera design variable"""

    __cameraModelFactory = None
    """Factory object that can create a typed objects for a camera (error terms, frames, design variables etc)"""

    __poseSpline = None
    """The spline describing the pose of the camera"""

    __poseSpline_dv = None
    """The design variable representation of the pose spline of the camera"""

    __config = None
    """Configuration container \see RsCalibratorConfiguration"""

    __frames = []
    """All frames observed"""

    __reprojection_errors = []
    """Reprojection errors of the latest optimizer iteration"""

    def calibrate(self,
        cameraGeometry,
        observations,
        config
    ):
        """
        A Motion regularization term is added with low a priori knowledge to avoid
        diverging parts in the spline of too many knots are selected/provided or if
        no image information is available for long sequences and to regularize the
        last few frames (which typically contain no image information but need to have
        knots to /close/ the spline).

        Kwargs:
            cameraGeometry (kcc.CameraGeometry): a camera geometry object with an initialized target
            observations ([]: The list of observation \see extractCornersFromDataset
            config (RsCalibratorConfiguration): calibration configuration
        """

        ## set internal objects
        self.__observations = observations
        self.__cameraGeometry = cameraGeometry
        self.__cameraModelFactory = cameraGeometry.model
        self.__camera_dv = cameraGeometry.dv
        self.__camera = cameraGeometry.geometry
        self.__config = config

        self.__config.validate(self.__isRollingShutter())

        # obtain initial guesses for extrinsics and intrinsics
        if (not self.__generateIntrinsicsInitialGuess()):
            sm.logError("Could not generate initial guess.")

        # obtain the extrinsic initial guess for every observation
        self.__generateExtrinsicsInitialGuess()

        # set the value for the motion prior term or uses the defaults
        W = self.__getMotionModelPriorOrDefault()

        self.__poseSpline = self.__generateInitialSpline(
            self.__config.splineOrder,
            self.__config.timeOffsetPadding,
            self.__config.numberOfKnots,
            self.__config.framerate
        )

        # build estimator problem
        optimisation_problem = self.__buildOptimizationProblem(W)

        self.__runOptimization(
            optimisation_problem,
            self.__config.deltaJ,
            self.__config.deltaX,
            self.__config.maxNumberOfIterations
        )

        # continue with knot replacement
        if self.__config.adaptiveKnotPlacement:
            knotUpdateStrategy = self.__config.knotUpdateStrategy(self.__config.framerate)

            for iteration in range(self.__config.maxKnotPlacementIterations):

                # generate the new knots list
                [knots, requiresUpdate] = knotUpdateStrategy.generateKnotList(
                    self.__reprojection_errors,
                    self.__poseSpline_dv.spline()
                )
                # if no new knotlist was generated, we are done.
                if (not requiresUpdate):
                    break;

                # otherwise update the spline dv and rebuild the problem
                self.__poseSpline = knotUpdateStrategy.getUpdatedSpline(self.__poseSpline_dv.spline(), knots, self.__config.splineOrder)

                optimisation_problem = self.__buildOptimizationProblem(W)
                self.__runOptimization(
                    optimisation_problem,
                    self.__config.deltaJ,
                    self.__config.deltaX,
                    self.__config.maxNumberOfIterations
                )

        self.__printResults()

    def __generateExtrinsicsInitialGuess(self):
        """Estimate the pose of the camera with a PnP solver. Call after initializing the intrinsics"""
        # estimate and set T_c in the observations
        for idx, observation in enumerate(self.__observations):
            (success, T_t_c) = self.__camera.estimateTransformation(observation)
            if (success):
                observation.set_T_t_c(T_t_c)
            else:
                sm.logWarn("Could not estimate T_t_c for observation at index" . idx)

        return

    def __generateIntrinsicsInitialGuess(self):
        """
        Get an initial guess for the camera geometry (intrinsics, distortion). Distortion is typically left as 0,0,0,0.
        The parameters of the geometryModel are updated in place.
        """
        if (self.__isRollingShutter()):
            sensorRows = self.__observations[0].imRows()
            self.__camera.shutter().setParameters(np.array([1.0 / self.__config.framerate / float(sensorRows)]))

        return self.__camera.initializeIntrinsics(self.__observations)

    def __getMotionModelPriorOrDefault(self):
        """Get the motion model prior or the default value"""
        W = self.__config.W
        if W is None:
            W = np.eye(6)
            W[:3,:3] *= 1e-3
            W[3:,3:] *= 1
            W *= 1e-2
        return W

    def __generateInitialSpline(self, splineOrder, timeOffsetPadding, numberOfKnots = None, framerate = None):
        poseSpline = bsplines.BSplinePose(splineOrder, sm.RotationVector())

        # Get the observation times.
        times = np.array([observation.time().toSec() for observation in self.__observations ])
        # get the pose values of the initial transformations at observation time
        curve = np.matrix([ poseSpline.transformationToCurveValue( observation.T_t_c().T() ) for observation in self.__observations]).T
        # make sure all values are well defined
        if np.isnan(curve).any():
            raise RuntimeError("Nans in curve values")
            sys.exit(0)
        # Add 2 seconds on either end to allow the spline to slide during optimization
        times = np.hstack((times[0] - (timeOffsetPadding * 2.0), times, times[-1] + (timeOffsetPadding * 2.0)))
        curve = np.hstack((curve[:,0], curve, curve[:,-1]))

        self.__ensureContinuousRotationVectors(curve)

        seconds = times[-1] - times[0]

        # fixed number of knots
        if (numberOfKnots is not None):
            knots = numberOfKnots
        # otherwise with framerate estimate
        else:
            knots = int(round(seconds * framerate/3))

        print
        print "Initializing a pose spline with %d knots (%f knots per second over %f seconds)" % ( knots, 100, seconds)
        poseSpline.initPoseSplineSparse(times, curve, knots, 1e-4)

        return poseSpline

    def __buildOptimizationProblem(self, W):
        """Build the optimisation problem"""
        problem = inc.CalibrationOptimizationProblem()

        # Initialize all design variables.
        self.__initPoseDesignVariables(problem)

        #####
        ## build error terms and add to problem

        # store all frames
        self.__frames = []
        self.__reprojection_errors = []

        # This code assumes that the order of the landmarks in the observations
        # is invariant across all observations. At least for the chessboards it is true.

        #####
        # add all the landmarks once
        landmarks = []
        landmarks_expr = []
        for landmark in self.__observations[0].getCornersTargetFrame():
            # design variable for landmark
            landmark_w_dv = aopt.HomogeneousPointDv(sm.toHomogeneous(landmark))
            landmark_w_dv.setActive(self.__config.estimateParameters['landmarks']);
            landmarks.append(landmark_w_dv)
            landmarks_expr.append(landmark_w_dv.toExpression())
            problem.addDesignVariable(landmark_w_dv, CALIBRATION_GROUP_ID)

        #####
        # activate design variables
        self.__camera_dv.setActive(
            self.__config.estimateParameters['intrinsics'],
            self.__config.estimateParameters['distortion'],
            self.__config.estimateParameters['shutter']
        )

        #####
        # Add design variables

        # add the camera design variables last for optimal sparsity patterns
        problem.addDesignVariable(self.__camera_dv.shutterDesignVariable(), CALIBRATION_GROUP_ID)
        problem.addDesignVariable(self.__camera_dv.projectionDesignVariable(), CALIBRATION_GROUP_ID)
        problem.addDesignVariable(self.__camera_dv.distortionDesignVariable(), CALIBRATION_GROUP_ID)

        #####
        # Regularization term / motion prior
        motionError = asp.BSplineMotionError(self.__poseSpline_dv, W)
        problem.addErrorTerm(motionError)

        #####
        # add a reprojection error for every corner of each observation
        for observation in self.__observations:
            # only process successful observations of a pattern
            if (observation.hasSuccessfulObservation()):
                # add a frame
                frame = self.__cameraModelFactory.frameType()
                frame.setGeometry(self.__camera)
                frame.setTime(observation.time())
                self.__frames.append(frame)

                #####
                # add an error term for every observed corner
                for index, point in enumerate(observation.getCornersImageFrame()):
                    # keypoint time offset by line delay as expression type
                    keypoint_time = self.__camera_dv.keypointTime(frame.time(), point)

                    # from target to world transformation.
                    T_w_t = self.__poseSpline_dv.transformationAtTime(
                        keypoint_time,
                        self.__config.timeOffsetConstantSparsityPattern,
                        self.__config.timeOffsetConstantSparsityPattern
                    )
                    T_t_w = T_w_t.inverse()

                    # transform target point to camera frame
                    p_t = T_t_w * landmarks_expr[index]

                    # create the keypoint
                    keypoint = acv.Keypoint2()
                    keypoint.setMeasurement(point)
                    inverseFeatureCovariance = self.__config.inverseFeatureCovariance;
                    keypoint.setInverseMeasurementCovariance(np.eye(len(point)) * inverseFeatureCovariance)
                    keypoint.setLandmarkId(index)
                    frame.addKeypoint(keypoint)
                    keypoint_index = frame.numKeypoints() - 1

                    # create reprojection error
                    reprojection_error = self.__buildErrorTerm(
                        frame,
                        keypoint_index,
                        p_t,
                        self.__camera_dv,
                        self.__poseSpline_dv
                    )
                    self.__reprojection_errors.append(reprojection_error)
                    problem.addErrorTerm(reprojection_error)

        return problem

    def __buildErrorTerm(self, frame, keypoint_index, p_t, camera_dv, poseSpline_dv):
        """
        Build an error term that considers the shutter type. A Global Shutter camera gets the standard reprojection error
        a Rolling Shutter gets the adaptive covariance error term that considers the camera motion.
        """
        # it is a global shutter camera -> no covariance error
        if (self.__isRollingShutter()):
            return self.__cameraModelFactory.reprojectionErrorAdaptiveCovariance(
                frame,
                keypoint_index,
                p_t,
                camera_dv,
                poseSpline_dv
            )
        else:
            return self.__cameraModelFactory.reprojectionError(
                frame,
                keypoint_index,
                p_t,
                camera_dv
            )

    def __ensureContinuousRotationVectors(self, curve):
        """
        Ensures that the rotation vector does not flip and enables a continuous trajectory modeling.
        Updates curves in place.
        """
        for i in range(1, curve.shape[1]):
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

    def __initPoseDesignVariables(self, problem):
        """Get the design variable representation of the pose spline and add them to the problem"""
        # get the design variable
        self.__poseSpline_dv = asp.BSplinePoseDesignVariable(self.__poseSpline)
        # activate all contained dv and add to problem
        for i in range(0, self.__poseSpline_dv.numDesignVariables()):
            dv = self.__poseSpline_dv.designVariable(i)
            dv.setActive(self.__config.estimateParameters['pose'])
            problem.addDesignVariable(dv, CALIBRATION_GROUP_ID)

    def __runOptimization(self, problem ,deltaJ, deltaX, maxIt):
        """Run the given optimization problem problem"""

        print "run new optimisation with initial values:"
        self.__printResults()

        # verbose and choldmod solving with schur complement trick
        options = aopt.Optimizer2Options()
        options.verbose = True
        options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()
        options.doSchurComplement = True

        # stopping criteria
        options.maxIterations = maxIt
        options.convergenceDeltaJ = deltaJ
        options.convergenceDeltaX = deltaX

        # use the dogleg trustregion policy
        options.trustRegionPolicy = aopt.DogLegTrustRegionPolicy()

        # create the optimizer
        optimizer = aopt.Optimizer2(options)
        optimizer.setProblem(problem)

        # go for it:
        return optimizer.optimize()

    def __isRollingShutter(self):
        return self.__cameraModelFactory.shutterType == acv.RollingShutter

    def __printResults(self):
        shutter = self.__camera_dv.shutterDesignVariable().value()
        proj = self.__camera_dv.projectionDesignVariable().value()
        dist = self.__camera_dv.distortionDesignVariable().value()
        print
        if (self.__isRollingShutter()):
            print "LineDelay:"
            print shutter.lineDelay()
        print "Intrinsics:"
        print proj.getParameters().flatten()
        #print "(",proj.fu(),", ",proj.fv(),") (",proj.cu(),", ",proj.cv(),")" #in the future, not all projection models might support these parameters
        print "Distortion:"
        print dist.getParameters().flatten()
        #print "(",dist.p1(),", ",dist.p2(),") (",dist.k1(),", ",dist.k2(),")" #not all distortion models implement these parameters
