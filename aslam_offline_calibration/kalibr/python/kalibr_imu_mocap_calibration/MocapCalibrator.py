import aslam_backend as aopt
import aslam_splines as asp
import incremental_calibration as inc
import kalibr_common as kc
import sm

import gc
import numpy as np
import multiprocessing
import sys

# make numpy print prettier
np.set_printoptions(suppress=True)

CALIBRATION_GROUP_ID = 0
HELPER_GROUP_ID = 1


def addSplineDesignVariables(problem, dvc, setActive=True, group_id=HELPER_GROUP_ID):
    for i in range(0, dvc.numDesignVariables()):
        dv = dvc.designVariable(i)
        dv.setActive(setActive)
        problem.addDesignVariable(dv, group_id)


class MocapCalibrator(object):
    def __init__(self):
        self.ImuList = []

    def initDesignVariables(self, problem, poseSpline, noTimeCalibration,
                            estimateGravityLength=False, initialGravityEstimate=np.array([0.0, 9.81, 0.0])):
        # Initialize the system pose spline (always attached to imu0)
        self.poseDv = asp.BSplinePoseDesignVariable(poseSpline)
        addSplineDesignVariables(problem, self.poseDv)

        # Add the calibration target orientation design variable. (expressed as gravity vector in target frame)
        if estimateGravityLength:
            self.gravityDv = aopt.EuclideanPointDv(initialGravityEstimate)
        else:
            self.gravityDv = aopt.EuclideanDirection(initialGravityEstimate)
        self.gravityExpression = self.gravityDv.toExpression()
        self.gravityDv.setActive(True)
        problem.addDesignVariable(self.gravityDv, HELPER_GROUP_ID)

        # Add all DVs for all IMUs
        for imu in self.ImuList:
            imu.addDesignVariables(problem)

        # Add all DVs for the mocap
        self.Mocap.addDesignVariables(problem, noTimeCalibration)

    def registerMocap(self, sensor):
        self.Mocap = sensor

    def registerImu(self, sensor):
        self.ImuList.append(sensor)

    def buildProblem(self,
                     splineOrder=6,
                     poseKnotsPerSecond=70,
                     biasKnotsPerSecond=70,
                     doBiasMotionError=True,
                     huberAccel=-1,
                     huberGyro=-1,
                     noTimeCalibration=False,
                     maxIterations=20,
                     gyroNoiseScale=1.0,
                     accelNoiseScale=1.0,
                     timeOffsetPadding=0.02,
                     verbose=False):

        print("\tSpline order: %d" % (splineOrder))
        print("\tPose knots per second: %d" % (poseKnotsPerSecond))
        print("\tBias knots per second: %d" % (biasKnotsPerSecond))
        print("\tDo bias motion regularization: %s" % (doBiasMotionError))
        print("\tAcceleration Huber width (sigma): %f" % (huberAccel))
        print("\tGyroscope Huber width (sigma): %f" % (huberGyro))
        print("\tDo time calibration: %s" % (not noTimeCalibration))
        print("\tMax iterations: %d" % (maxIterations))
        print("\tTime offset padding: %f" % (timeOffsetPadding))

        ############################################
        ## initialize mocap transform
        ############################################

        # estimate the timeshift for all cameras to the main imu
        self.noTimeCalibration = noTimeCalibration
        if not noTimeCalibration:
            self.Mocap.findTimeshiftMocapImuPrior(self.ImuList[0], verbose)

        # obtain orientation prior between main imu and camera chain (if no external input provided)
        # and initial estimate for the direction of gravity
        self.Mocap.findOrientationPriorMocapToImu(self.ImuList[0])
        estimatedGravity = self.Mocap.getEstimatedGravity()

        ############################################
        ## init optimization problem
        ############################################

        # initialize a pose spline using the camera poses in the camera chain
        poseSpline = self.Mocap.initPoseSplineFromMocap(splineOrder, poseKnotsPerSecond,
                                                        timeOffsetPadding)

        # Initialize bias splines for all IMUs
        for imu in self.ImuList:
            imu.initBiasSplines(poseSpline, splineOrder, biasKnotsPerSecond)

        # Now I can build the problem
        problem = inc.CalibrationOptimizationProblem()

        # Initialize all design variables.
        self.initDesignVariables(problem, poseSpline, noTimeCalibration,
                                 initialGravityEstimate=estimatedGravity)

        ############################################
        ## add error terms
        ############################################

        # Add MOCAP pose to spline error terms.
        self.Mocap.addMocapErrorTerms(problem, self.poseDv, timeOffsetPadding=timeOffsetPadding)

        # Initialize IMU error terms.
        for imu in self.ImuList:
            imu.addAccelerometerErrorTerms(problem, self.poseDv, self.gravityExpression, mSigma=huberAccel,
                                           accelNoiseScale=accelNoiseScale)
            imu.addGyroscopeErrorTerms(problem, self.poseDv, mSigma=huberGyro, gyroNoiseScale=gyroNoiseScale,
                                       g_w=self.gravityExpression)

            # Add the bias motion terms.
            if doBiasMotionError:
                imu.addBiasMotionTerms(problem)

        # Add a gravity prior
        self.problem = problem

    def optimize(self, options=None, maxIterations=30):

        if options is None:
            options = aopt.Optimizer2Options()
            options.verbose = True
            options.doLevenbergMarquardt = True
            options.levenbergMarquardtLambdaInit = 10.0
            options.nThreads = max(1, multiprocessing.cpu_count() - 1)
            options.convergenceDeltaX = 1e-5
            options.convergenceDeltaJ = 1e-2
            options.maxIterations = maxIterations
            options.trustRegionPolicy = aopt.LevenbergMarquardtTrustRegionPolicy(options.levenbergMarquardtLambdaInit)
            options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()  # does not have multi-threading support

        # run the optimization
        self.optimizer = aopt.Optimizer2(options)
        self.optimizer.setProblem(self.problem)

        print(self.gravityDv.toEuclidean())

        optimizationFailed = False
        try:
            retval = self.optimizer.optimize()
            if retval.linearSolverFailure:
                optimizationFailed = True
        except Exception as e:
            sm.logError(str(e))
            optimizationFailed = True

        if optimizationFailed:
            sm.logError("Optimization failed!")
            raise RuntimeError("Optimization failed!")

        print(self.gravityDv.toEuclidean())

        # free some memory
        del self.optimizer
        gc.collect()

    # def saveImuSetParametersYaml(self, resultFile):
    #     imuSetConfig = kc.ImuSetParameters(resultFile, True)
    #     for imu in self.ImuList:
    #         imuConfig = imu.getImuConfig()
    #         imuSetConfig.addImuParameters(imu_parameters=imuConfig)
    #     imuSetConfig.writeYaml(resultFile)

    # def saveCamChainParametersYaml(self, resultFile):
    #     chain = self.CameraChain.chainConfig
    #     nCams = len(self.CameraChain.camList)
    #
    #     # Calibration results
    #     for camNr in range(0,nCams):
    #         #cam-cam baselines
    #         if camNr > 0:
    #             T_cB_cA, baseline = self.CameraChain.getResultBaseline(camNr-1, camNr)
    #             chain.setExtrinsicsLastCamToHere(camNr, T_cB_cA)
    #
    #         #imu-cam trafos
    #         T_ci = self.CameraChain.getResultTrafoImuToCam(camNr)
    #         chain.setExtrinsicsImuToCam(camNr, T_ci)
    #
    #         if not self.noTimeCalibration:
    #             #imu to cam timeshift
    #             timeshift = float(self.CameraChain.getResultTimeShift(camNr))
    #             chain.setTimeshiftCamImu(camNr, timeshift)
    #
    #     try:
    #         chain.writeYaml(resultFile)
    #     except:
    #         raise RuntimeError("ERROR: Could not write parameters to file: {0}\n".format(resultFile))
