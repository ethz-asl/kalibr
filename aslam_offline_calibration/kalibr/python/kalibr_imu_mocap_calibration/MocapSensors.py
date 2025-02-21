from __future__ import print_function  # handle print in 2.x python
import sm
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_splines as asp
import aslam_backend as aopt
import bsplines
import kalibr_common as kc
import kalibr_errorterms as ket
from . import MocapCalibrator as im
from .MocapCalibrator import *

import cv2
import sys
import math
import numpy as np
import pylab as pl
import scipy.optimize


def initMocapBagDataset(bagfile, topic, from_to, perform_synchronization):
    print("Initializing mocap rosbag dataset reader:")
    print("\tDataset:          {0}".format(bagfile))
    print("\tTopic:            {0}".format(topic))
    reader = kc.BagMocapDatasetReader(bagfile, topic, bag_from_to=from_to,
                                      perform_synchronization=perform_synchronization)
    print("\tNumber of mocap: {0}".format(len(reader.index)))
    return reader


# mocap pose sensor
class IccMocap():
    def __init__(self, parsed):

        # noises used
        self.oriUncertainty = parsed.ori_sigma
        self.posUncertainty = parsed.pos_sigma

        # store the configuration
        self.dataset = initMocapBagDataset(parsed.bagfile[0], parsed.topic, parsed.bag_from_to,
                                           parsed.perform_synchronization)

        # initialize timeshift prior to zero
        self.timeshiftMocapToImuPrior = 0.0

        # set the extrinsic prior to default
        self.T_extrinsic = sm.Transformation()
        # base_transform = np.array([[0.338087, 0.00183229, 0.941113, 0.0701093],
        #                            [0.0298411, -0.999516, -0.00877416, -0.0701093],
        #                            [0.940642, 0.0310503, -0.337978, -0.128204],
        #                            [0.0, 0.0, 0.0, 1.0]])
        # self.T_extrinsic = sm.Transformation(base_transform)

        # load the mocap dataset
        self.loadMocapData()

        # an estimate of the gravity in the world coordinate frame
        self.gravity_w = np.array([9.80655, 0., 0.])

    class MocapMeasurement(object):
        def __init__(self, stamp, quat, pos):
            # There is a required conjugation between Hamilton's quaternion and JPL convention.
            quat[:3] = -quat[:3]
            self.T_w_m = sm.Transformation(quat, pos)
            self.stamp = stamp

    def loadMocapData(self):
        print("Reading MOCAP data ({0})".format(self.dataset.topic))

        # prepare progess bar
        iProgress = sm.Progress2(self.dataset.numMessages())
        iProgress.sample()

        # Now read the imu measurements.
        data = []
        for timestamp, quat, pos in self.dataset:
            timestamp = acv.Time(timestamp.toSec())
            data.append(self.MocapMeasurement(timestamp, quat, pos))
            iProgress.sample()

        self.mocapData = data

        if len(self.mocapData) > 1:
            print("\r  Read %d mocap readings over %.1f seconds                   "
                  % (len(data), data[-1].stamp.toSec() - data[0].stamp.toSec()))
        else:
            sm.logFatal("Could not find any MOCAP messages. Please check the dataset.")
            sys.exit(-1)

    def findOrientationPriorMocapToImu(self, imu):
        print("")
        print("Estimating mocap-imu rotation prior")

        # build the problem
        problem = aopt.OptimizationProblem()

        # Add the rotation as design variable
        q_i_m_Dv = aopt.RotationQuaternionDv(self.T_extrinsic.q())
        q_i_m_Dv.setActive(True)
        problem.addDesignVariable(q_i_m_Dv)

        # Add the gyro bias as design variable
        gyroBiasDv = aopt.EuclideanPointDv(np.zeros(3))
        gyroBiasDv.setActive(True)
        problem.addDesignVariable(gyroBiasDv)

        # initialize a pose spline using the mocap poses
        poseSpline = self.initPoseSplineFromMocap(timeOffsetPadding=0.0)
        for im in imu.imuData:
            tk = im.stamp.toSec()
            if tk > poseSpline.t_min() and tk < poseSpline.t_max():
                # DV expressions
                R_i_m = q_i_m_Dv.toExpression()
                bias = gyroBiasDv.toExpression()

                # get the vision predicted omega and measured omega (IMU)
                omega_predicted = R_i_m * aopt.EuclideanExpression(
                    np.matrix(poseSpline.angularVelocityBodyFrame(tk)).transpose())
                omega_measured = im.omega

                # error term
                gerr = ket.GyroscopeError(omega_measured, im.omegaInvR, omega_predicted, bias)
                problem.addErrorTerm(gerr)

        if problem.numErrorTerms() == 0:
            sm.logFatal("Failed to obtain orientation prior. "
                        "Please make sure that your sensors are synchronized correctly.")
            sys.exit(-1)

        # define the optimization
        options = aopt.Optimizer2Options()
        options.verbose = False
        options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()  # does not have multi-threading support
        options.nThreads = 2
        options.convergenceDeltaX = 1e-4
        options.convergenceDeltaJ = 1
        options.maxIterations = 50

        # run the optimization
        optimizer = aopt.Optimizer2(options)
        optimizer.setProblem(problem)

        # get the prior
        try:
            optimizer.optimize()
        except:
            sm.logFatal("Failed to obtain orientation prior!")
            sys.exit(-1)

        # overwrite the external rotation prior (keep the external translation prior)
        R_m_i = q_i_m_Dv.toRotationMatrix().transpose()
        self.T_extrinsic = sm.Transformation(sm.rt2Transform(R_m_i, self.T_extrinsic.t()))

        # estimate gravity in the world coordinate frame as the mean specific force
        # NOTE: we re-initalize the spline here so it is now in the IMU sensor frame
        poseSpline = self.initPoseSplineFromMocap(timeOffsetPadding=0.0)
        a_w = []
        for im in imu.imuData:
            tk = im.stamp.toSec()
            if tk > poseSpline.t_min() and tk < poseSpline.t_max():
                # a_w.append(np.dot(poseSpline.orientation(tk), np.dot(R_m_i, - im.alpha)))
                a_w.append(np.dot(poseSpline.orientation(tk), - im.alpha))
        mean_a_w = np.mean(np.asarray(a_w).T, axis=1)
        self.gravity_w = mean_a_w / np.linalg.norm(mean_a_w) * 9.80655
        print("Gravity was initialized to", self.gravity_w, "[m/s^2]")

        # set the gyro bias prior (if we have more than 1 cameras use recursive average)
        b_gyro = bias.toEuclidean()
        imu.GyroBiasPriorCount += 1
        imu.GyroBiasPrior = (
                                    imu.GyroBiasPriorCount - 1.0) / imu.GyroBiasPriorCount * imu.GyroBiasPrior + 1.0 / imu.GyroBiasPriorCount * b_gyro

        # print result
        print("  Orientation prior mocap-imu found as: (T_m_i)")
        print(R_m_i)
        print("  Gyro bias prior found as: (b_gyro)")
        print(b_gyro)

    # return an etimate of gravity in the world coordinate frame as perceived by this camera
    def getEstimatedGravity(self):
        return self.gravity_w

    # estimates the timeshift between the mocap and the imu using a cross-correlation approach
    #
    # approach: angular rates are constant on a fixed body independent of location
    #          using only the norm of the gyro outputs and assuming that the biases are small
    #          we can estimate the timeshift between the cameras and the imu by calculating
    #          the angular rates of the cameras by fitting a spline and evaluating the derivatives
    #          then computing the cross correlating between the "predicted" angular rates (camera)
    #          and imu, the maximum corresponds to the timeshift...
    #          in a next step we can use the time shift to estimate the rotation between camera and imu
    def findTimeshiftMocapImuPrior(self, imu, verbose=False):
        print("Estimating time shift mocap to imu:")

        # fit a spline to the camera observations
        poseSpline = self.initPoseSplineFromMocap(timeOffsetPadding=0.0)

        # predict time shift prior
        t = []
        omega_measured_norm = []
        omega_predicted_norm = []
        for im in imu.imuData:
            tk = im.stamp.toSec()
            if tk > poseSpline.t_min() and tk < poseSpline.t_max():
                # get imu measurements and spline from camera
                omega_measured = im.omega
                omega_predicted = aopt.EuclideanExpression(
                    np.matrix(poseSpline.angularVelocityBodyFrame(tk)).transpose())

                # calc norm
                t = np.hstack((t, tk))
                omega_measured_norm = np.hstack((omega_measured_norm, np.linalg.norm(omega_measured)))
                omega_predicted_norm = np.hstack((omega_predicted_norm, np.linalg.norm(omega_predicted.toEuclidean())))

        if len(omega_predicted_norm) == 0 or len(omega_measured_norm) == 0:
            sm.logFatal("The time ranges of the mocap and IMU do not overlap. "
                        "Please make sure that your sensors are synchronized correctly.")
            sys.exit(-1)

        # get the time shift
        corr = np.correlate(omega_predicted_norm, omega_measured_norm, "full")
        discrete_shift = corr.argmax() - (np.size(omega_measured_norm) - 1)

        # get cont. time shift
        times = [im.stamp.toSec() for im in imu.imuData]
        dT = np.mean(np.diff(times))
        shift = -discrete_shift * dT

        # Create plots
        if verbose:
            pl.plot(t, omega_measured_norm, label="measured_raw")
            pl.plot(t, omega_predicted_norm, label="predicted")
            pl.plot(t - shift, omega_measured_norm, label="measured_corrected")
            pl.legend()
            pl.title("Time shift prior mocap-imu estimation")
            pl.figure()
            pl.plot(corr)
            pl.title("Cross-correlation ||omega_predicted||, ||omega_measured||")
            pl.show()
            sm.logDebug("discrete time shift: {0}".format(discrete_shift))
            sm.logDebug("cont. time shift: {0}".format(shift))
            sm.logDebug("dT: {0}".format(dT))

        # store the timeshift (t_imu = t_mocap + timeshiftMocapToImuPrior)
        self.timeshiftMocapToImuPrior = shift
        print("  Time shift mocap to imu (t_imu = t_mocap + shift):")
        print(self.timeshiftMocapToImuPrior)

    # initialize a pose spline using camera poses (pose spline = T_wb)
    def initPoseSplineFromMocap(self, splineOrder=6, poseKnotsPerSecond=100, timeOffsetPadding=0.02):

        T_m_b = self.T_extrinsic.T()
        pose = bsplines.BSplinePose(splineOrder, sm.RotationVector())

        # Get the checkerboard times.
        times = np.array([obs.stamp.toSec() + self.timeshiftMocapToImuPrior for obs in self.mocapData])
        curve = np.matrix(
            [pose.transformationToCurveValue(np.dot(obs.T_w_m.T(), T_m_b)) for obs in self.mocapData]).T

        if np.isnan(curve).any():
            raise RuntimeError("Nans in curve values")
            sys.exit(0)

        # Add 2 seconds on either end to allow the spline to slide during optimization
        times = np.hstack((times[0] - (timeOffsetPadding * 2.0), times, times[-1] + (timeOffsetPadding * 2.0)))
        curve = np.hstack((curve[:, 0], curve, curve[:, -1]))

        # Make sure the rotation vector doesn't flip
        for i in range(1, curve.shape[1]):
            previousRotationVector = curve[3:6, i - 1]
            r = curve[3:6, i]
            angle = np.linalg.norm(r)
            axis = r / angle
            best_r = r
            best_dist = np.linalg.norm(best_r - previousRotationVector)

            for s in range(-3, 4):
                aa = axis * (angle + math.pi * 2.0 * s)
                dist = np.linalg.norm(aa - previousRotationVector)
                if dist < best_dist:
                    best_r = aa
                    best_dist = dist
            curve[3:6, i] = best_r

        seconds = times[-1] - times[0]
        knots = int(round(seconds * poseKnotsPerSecond))

        print("")
        print("Initializing a pose spline with %d knots (%f knots per second over %f seconds)" % (
            knots, poseKnotsPerSecond, seconds))
        pose.initPoseSplineSparse(times, curve, knots, 1e-4)
        return pose

    def addDesignVariables(self, problem, noTimeCalibration=True):
        # Add the calibration design variables.
        active = True
        self.T_m_b_Dv = aopt.TransformationDv(self.T_extrinsic, rotationActive=active, translationActive=active)
        for i in range(0, self.T_m_b_Dv.numDesignVariables()):
            problem.addDesignVariable(self.T_m_b_Dv.getDesignVariable(i), CALIBRATION_GROUP_ID)

        # Add the time delay design variable.
        self.mocapTimeToImuTimeDv = aopt.Scalar(0.0)
        self.mocapTimeToImuTimeDv.setActive(not noTimeCalibration)
        problem.addDesignVariable(self.mocapTimeToImuTimeDv, CALIBRATION_GROUP_ID)

    def addMocapErrorTerms(self, problem, poseSplineDv, timeOffsetPadding=0.0, mSigma=0.0):

        print("")
        print("Adding mocap error terms ({0})".format(self.dataset.topic))

        # progress bar
        iProgress = sm.Progress2(len(self.mocapData))
        iProgress.sample()

        if mSigma > 0.0:
            mest = aopt.HuberMEstimator(mSigma)
        else:
            mest = aopt.NoMEstimator()

        mocapErrors = []
        for md in self.mocapData:

            # Build a transformation expression for the time.
            frameTime = self.mocapTimeToImuTimeDv.toExpression() + md.stamp.toSec() + self.timeshiftMocapToImuPrior
            frameTimeScalar = frameTime.toScalar()

            # as we are applying an initial time shift outside the optimization so
            # we need to make sure that we dont add data outside the spline definition
            if frameTimeScalar <= poseSplineDv.spline().t_min() or frameTimeScalar >= poseSplineDv.spline().t_max():
                continue

            # now lets construct our constraint for this mocap observation
            # mocap provides measurement of marker frame at given time
            # T_b_w: from world to imu coords
            # T_m_b: from imu to mocap marker
            T_w_b = poseSplineDv.transformationAtTime(frameTime, timeOffsetPadding, timeOffsetPadding)
            # T_w_b = poseSplineDv.transformation(frameTimeScalar)
            T_b_m = self.T_m_b_Dv.toExpression().inverse()
            T_w_m = T_w_b * T_b_m
            err = aopt.ErrorTermTransformation(T_w_m, md.T_w_m, 1.0 / self.oriUncertainty, 1.0 / self.posUncertainty)
            # Rtmp = np.eye(3) * self.posUncertainty * self.posUncertainty
            # err = ket.EuclideanError(md.T_w_m.t(), np.linalg.inv(Rtmp), T_w_m.t())
            err.setMEstimatorPolicy(mest)
            mocapErrors.append(err)
            problem.addErrorTerm(err)

            # update progress bar
            iProgress.sample()

        print("\r  Added {0} of {1} mocap error terms        ".format(len(mocapErrors),
                                                                      len(self.mocapData)))
        self.mocapErrors = mocapErrors

    def getResultTrafoImuToMocap(self):
        return sm.Transformation(self.T_m_b_Dv.T())

    def getResultTimeShift(self):
        return self.mocapTimeToImuTimeDv.toScalar() + self.timeshiftMocapToImuPrior
