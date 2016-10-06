import sm
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_splines as asp
import aslam_backend as aopt
import bsplines
import kalibr_common as kc
import kalibr_errorterms as ket
import laser_errorterms as let
import IccCalibrator as ic

import cv2
import sys
import math
import numpy as np
import pylab as pl

from scipy import stats

import random
from matplotlib import rc
# make numpy print prettier
np.set_printoptions(suppress=True)
rc('text', usetex=True)

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def initScanBagDataset(bagfile, topic, from_to=None):
    print "Initializing imu rosbag dataset reader:"
    print "\tDataset:          {0}".format(bagfile)
    print "\tTopic:            {0}".format(topic)
#     reader = kc.BagScanDatasetReader(bagfile, topic, bag_from_to=from_to)
    reader = kc.BagTriggeredScanDatasetReader(bagfile, topic, '/external_trigger', 1, \
                                              indexoffset = -1, bag_from_to=from_to)
    print "\tNumber of messages: {0}".format(len(reader.index))
    return reader

#LRF
class IccLaser:
    def __init__(self, parsed, distanceSigma=7.5e-3):
        self.dataset = initScanBagDataset(parsed.bagfile[0], parsed.scanTopic, parsed.bag_from_to)

        #TODO move all these parameters to a configuration yaml
        self.qInit = np.array(parsed.q_bl)
        self.tInit = np.array(parsed.t_bl)

        self.distanceUncertainty = distanceSigma
        self.ransacThreshold = .1
        self.lineOfSightErrorThreshold = 2. * self.ransacThreshold
        self.bzThresholdSigmas = 4.0 #2.0
        self.minimumPlaneSupport = 100
        self.seedFactor = 0.1
        self.removeFactor = 0.1
        self.pruningFactor = 0.01
        self.timeOffsetPadding = parsed.timeoffset_padding

        self.loadLaserData()

    class LaserMeasurement(object):
        def __init__(self, stamp, angle, distance, R):
            self.stamp = stamp
            self.R = R
            self.invR = np.array([1.0/R[0]])
            self.angle = angle
            self.distance = distance
        
    class Plane(object):
        def __init__(self, n, d, inliers, errorTerms = [], isValid = True):
            self.n = n
            self.d = d
            self.inliers = inliers
            self.errorTerms = errorTerms
            self.valid = isValid
            self.n_Dv = None
            self.d_Dv = None

    def subsampleData(self, data, pruningFactor):
        if pruningFactor > 1.0:
            raise Exception('Factor exceeds 1.0.')
        return [ data[i] for i in sorted(random.sample( \
                    xrange(len(data)), int(math.ceil(len(data) * pruningFactor)))) ]

    def loadLaserData(self):
        print "Reading laser data ({0})".format(self.dataset.topic)
        
        Rlaser = np.array([self.distanceUncertainty**2])
        
        # Now read the laser measurements.
        laserData = []
        for (timestamp, angle, distance) in self.dataset:
            if not np.isnan(distance) and not np.isinf(distance):
                laserData.append( self.LaserMeasurement(timestamp, angle, distance, Rlaser) )

        self.laserData = self.subsampleData(laserData, self.pruningFactor)

        if len(self.laserData) > 1:
            print "\r  Read %d laser readings over %.1f seconds                   " \
                  % (len(self.laserData), self.laserData[-1].stamp.toSec() - self.laserData[0].stamp.toSec())
        else:
            sm.logFatal("Could not find any scan messages. Please check the dataset.")
            sys.exit(-1)


    def findClosestPoints(self, points, index, distanceThreshold = .1):
        closestPoints = []
        p = points - np.tile(points[:,index:index+1], (1, points.shape[1]))
        dSquared = np.sum(np.square(p),axis=0)
        #closestPoints = np.where(np.logical_and(dSquared <= (distanceThreshold*distanceThreshold), dSquared > 0.0))
        closestPoints = np.where(dSquared <= (distanceThreshold*distanceThreshold))
        return closestPoints[0].tolist(), dSquared

    #TODO(jrn) region growing does not seem to capture all points of a plane.
        #Implement a more suitable approach based on splitting and merging?
    def growRegion(self, points, inliers, notVisited, index, distanceThreshold=.1, \
                   distanceFactor=.1, recursionDepth=0, maxRecursionDepth=250):

        notVisited = notVisited - set([index])

        closestPoints, distanceSquared = self.findClosestPoints(points, index,\
                                                          distanceThreshold)

#         testPoints, testDistanceSquared = self.findClosestPoints(points, index,\
#                                                           .5*distanceThreshold)

        if not closestPoints:
            return [index], notVisited

        if not (set(closestPoints) <= inliers) :
            return [], notVisited
#         if not (set(testPoints) <= inliers) :        #different threshold to relax growing criterion
#             return [], notVisited

        if recursionDepth is maxRecursionDepth:
            return [index], notVisited

        region = [index]
        for i in closestPoints:
            if (i in notVisited) and (distanceSquared[i] < (distanceFactor * distanceThreshold)**2) :
                indices, notVisited = self.growRegion(points=points, inliers=set(inliers), notVisited=set(notVisited), index=i, \
                                                      distanceThreshold=distanceThreshold, distanceFactor=distanceFactor, \
                                                      recursionDepth=recursionDepth+1, maxRecursionDepth=maxRecursionDepth)
                if indices:
                    region = region + indices

        return list(region), set(notVisited)

    def refinePlane(self, points, inliers, threshold):
        if len(inliers) == 0:
            return (None, None, [])
        
        inlierPoints = points[:, np.asarray(inliers)]
        centroid = np.mean(inlierPoints, axis=1)
        p = inlierPoints - np.tile(np.array([centroid]).T, (1, inlierPoints.shape[1]))
        #The SVD might not converge, so catch this exception here!
        try:
            u,s,v = np.linalg.svd(p)
        except:
            return (None, None, [])
        n = u[:,2:3]
        d = np.dot(n.T, centroid)[0]
        if d < 0:
            n = -n
            d = -d
        distances = np.fabs(np.dot(n.T, points) - d)
        inliers = np.where(distances <= threshold)[1].tolist()
        return (n, d, inliers)

    #This method implemnets a test of whether the plane is valid or has been induced for example by
    # the scanning plane. It is a hand-tuned heuristic and open for discussions.
    def isValidPlane(self, points):

        if points.shape[1] < self.minimumPlaneSupport :
            return False

        centroid = np.mean(points,axis=1)
        p = points - np.tile(np.array([centroid]).T, (1, points.shape[1]))
        u,s,v = np.linalg.svd(p)
        rotatedPoints = np.dot(u.T, p);
        scaledPoints = np.dot(np.linalg.inv(np.diag(s)), rotatedPoints)
        
        m2 = np.sqrt(stats.moment(rotatedPoints, 2, axis=1))
        m2 = m2[0:2]/m2[2]
#         print m2
        ks = np.array([stats.kstest(scaledPoints[0,:], 'uniform')[0], \
                       stats.kstest(scaledPoints[1,:], 'uniform')[0],\
                       stats.kstest(scaledPoints[2,:], 'uniform')[0]])
#         print ks

        if np.all(m2 > 4.0) and np.all(ks > 0.85):   
            return True
        else:
            return False

    def transformMeasurementsToWorldFrame(self, poseSplineDv):
        points = []
        laserData = [] #maintain a list of all measurements considered in the point cloud
        for l in self.laserData:
            tk = l.stamp.toSec() + self.laserOffsetDv.toScalar()
            if tk > poseSplineDv.spline().t_min() and tk < poseSplineDv.spline().t_max():
               T_w_b = poseSplineDv.transformation(tk)
               p_l = np.array([l.distance * np.cos(l.angle), l.distance * np.sin(l.angle), 0.0, 1.0])
               p_w = np.dot(T_w_b.toTransformationMatrix(), np.dot(self.T_b_l_Dv.toTransformationMatrix(), p_l))
               points.append(p_w[0:3])
               laserData.append(l)

        return (laserData, np.asarray(points, dtype=float).T)

    def findPlanes(self, poseSplineDv, plotInitialPointCloud = True):

        (self.laserData, self.pointCloud) = self.transformMeasurementsToWorldFrame(poseSplineDv)
        pointCloudIndices = range(0, self.pointCloud.shape[1])
        
        if plotInitialPointCloud:
            self.plotPlanes(self.pointCloud, planes=[], figNr=1, plotNonPlanePoints=True, \
                visualizeError=False, block=True)

        planes = []

        while True:
            #Identify points that are not part of a plane yet
            planeIndices = [i for p in planes for i in p.inliers]            
            pointIndices = list(set(pointCloudIndices) - set(planeIndices))
            points = self.pointCloud[:, pointIndices]

#             print "length of point indices %d" % len(pointIndices)

            if len(pointIndices) <= self.minimumPlaneSupport:
                break

            #simple plane hyptothesizing
            (inliers, distances) = self.findClosestPoints(points, random.choice(range(points.shape[1])), \
                                                    self.ransacThreshold)
            (n, d, inliers) = self.refinePlane(points, inliers, self.ransacThreshold)

            if len(inliers) == 0:
                planes.append(self.Plane(n, d, \
                         self.subsampleData(pointIndices, self.removeFactor), \
                         [], isValid = False))  
                continue

            seeds = self.subsampleData(list(inliers), self.seedFactor)
            random.shuffle(seeds)

            regionInliers = []
            for seed in seeds:
                if seed not in regionInliers:
                    tmpRegionInliers, notVisited = self.growRegion(points=points[:], \
                                                                   inliers=set(inliers[:]), \
                                                                   notVisited=set(inliers[:]), \
                                                                   index=seed, \
                                                                   distanceThreshold=2.0*self.ransacThreshold, \
                                                                   distanceFactor=1.0)
                    if len(tmpRegionInliers) > len(regionInliers):
                        regionInliers = tmpRegionInliers
                        
            if len(regionInliers) == 0:
                planes.append(self.Plane(n, d, \
                         self.subsampleData(pointIndices, self.removeFactor), \
                         [], isValid = False))              
                continue
            
            (n, d, dummy) = self.refinePlane(points, regionInliers, self.ransacThreshold)

            if self.isValidPlane(points[:, np.asarray(regionInliers)]):
                planes.append(self.Plane(n, d, np.asarray(pointIndices)[regionInliers].tolist(), \
                                         [], isValid = True))
                print "Added plane with %d inliers." % len(regionInliers)
            else:
                # exclude a subset of inliers to prevent the plane from being picked up again
                planes.append(self.Plane(n, d, \
                                         np.asarray(pointIndices)[self.subsampleData(list(inliers), self.removeFactor)].tolist(), \
                                         [], isValid = False))

#         self.plotPlanes(self.pointCloud, planes, figNr=1, plotNonPlanePoints=False, visualizeError=True)
        
        return planes

    def addDesignVariables(self, problem):
        self.laserOffsetDv = aopt.Scalar(0.0e-3)
        self.laserOffsetDv.setActive(True)
        problem.addDesignVariable(self.laserOffsetDv, ic.HELPER_GROUP_ID)

        self.C_b_l_Dv = aopt.RotationQuaternionDv(self.qInit) #TODO(jrn) move the initial guess to the constructor!
        self.C_b_l_Dv.setActive(True)
        problem.addDesignVariable(self.C_b_l_Dv, ic.HELPER_GROUP_ID)
        self.t_b_l_Dv = aopt.EuclideanPointDv(self.tInit)
        self.t_b_l_Dv.setActive(True)
        problem.addDesignVariable(self.t_b_l_Dv, ic.HELPER_GROUP_ID)
        self.T_b_l_Dv = aopt.TransformationBasicDv(self.C_b_l_Dv.toExpression(), self.t_b_l_Dv.toExpression())

#         self.rangeBias_Dv = aopt.Scalar(0.0)
#         self.rangeBias_Dv.setActive(True)
#         problem.addDesignVariable(self.rangeBias_Dv, ic.HELPER_GROUP_ID)
        
	#experimental, polynomial bias term
        self.biasCoefficients = [aopt.Scalar(0.0) for i in range(0,1)]
        for c in self.biasCoefficients:
            c.setActive(True)
            problem.addDesignVariable(c, ic.HELPER_GROUP_ID)

    #TODO(jrn) Make this generic, so that we can swap detectors afterwards.
    # A suitable data structure would hold a plane and a set of measurements induced by this plane.
    def addLaserErrorTerms(self, problem, poseSplineDv, planes):
        for p in planes:
            if p.valid:
                p.n_Dv = aopt.EuclideanDirection(p.n)
                p.n_Dv.setActive(True)
                problem.addDesignVariable(p.n_Dv, ic.HELPER_GROUP_ID)
                p.d_Dv = aopt.Scalar(p.d)
                p.d_Dv.setActive(True)
                problem.addDesignVariable(p.d_Dv, ic.HELPER_GROUP_ID)

                mest = aopt.BzMEstimator(self.bzThresholdSigmas)

                removeList = []
                for i in p.inliers:
                    tk = self.laserOffsetDv.toExpression() + self.laserData[i].stamp.toSec()
                    T_w_b = poseSplineDv.transformationAtTime(tk, self.timeOffsetPadding, self.timeOffsetPadding)

                    dir_l = np.array([np.cos(self.laserData[i].angle), np.sin(self.laserData[i].angle), 0.0])
                    n_w = p.n_Dv.toExpression()
                    T_w_l = T_w_b * self.T_b_l_Dv.toExpression()
                    t_w = T_w_l.toEuclideanExpression()
                    C_w_l = T_w_l.toRotationExpression()
#                     b = self.rangeBias_Dv.toExpression()
                    d = p.d_Dv.toExpression()
                    
                    predictedMeasurement = (d - n_w.dot(t_w)) / (n_w.dot(C_w_l * dir_l))

                    if predictedMeasurement.toScalar() < 0. :
                        predictedMeasurement = predictedMeasurement * -1.0
                        print "Swapped sign! This should not happen normally!"
                        
#                     predictedMeasurement = predictedMeasurement + b
                    polynomialBias = [c.toExpression() * self.laserData[i].distance**idx \
                                      for (idx, c) in enumerate(self.biasCoefficients)]
                    
                    sumList = lambda (l, i) : l[i] if (i==len(l)-1) else l[i] + sumList((l, i+1))
                    
                    predictedMeasurement = predictedMeasurement + sumList( (polynomialBias, 0) )
                    
                    lerr = let.ScalarError(self.laserData[i].distance, self.laserData[i].invR, 
                                           predictedMeasurement)

                    #                     lerr = let.LaserError(self.laserData[i].distance, self.laserData[i].invR, dir_l, \
                    #                                           T_w_b * self.T_b_l_Dv.toExpression(), \
                    #                                           p.n_Dv.toExpression(), p.d_Dv.toExpression(), \
                    #                                           self.rangeBias_Dv.toExpression());

                    #Remove measurement when distance error in "line-of-sight" direction is too large. 
                    #if lerr.evaluateError() < self.lineOfSightErrorThreshold:
                    if np.fabs(lerr.error()) < self.lineOfSightErrorThreshold:
                        lerr.setMEstimatorPolicy(mest)
                        p.errorTerms.append(lerr)
                        problem.addErrorTerm(lerr)
                    else:
                        removeList.append(i)
                #Keep order of inliers, since it determines the association with error terms.
                p.inliers = [i for i in p.inliers if i not in removeList]


    def scaleAxes(self, ax, data):
        ax.set_aspect('equal')
        X = data[0,:]
        Y = data[1,:]
        Z = data[2,:]
        max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
        mean_x = X.mean()
        mean_y = Y.mean()
        mean_z = Z.mean()
        ax.set_xlim(mean_x - max_range, mean_x + max_range)
        ax.set_ylim(mean_y - max_range, mean_y + max_range)
        ax.set_zlim(mean_z - max_range, mean_z + max_range)

    def filterLaserErrorTerms(self, problem, planes, threshold):
        for p in planes:
            e = np.array([e.error() for e in p.errorTerms])
            removeList = [(inlier, error) for (inlier, error) in zip(p.inliers, p.errorTerms) \
                          if np.fabs(error.error()) > threshold * np.std(e)]
 
            for (inlier, error) in removeList:
                problem.removeErrorTerm(error)
                p.inliers.remove(inlier)
                p.errorTerms.remove(error)

    def plotPlaneErrors(self, planes, figNr=1, weighted=False, block=True):
        fontsize = 15

        fig = plt.figure(figNr)
        fig.clf()

        validPlanes = [p for p in planes if p.valid]
        
        for (index, p) in enumerate(validPlanes):
            plt.subplot(len(validPlanes), 1, index+1)
            if weighted:
                planeErrors = [e.evaluateError() for e in p.errorTerms]
            else:
                planeErrors = [e.error() for e in p.errorTerms]
            plt.plot(planeErrors)
            ax = plt.gca()
            ax.set_ylabel(('Plane %d' % index), fontsize=fontsize)
            
        plt.show(block=block)

    def plotPlanes(self, pointCloud, planes=[], figNr=1, plotNonPlanePoints=True, \
                   visualizeError=False, block=True):

        fontsize = 15; pointsize = 10.
        fig = plt.figure(figNr)
        fig.clf()
        ax = fig.gca(projection='3d')

        nonPlaneIndices = range(0, pointCloud.shape[1])
#         if planes != []:
#             planeIndices = [i for p in planes for i in p.inliers if p.valid == True]
#             planePts =  pointCloud[:, planeIndices]
#             nonPlaneIndices = list(set(nonPlaneIndices) - set(planeIndices))
        if planes != []:
            validPlanes = [p for p in planes if p.valid]
            for (planeIndex, p) in enumerate(validPlanes):
                    nonPlaneIndices = list(set(nonPlaneIndices) - set(p.inliers))
                    if visualizeError:
                        planeErrors = np.asarray([e.evaluateError() for e in p.errorTerms])
                        ax.scatter(pointCloud[0, p.inliers], pointCloud[1, p.inliers], pointCloud[2, p.inliers],
                                   s=pointsize, c=planeErrors, cmap=plt.get_cmap("jet"), edgecolor='k',\
                                   lw=.0, vmin=0.0, vmax=np.max(planeErrors)) #, alpha=1., vmin=0.0, vmax=1.0)               
                    else:
                        ax.scatter(pointCloud[0, p.inliers], pointCloud[1, p.inliers], pointCloud[2, p.inliers],\
                                   s=pointsize, c='r', edgecolor='k', lw=.25)
                    centroid = np.mean(pointCloud[:, p.inliers], axis=1)
                    ax.text(centroid[0], centroid[1], centroid[2], '%d' % planeIndex, fontsize=2.0*fontsize) 
        planePointIndices = list(set(range(0, pointCloud.shape[1]))-set(nonPlaneIndices))
        planePts = pointCloud[:, planePointIndices]
        
        if planePointIndices:
            self.scaleAxes(ax, planePts)

        nonPlanePts = pointCloud[:, nonPlaneIndices]
        if plotNonPlanePoints:
            ax.scatter(nonPlanePts[0,:], nonPlanePts[1,:], nonPlanePts[2,:], \
                        s=pointsize, c='k', edgecolor='k', lw=.25)
            self.scaleAxes(ax, nonPlanePts)

        print "Number of non-plane points %d" % nonPlanePts.shape[1]
        if planes != []:
            print "Number of plane points %d" % planePts.shape[1]
            
        ax.set_xlabel('x [m]', fontsize=fontsize)
        ax.set_ylabel('y [m]', fontsize=fontsize)
        ax.set_zlabel('z [m]', fontsize=fontsize)
        pl.setp(ax.get_xticklabels(), fontsize=fontsize)
        pl.setp(ax.get_yticklabels(), fontsize=fontsize)
        pl.setp(ax.get_zticklabels(), fontsize=fontsize)
        
        ax.view_init(elev=25., azim=45.)
        plt.show(block=block)
