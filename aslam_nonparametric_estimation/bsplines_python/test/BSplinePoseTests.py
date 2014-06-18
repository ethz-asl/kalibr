#!/usr/bin/env python
import roslib; roslib.load_manifest('bsplines');
roslib.load_manifest('sm_python') 
import bsplines
import sm
import numpy
import scipy.interpolate.fitpack as fp
import sys
import unittest
import numdifftools as nd
import math



class TestBSplinePose(unittest.TestCase):
    def runTest(self):
        x = 0
    def assertMatricesEqual(self,M1, M2, tolerance, msg):
        d1 = numpy.array(M1.shape)
        d2 = numpy.array(M2.shape)
        self.assertEqual(d1.size,d2.size)
        for i in range(0,d1.size):
            self.assertEqual(M1.shape[i], M2.shape[i])
        md = numpy.max(numpy.abs(M1 - M2))
        self.assertTrue(md < tolerance, msg= "The matrices\n%s\nand\n%s\nwere not equal to within tolerance %e [%e > %e]: %s" % (M1,M2,tolerance,md,tolerance, msg))
    def testCurveToTransformation(self):
        rvs = (sm.RotationVector(), sm.EulerAnglesZYX(), sm.EulerRodriguez())
        for r in rvs:
            bsp = bsplines.BSplinePose(4,r)
            # Build a random, valid transformation.
            T1 = bsp.curveValueToTransformation(numpy.random.random(6))
            p = bsp.transformationToCurveValue(T1)
            T2 = bsp.curveValueToTransformation(p)
            self.assertMatricesEqual(T1, T2, 1e-9,"Checking the invertiblity of the transformation to curve values:")
    def testTransformationJacobian(self):
        rvs = (sm.RotationVector(), sm.EulerAnglesZYX(), sm.EulerRodriguez())
        for r in rvs:
            for order in range(2,7):
                bsp = bsplines.BSplinePose(order,r)
                T_n_0 = bsp.curveValueToTransformation(numpy.random.random(6))
                T_n_1 = bsp.curveValueToTransformation(numpy.random.random(6))
                
                # Initialize the curve.
                bsp.initPoseSpline(0.0,1.0,T_n_0, T_n_1)
                
                for t in numpy.linspace(bsp.t_min(), bsp.t_max(), 4):
                    # Create a random homogeneous vector
                    v = numpy.random.random(4)
                    TJI = bsp.transformationAndJacobian(t);
                    #print "TJI: %s" % (TJI)
                    je = nd.Jacobian(lambda c: bsp.setLocalCoefficientVector(t,c) or numpy.dot(bsp.transformation(t), v))
                    estJ = je(bsp.localCoefficientVector(t))
                    JT = TJI[1]
                    J = numpy.dot(sm.boxMinus(numpy.dot(TJI[0],v)), JT)                    
                    self.assertMatricesEqual(J, estJ, 1e-9,"T_n_0")
    def testOrientationJacobian(self):
        rvs = (sm.RotationVector(), sm.EulerAnglesZYX(), sm.EulerRodriguez())
        for r in rvs:
            for order in range(2,7):
                bsp = bsplines.BSplinePose(order,r)
                T_n_0 = bsp.curveValueToTransformation(numpy.random.random(6))
                T_n_1 = bsp.curveValueToTransformation(numpy.random.random(6))
                
                # Initialize the curve.
                bsp.initPoseSpline(0.0,1.0,T_n_0, T_n_1)
                
                for t in numpy.linspace(bsp.t_min(), bsp.t_max(), 4):
                    # Create a random homogeneous vector
                    v = numpy.random.random(3)
                    CJI = bsp.orientationAndJacobian(t);
                    #print "TJI: %s" % (TJI)
                    je = nd.Jacobian(lambda c: bsp.setLocalCoefficientVector(t,c) or numpy.dot(bsp.orientation(t), v))
                    estJ = je(bsp.localCoefficientVector(t))
                    JT = CJI[1]
                    J = numpy.dot(sm.crossMx(numpy.dot(CJI[0],v)), JT)                    
                    self.assertMatricesEqual(J, estJ, 1e-8,"C_n_0")

    def testInverseOrientationJacobian(self):
        rvs = (sm.RotationVector(), sm.EulerAnglesZYX(), sm.EulerRodriguez())
        for r in rvs:
            for order in range(2,7):
                bsp = bsplines.BSplinePose(order,r)
                T_n_0 = bsp.curveValueToTransformation(numpy.random.random(6))
                T_n_1 = bsp.curveValueToTransformation(numpy.random.random(6))
                
                # Initialize the curve.
                bsp.initPoseSpline(0.0,1.0,T_n_0, T_n_1)
                
                for t in numpy.linspace(bsp.t_min(), bsp.t_max(), 4):
                    # Create a random homogeneous vector
                    v = numpy.random.random(3)
                    CJI = bsp.inverseOrientationAndJacobian(t);
                    #print "TJI: %s" % (TJI)
                    je = nd.Jacobian(lambda c: bsp.setLocalCoefficientVector(t,c) or numpy.dot(bsp.inverseOrientation(t), v))
                    estJ = je(bsp.localCoefficientVector(t))
                    JT = CJI[1]
                    J = numpy.dot(sm.crossMx(numpy.dot(CJI[0],v)), JT)                    
                    self.assertMatricesEqual(J, estJ, 1e-8,"C_n_0")


    def testAngualrVelocityJacobian(self):
        r = sm.EulerAnglesZYX();
        for order in range(2,7):
            bsp = bsplines.BSplinePose(order,r)
            T_n_0 = bsp.curveValueToTransformation(numpy.random.random(6))
            T_n_1 = bsp.curveValueToTransformation(numpy.random.random(6))
                
            # Initialize the curve.
            bsp.initPoseSpline(0.0,1.0,T_n_0, T_n_1)
                
            for t in numpy.linspace(bsp.t_min(), bsp.t_max(), 4):
                
                oJI = bsp.angularVelocityAndJacobian(t);
                #print "TJI: %s" % (TJI)
                je = nd.Jacobian(lambda c: bsp.setLocalCoefficientVector(t,c) or bsp.angularVelocity(t))
                estJ = je(bsp.localCoefficientVector(t))
                J = oJI[1]
                
                self.assertMatricesEqual(J, estJ, 1e-9,"omega Jacobian")

    def testAngularVelocityBodyFrameJacobian(self):
        r = sm.EulerAnglesZYX();
        for order in range(2,7):
            bsp = bsplines.BSplinePose(order,r)
            T_n_0 = bsp.curveValueToTransformation(numpy.random.random(6))
            T_n_1 = bsp.curveValueToTransformation(numpy.random.random(6))
                
            # Initialize the curve.
            bsp.initPoseSpline(0.0,1.0,T_n_0, T_n_1)
                
            for t in numpy.linspace(bsp.t_min(), bsp.t_max(), 4):
                
                oJI = bsp.angularVelocityBodyFrameAndJacobian(t);
                #print "TJI: %s" % (TJI)
                je = nd.Jacobian(lambda c: bsp.setLocalCoefficientVector(t,c) or bsp.angularVelocityBodyFrame(t))
                estJ = je(bsp.localCoefficientVector(t))
                J = oJI[1]
                
                self.assertMatricesEqual(J, estJ, 1e-9,"omega Jacobian")

    
    def testInitPose(self):
        bsp = bsplines.BSplinePose(4,sm.RotationVector())
        # Create two random transformations.
        T_n_0 = bsp.curveValueToTransformation(numpy.random.random(6))
        T_n_1 = bsp.curveValueToTransformation(numpy.random.random(6))
        
        # Initialize the curve.
        bsp.initPoseSpline(0.0,1.0,T_n_0, T_n_1)
        # Check the values.
        self.assertEqual(bsp.t_min(),0.0);
        self.assertEqual(bsp.t_max(),1.0);

        curve_T_n_0 = bsp.transformation(0.0);
        self.assertMatricesEqual(T_n_0, curve_T_n_0, 1e-9,"T_n_0")

        curve_T_n_1 = bsp.transformation(1.0);
        self.assertMatricesEqual(T_n_1, curve_T_n_1, 1e-9,"T_n_1")

        tend = 2.0
        # Extend the segment.
        T_n_25 = bsp.curveValueToTransformation(numpy.random.random(6))
        bsp.addPoseSegment(tend, T_n_25);

        # Check the values.
        self.assertEqual(bsp.t_min(),0.0);
        self.assertEqual(bsp.t_max(),tend);

        curve_T_n_0 = bsp.transformation(0.0);
        self.assertMatricesEqual(T_n_0, curve_T_n_0, 1e-6, "T_n_0")

        curve_T_n_1 = bsp.transformation(1.0);
        self.assertMatricesEqual(T_n_1, curve_T_n_1, 1e-4, "T_n_1")
        
        curve_T_n_25 = bsp.transformation(tend);
        self.assertMatricesEqual(T_n_25, curve_T_n_25, 1e-4, "T_n_25")
    def testAngularVelocity(self):
        bsp = bsplines.BSplinePose(4,sm.EulerAnglesZYX())
        # Create two 
        e1 = numpy.array([0,0,0,0,0,0])
        T_n_0 = bsp.curveValueToTransformation(e1)
        e2 = numpy.array([0,0,0,math.pi*0.5,0,0])
        T_n_1 = bsp.curveValueToTransformation(e2)
        
        # Initialize the curve.
        bsp.initPoseSpline(0.0,1.0,T_n_0, T_n_1)
    def testInversePose(self):
        rvs = (sm.RotationVector(), sm.EulerAnglesZYX(), sm.EulerRodriguez())
        for r in rvs:
            bsp = bsplines.BSplinePose(4,r)
            # Create two random transformations.
            T_n_0 = bsp.curveValueToTransformation(numpy.random.random(6))
            T_n_1 = bsp.curveValueToTransformation(numpy.random.random(6))
            
            # Initialize the curve.
            bsp.initPoseSpline(0.0,1.0,T_n_0, T_n_1)
            for t in numpy.arange(0.0,1.0,0.1):
                T = bsp.transformation(t)
                invT = bsp.inverseTransformation(t)
                one = numpy.dot(T,invT)
                self.assertMatricesEqual(one,numpy.eye(4),1e-14,"T * inv(T)")

    def testInversePose2(self):
        rvs = (sm.RotationVector(), sm.EulerAnglesZYX(), sm.EulerRodriguez())
        for r in rvs:
            bsp = bsplines.BSplinePose(4,r)
            # Create two random transformations.
            T_n_0 = bsp.curveValueToTransformation(numpy.random.random(6))
            T_n_1 = bsp.curveValueToTransformation(numpy.random.random(6))
            
            # Initialize the curve.
            bsp.initPoseSpline(0.0,1.0,T_n_0, T_n_1)
            for t in numpy.arange(0.0,1.0,0.1):
                T = bsp.transformation(t)
                invT,J,C = bsp.inverseTransformationAndJacobian(t)
                one = numpy.dot(T,invT)
                self.assertMatricesEqual(one,numpy.eye(4),1e-14,"T * inv(T)")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('splines', 'test_bspline_pose', TestBSplinePose)

