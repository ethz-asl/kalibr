#!/usr/bin/env python
import roslib; roslib.load_manifest('bsplines'); 
import bsplines
import numpy
import scipy.interpolate.fitpack as fp
import scipy.integrate as si

import sys
import unittest

def createUniformKnotBSpline(order,segments,dim,knotSpacing=1.0):
    aspl = bsplines.BSpline(order)
    kr = aspl.numKnotsRequired(segments)
    kc = aspl.numCoefficientsRequired(segments);
    # Choose a uniform knot sequence.
    knots = numpy.linspace(0.0,kr - 1, kr)*knotSpacing
    cp = numpy.random.random([dim,kc])
    aspl.setKnotVectorAndCoefficients(knots, cp)
    return (aspl,(knots,cp,order-1))
def createExponentialKnotBSpline(order,segments,dim,knotSpacing=1.0):
    aspl = bsplines.BSpline(order)
    kr = aspl.numKnotsRequired(segments)
    kc = aspl.numCoefficientsRequired(segments);
    # Choose a uniform knot sequence.
    knots = numpy.zeros(kr)
    for i in range(0,kr):
        knots[i] = knotSpacing * 2**i 
    cp = numpy.random.random([dim,kc])
    aspl.setKnotVectorAndCoefficients(knots, cp)
    return (aspl,(knots,cp,order-1))
def createRandomKnotBSpline(order,segments,dim):
    aspl = bsplines.BSpline(order)
    kr = aspl.numKnotsRequired(segments)
    kc = aspl.numCoefficientsRequired(segments);
    # Choose a uniform knot sequence.
    knots = numpy.random.random(kr)*10
    knots.sort()
    cp = numpy.random.random([dim,kc])
    aspl.setKnotVectorAndCoefficients(knots, cp)
    return (aspl,(knots,cp,order-1))
def createRandomRepeatedKnotBSpline(order,segments,dim):
    aspl = bsplines.BSpline(order)
    kr = aspl.numKnotsRequired(segments)
    kc = aspl.numCoefficientsRequired(segments);
    # Choose a uniform knot sequence.
    knots = numpy.random.random(kr)*10
    knots.sort()
    for i in range(0,len(knots)):
        if i&1:
            knots[i-1] = knots[i]
    cp = numpy.random.random([dim,kc])
    aspl.setKnotVectorAndCoefficients(knots, cp)
    return (aspl,(knots,cp,order-1))


class BSplineTestCase(unittest.TestCase):
    def runTest(self):
        x=0
    def assertMatricesEqual(self,M1, M2, tolerance, msg):
        d1 = numpy.array(M1.shape)
        d2 = numpy.array(M2.shape)
        self.assertEqual(d1.size,d2.size)
        for i in range(0,d1.size):
            self.assertEqual(M1.shape[i], M2.shape[i])
        md = numpy.max(numpy.abs(M1 - M2))
        self.assertTrue(md < tolerance, msg= "The matrices\n%s\nand\n%s\nwere not equal to within tolerance %e [%e > %e]: %s" % (M1,M2,tolerance,md,tolerance, msg))
        
class TestBSplines(BSplineTestCase):
    def test_bounds(self):
        numpy.random.seed(3)
        for order in range(2,10):
            A = createUniformKnotBSpline(order,3,1);
            aspl = A[0]
            # Now, test that the bounds checking works.
            # These shouldn't raise an exception.
            aspl.eval(aspl.t_min())
            aspl.eval(aspl.t_max())
            # These boundary cases should.
            self.assertRaises(RuntimeError, lambda: aspl.eval(aspl.t_min() - 1e-15))
            self.assertRaises(RuntimeError, lambda: aspl.eval(aspl.t_max() + 1e-15))
            aspl.eval(aspl.t_max() - 1e-15)
    def test_init(self):
        numpy.random.seed(5)
        # Test the initialization from two times and two positions.
        p_0 = numpy.array([1,2,3]);
        p_1 = numpy.array([2,4,6]);
        t_0 = 0.0
        t_1 = 0.1
        dt = t_1 - t_0
        v = (p_1 - p_0)/dt
        for order in range(2,10):
            aspl = bsplines.BSpline(order)
            #print "order: %d" % order
            #print "p_0: %s" % p_0
            #print "p_1: %s" % p_1
            # Initialize the spline with these two times 
            aspl.initSpline(t_0,t_1,p_0,p_1);
            b_0 = aspl.eval(t_0)
            b_1 = aspl.eval(t_1)
            v_0 = aspl.evalD(t_0,1)
            v_1 = aspl.evalD(t_1,1)
            #print "b_0: %s" % b_0
            #print "b_1: %s" % b_1
            for j in range(0,p_0.size):
                # Keep the threshold low for even power cases.
                self.assertAlmostEqual(p_0[j],b_0[j],places=2)
                self.assertAlmostEqual(p_1[j],b_1[j],places=2)
                self.assertAlmostEqual(v_0[j],v[j],places=2)
                self.assertAlmostEqual(v_1[j],v[j],places=2)
    def test_time_interval(self):
        numpy.random.seed(6)
        # Test two functions: 
        for order in range(2,10):
            A = createUniformKnotBSpline(order,3,3)
            aspl = A[0]
            # Check that the time interval function works.
            ti = aspl.timeInterval()
            self.assertEqual(ti[0], aspl.t_min())
            self.assertEqual(ti[1], aspl.t_max())
            # Check that the individual segment time interval function works
            for i in range(0,3):
                ti = aspl.timeInterval(i)
                self.assertEqual(ti[0], order - 1 + i)
                self.assertEqual(ti[1], order + i)
    def test_new_segment(self):
        numpy.random.seed(7)
        # This function tests adding a new segment to the curve.
        for order in range(2,10):
            # Create a spline with two segments
            aspl = bsplines.BSpline(order)
            
            kr = aspl.numKnotsRequired(order + 1)
            kc = aspl.numCoefficientsRequired(order + 1);
            # Choose a uniform knot sequence.
            knots = numpy.linspace(0.0,kr - 1, kr)
            cp = numpy.random.random(kc);
            # build a vector-valued spline
            cpa    = numpy.array([cp,cp*cp,cp*cp*cp])
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            # Store a reference spline that doesn't get modified.
            aspl_ref = bsplines.BSpline(order)
            aspl_ref.setKnotVectorAndCoefficients(knots,cpa)
            # Now add a segment to the spline.
            ti = aspl.timeInterval()
            # the current set of knots is uniformly spaced with spacing 1.0
            # Let's muck around with that.
            t_k = ti[1] + 0.5
            p_k = numpy.array([1.0,2.0,3.0]);
            aspl.addCurveSegment(t_k,p_k);
            # This function doesn't necessarily preserve the existing curve. It
            # does, however, preserve the curve at ti[0] (all derivatives) and
            # interpolate the value at ti[1]. Verify this.
            # For all derivatives at ti[0]
            for d in range(0,order):
                # Evaluate the new curve and the reference curve
                ref_p = aspl_ref.evalD(ti[0],d)
                p = aspl.evalD(ti[0],d)
                #print "[%f %f] S^%d(%f,%d) = %s, %s" % (ti[0], ti[1], order,t,d,ref_p,p)
                # Check that they are almost equal
                for i in range(0,p.size):
                    self.assertAlmostEqual(p[i],ref_p[i], msg="[%f %f] S^%d(%f,%d) = %s, %s" % (ti[0], ti[1], order,ti[0],d,ref_p,p))
            # Now check that it interpolates the position at ti[1]
            # Evaluate the new curve and the reference curve
            ref_p = aspl_ref.evalD(ti[1],0)
            p = aspl.evalD(ti[1],0)
            # Check that they are almost equal
            for i in range(0,p.size):
                self.assertAlmostEqual(p[i],ref_p[i], msg="[%f %f] S^%d(%f,%d) = %s, %s" % (ti[0], ti[1], order,ti[1],d,ref_p,p))
            # Now check that the curve interpolates p_k at t_k
            curve_p_k = aspl.evalD(t_k,0)
            for i in range(0,p.size):
                self.assertAlmostEqual(p_k[i],curve_p_k[i], msg="[%f %f] S^%d(%f,%d) = %s, %s" % (ti[0], ti[1], order,t_k,d,ref_p,p))
    def test_remove_segment(self):
        numpy.random.seed(8)
        for order in range(2,10):
            # Create a spline with two segments
            aspl = bsplines.BSpline(order)
            
            kr = aspl.numKnotsRequired(order + 1)
            kc = aspl.numCoefficientsRequired(order + 1);
            # Choose a uniform knot sequence.
            knots = numpy.linspace(0.0,kr - 1, kr)
            cp = numpy.random.random(kc);
            # build a vector-valued spline
            cpa    = numpy.array([cp,cp*cp,cp*cp*cp])
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            # Store a reference spline that doesn't get modified.
            aspl_ref = bsplines.BSpline(order)
            aspl_ref.setKnotVectorAndCoefficients(knots,cpa)
            # Now remove a curve segment
            aspl.removeCurveSegment()
            # Check that the knot sequence is good.
            ref_knots = aspl_ref.knots()
            knots = aspl.knots()
            self.assertEqual(knots.size,ref_knots.size - 1)
            for i in range(0,knots.size):
                self.assertEqual(knots[i],ref_knots[i+1])
            # Check that the time range is still good.
            self.assertEqual(aspl.t_min(),aspl_ref.timeInterval(0)[1])
            # Check that the coefficients survived.
            ref_coeff = aspl_ref.coefficients()
            coeff = aspl.coefficients()
            self.assertEqual(coeff.shape[1], ref_coeff.shape[1] - 1)
            for r in range(0,coeff.shape[0]):
                for c in range(0,coeff.shape[1]):
                    self.assertEqual(coeff[r,c], ref_coeff[r,c+1], msg="Order %s, coeff[%d,%d] %f != %f\n%s\n%s" % (order, r,c,coeff[r,c], ref_coeff[r,c+1],coeff,ref_coeff))
            # Now we check that the curve still evaluates well.
            for t in numpy.linspace(aspl.t_min(),aspl.t_max(),0.01):
                for d in range(0,order):
                    # Exactly equal...not approximately equal.
                    self.assertEqual(aspl.evalD(t,d),aspl_ref.evalD(t,d))
    def test_uniform(self):
        numpy.random.seed(1)
        for order in range(2,10):
            aspl = bsplines.BSpline(order)

            kr = aspl.numKnotsRequired(3)
            kc = aspl.numCoefficientsRequired(3);
            # Choose a uniform knot sequence.
            knots = numpy.linspace(0.0,kr*1.0, kr)

            cp    = numpy.random.random([kc])
            cpa    = numpy.array([cp])
            
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            fspl = (knots,cp,order-1)
            for i in numpy.linspace(aspl.t_min(),aspl.t_max()-1e-15,10):
                f = fp.spalde(float(i),fspl)
                a = aspl.eval(i)
                for j in range(0,f.shape[0]):
                    a = aspl.evalD(i,j)
                    self.assertAlmostEqual(a, f[j])

    def test_repeated(self):
        numpy.random.seed(2)
        for order in range(2,10):
            aspl = bsplines.BSpline(order)

            kr = aspl.numKnotsRequired(3)
            kc = aspl.numCoefficientsRequired(3);
            # Make a knot sequence that is all zeros at one end and all ones at the other.
            knots = numpy.zeros(kr)
            for i in range(0,knots.size):
                if i >= knots.size * 0.5:
                    knots[i] = 1.0

            cp    = numpy.random.random([kc])
            cpa    = numpy.array([cp])
            
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            fspl = (knots,cp,order-1)
            for i in numpy.linspace(aspl.t_min(),aspl.t_max()-1e-15,10):
                f = fp.spalde(float(i),fspl)
                a = aspl.eval(i)
                for j in range(0,f.shape[0]):
                    a = aspl.evalD(i,j)
                    self.assertAlmostEqual(a, f[j])

                                            
    def test_random(self):
        numpy.random.seed(3)
        
        for order in range(2,10):
            aspl = bsplines.BSpline(order)

            kr = aspl.numKnotsRequired(3)
            kc = aspl.numCoefficientsRequired(3);
            knots = numpy.random.random([kr]) * 10
            knots.sort()
            cp    = numpy.random.random([kc])
            cpa    = numpy.array([cp])
            
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            fspl = (knots,cp,order-1)
            for i in numpy.linspace(aspl.t_min(),aspl.t_max(),10):
                f = fp.spalde(float(i),fspl)
                a = aspl.eval(i)
                for j in range(0,f.shape[0]):
                    a = aspl.evalD(i,j)
                    self.assertAlmostEqual(a, f[j])
    def test_phi_c(self):
        numpy.random.seed(4)
        # Test that the linear algebra of Phi(t) * c is equivalent to the evaluation
        # of the spline curve at t: b(t)
        for order in range(2,10):
            aspl = bsplines.BSpline(order)
            kr = aspl.numKnotsRequired(3)
            kc = aspl.numCoefficientsRequired(3);
            # Choose a uniform knot sequence.
            knots = numpy.linspace(0.0,kr*1.0, kr)
            cp = numpy.linspace(1.0,kc,kc)
            # build a vector-valued spline
            cpa    = numpy.array([cp,cp*cp,cp*cp*cp])
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            for t in numpy.linspace(aspl.t_min(),aspl.t_max(),10):
                for i in range(0,order):
                    # Check that Phi(t) c(t) = s(t)
                    s = aspl.evalD(t,i)
                    Phi = aspl.Phi(t,i)
                    c = aspl.localCoefficientVector(t)
                    sprime = numpy.dot(Phi,c)
                    for j in range(0,sprime.size):
                        self.assertAlmostEqual(s[j],sprime[j])
    def test_U_B_c(self):
        numpy.random.seed(4)
        # Test that the linear algebra of Phi(t) * c is equivalent to the evaluation
        # of the spline curve at t: b(t)
        for order in range(2,10):
            aspl = bsplines.BSpline(order)
            kr = aspl.numKnotsRequired(3)
            kc = aspl.numCoefficientsRequired(3);
            # Choose a uniform knot sequence.
            knots = numpy.linspace(0.0,kr*1.0, kr)
            cp = numpy.linspace(1.0,kc,kc)
            # build a vector-valued spline
            cpa    = numpy.array([cp,cp*cp,cp*cp*cp])
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            for t in numpy.linspace(aspl.t_min(),aspl.t_max(),10):
                for i in range(0,order):
                    # Check that Phi(t) c(t) = s(t)
                    s = aspl.evalD(t,i)
                    U = aspl.U(t,i)
                    M = aspl.Mi(aspl.segmentIndex(t))
                    c = aspl.localCoefficientVector(t)
                    sprime = numpy.dot(U.T,numpy.dot(M,c))
                    for j in range(0,sprime.size):
                        self.assertAlmostEqual(s[j],sprime[j])
    def test_U_D_B_c(self):
        numpy.random.seed(4)
        # Test that the linear algebra of Phi(t) * c is equivalent to the evaluation
        # of the spline curve at t: b(t)
        for order in range(2,10):
            aspl = bsplines.BSpline(order)
            kr = aspl.numKnotsRequired(3)
            kc = aspl.numCoefficientsRequired(3);
            # Choose a uniform knot sequence.
            knots = numpy.linspace(0.0,kr*1.0, kr)
            cp = numpy.linspace(1.0,kc,kc)
            # build a vector-valued spline
            cpa    = numpy.array([cp,cp*cp,cp*cp*cp])
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            for t in numpy.linspace(aspl.t_min(),aspl.t_max(),10):
                for i in range(0,order):
                    # Check that Phi(t) c(t) = s(t)
                    s = aspl.evalD(t,i)
                    U = aspl.U(t,0)
                    M = aspl.Mi(aspl.segmentIndex(t))
                    D = aspl.Di(aspl.segmentIndex(t))
                    # Evaluate the derivative as matrix multiplication
                    for d in range(0,i):
                        M = numpy.dot(D,M)
                    c = aspl.localCoefficientVector(t)
                    sprime = numpy.dot(U.T,numpy.dot(M,c))
                    for j in range(0,sprime.size):
                        self.assertAlmostEqual(s[j],sprime[j])
    def test_init(self):
        numpy.random.seed(5)
        # Test the initialization from two times and two positions.
        p_0 = numpy.array([1,2,3]);
        p_1 = numpy.array([2,4,6]);
        t_0 = 0.0
        t_1 = 0.1
        dt = t_1 - t_0
        v = (p_1 - p_0)/dt
        for order in range(2,10):
            aspl = bsplines.BSpline(order)
            #print "order: %d" % order
            #print "p_0: %s" % p_0
            #print "p_1: %s" % p_1
            # Initialize the spline with these two times 
            aspl.initSpline(t_0,t_1,p_0,p_1);
            b_0 = aspl.eval(t_0)
            b_1 = aspl.eval(t_1)
            v_0 = aspl.evalD(t_0,1)
            v_1 = aspl.evalD(t_1,1)
            #print "b_0: %s" % b_0
            #print "b_1: %s" % b_1
            for j in range(0,p_0.size):
                # Keep the threshold low for even power cases.
                self.assertAlmostEqual(p_0[j],b_0[j],places=2)
                self.assertAlmostEqual(p_1[j],b_1[j],places=2)
                self.assertAlmostEqual(v_0[j],v[j],places=2)
                self.assertAlmostEqual(v_1[j],v[j],places=2)
    def test_time_interval(self):
        numpy.random.seed(6)
        # Test two functions: 
        for order in range(2,10):
            nSegments = 3
            aspl = bsplines.BSpline(order)
            kr = aspl.numKnotsRequired(nSegments)
            kc = aspl.numCoefficientsRequired(nSegments);
            # Choose a uniform knot sequence at 0.0, 1.0, ...
            knots = numpy.linspace(0.0,kr-1, kr)
            cp = numpy.linspace(1.0,kc,kc)
            # build a vector-valued spline
            cpa    = numpy.array([cp,cp*cp,cp*cp*cp])
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            # Check that the time interval function works.
            ti = aspl.timeInterval()
            self.assertEqual(ti[0], aspl.t_min())
            self.assertEqual(ti[1], aspl.t_max())
            # Check that the individual segment time interval function works
            for i in range(0,3):
                ti = aspl.timeInterval(i)
                self.assertEqual(ti[0], order - 1 + i)
                self.assertEqual(ti[1], order + i)
    def test_new_segment(self):
        numpy.random.seed(7)
        # This function tests adding a new segment to the curve.
        for order in range(2,10):
            # Create a spline with two segments
            aspl = bsplines.BSpline(order)
            
            kr = aspl.numKnotsRequired(order + 1)
            kc = aspl.numCoefficientsRequired(order + 1);
            # Choose a uniform knot sequence.
            knots = numpy.linspace(0.0,kr - 1, kr)
            cp = numpy.random.random(kc);
            # build a vector-valued spline
            cpa    = numpy.array([cp,cp*cp,cp*cp*cp])
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            # Store a reference spline that doesn't get modified.
            aspl_ref = bsplines.BSpline(order)
            aspl_ref.setKnotVectorAndCoefficients(knots,cpa)
            # Now add a segment to the spline.
            ti = aspl.timeInterval()
            # the current set of knots is uniformly spaced with spacing 1.0
            # Let's muck around with that.
            t_k = ti[1] + 0.5
            p_k = numpy.array([1.0,2.0,3.0]);
            aspl.addCurveSegment(t_k,p_k);
            # This function doesn't necessarily preserve the existing curve. It
            # does, however, preserve the curve at ti[0] (all derivatives) and
            # interpolate the value at ti[1]. Verify this.
            # For all derivatives at ti[0]
            for d in range(0,order):
                # Evaluate the new curve and the reference curve
                ref_p = aspl_ref.evalD(ti[0],d)
                p = aspl.evalD(ti[0],d)
                #print "[%f %f] S^%d(%f,%d) = %s, %s" % (ti[0], ti[1], order,t,d,ref_p,p)
                # Check that they are almost equal
                for i in range(0,p.size):
                    self.assertAlmostEqual(p[i],ref_p[i], msg="[%f %f] S^%d(%f,%d) = %s, %s" % (ti[0], ti[1], order,ti[0],d,ref_p,p))
            # Now check that it interpolates the position at ti[1]
            # Evaluate the new curve and the reference curve
            ref_p = aspl_ref.evalD(ti[1],0)
            p = aspl.evalD(ti[1],0)
            # Check that they are almost equal
            for i in range(0,p.size):
                self.assertAlmostEqual(p[i],ref_p[i], msg="[%f %f] S^%d(%f,%d) = %s, %s" % (ti[0], ti[1], order,ti[1],d,ref_p,p))
            # Now check that the curve interpolates p_k at t_k
            curve_p_k = aspl.evalD(t_k,0)
            for i in range(0,p.size):
                self.assertAlmostEqual(p_k[i],curve_p_k[i], msg="[%f %f] S^%d(%f,%d) = %s, %s" % (ti[0], ti[1], order,t_k,d,ref_p,p))
    def test_remove_segment(self):
        numpy.random.seed(8)
        for order in range(2,10):
            # Create a spline with two segments
            aspl = bsplines.BSpline(order)
            
            kr = aspl.numKnotsRequired(order + 1)
            kc = aspl.numCoefficientsRequired(order + 1);
            # Choose a uniform knot sequence.
            knots = numpy.linspace(0.0,kr - 1, kr)
            cp = numpy.random.random(kc);
            # build a vector-valued spline
            cpa    = numpy.array([cp,cp*cp,cp*cp*cp])
            aspl.setKnotVectorAndCoefficients(knots, cpa)
            # Store a reference spline that doesn't get modified.
            aspl_ref = bsplines.BSpline(order)
            aspl_ref.setKnotVectorAndCoefficients(knots,cpa)
            # Now remove a curve segment
            aspl.removeCurveSegment()
            # Check that the knot sequence is good.
            ref_knots = aspl_ref.knots()
            knots = aspl.knots()
            self.assertEqual(knots.size,ref_knots.size - 1)
            for i in range(0,knots.size):
                self.assertEqual(knots[i],ref_knots[i+1])
            # Check that the time range is still good.
            self.assertEqual(aspl.t_min(),aspl_ref.timeInterval(0)[1])
            # Check that the coefficients survived.
            ref_coeff = aspl_ref.coefficients()
            coeff = aspl.coefficients()
            self.assertEqual(coeff.shape[1], ref_coeff.shape[1] - 1)
            for r in range(0,coeff.shape[0]):
                for c in range(0,coeff.shape[1]):
                    self.assertEqual(coeff[r,c], ref_coeff[r,c+1], msg="Order %s, coeff[%d,%d] %f != %f\n%s\n%s" % (order, r,c,coeff[r,c], ref_coeff[r,c+1],coeff,ref_coeff))
            # Now we check that the curve still evaluates well.
            for t in numpy.linspace(aspl.t_min(),aspl.t_max(),0.01):
                for d in range(0,order):
                    # Exactly equal...not approximately equal.
                    self.assertEqual(aspl.evalD(t,d),aspl_ref.evalD(t,d))
    def test_integral(self):
        for order in range(2,8,2):
            for dt in numpy.arange(0.1,2.0,0.1): 
                # Create a spline with three segments
                aspl = bsplines.BSpline(order)
                kr = aspl.numKnotsRequired(4)
                kc = aspl.numCoefficientsRequired(4);
                # Choose a uniform knot sequence.
                knots = numpy.linspace(0.0, (kr - 1)*dt, kr)
                cp = numpy.random.random(kc);
                cpa = numpy.array([cp])

                aspl = bsplines.BSpline(order);
                aspl.setKnotVectorAndCoefficients(knots,cpa);
                fspl = (knots,cp,order-1)
                
                for a in numpy.arange(aspl.t_min(),aspl.t_max()-1e-15,0.4*dt):
                    for i in numpy.arange(aspl.t_min(), aspl.t_max()-1e-15, 0.4*dt):
                        print "Eval at %f\n" % (i)
                        f = fp.splint(a,float(i),fspl)
                        b = aspl.evalI(a,i)
                        self.assertAlmostEqual(b, f, msg="order %d spline integral evaluated on [%f,%f] (%f != %f) was not right" % (order, a,i,float(b),f))
    def test_integral_non_uniform(self):
        for order in range(2,8,2):
            # Create a spline with three segments
            aspl = bsplines.BSpline(order)
            kr = aspl.numKnotsRequired(4)
            kc = aspl.numCoefficientsRequired(4);
            # Choose a non-uniform knot sequence.
            knots = numpy.linspace(0.0, (kr - 1), kr)
            knots = knots*knots
            cp = numpy.random.random(kc);
            cpa = numpy.array([cp])

            aspl = bsplines.BSpline(order);
            aspl.setKnotVectorAndCoefficients(knots,cpa);
            fspl = (knots,cp,order-1)
            
            for a in numpy.arange(aspl.t_min(),aspl.t_max()-1e-15,0.4):
                for i in numpy.arange(aspl.t_min(), aspl.t_max()-1e-15, 0.4):
                    print "Eval at %f\n" % (i)
                    f = fp.splint(a,float(i),fspl)
                    b = aspl.evalI(a,i)
                    self.assertAlmostEqual(b, f, msg="order %d spline integral evaluated on [%f,%f] (%f != %f) was not right" % (order, a,i,float(b),f))
                    
    def test_integral_non_uniform_repeated(self):
        for order in range(2,8,2):
            # Create a spline with three segments
            aspl = bsplines.BSpline(order)
            kr = aspl.numKnotsRequired(4)
            kc = aspl.numCoefficientsRequired(4);
            # Choose a non-uniform knot sequence.
            knots = numpy.linspace(0.0, (kr - 1), kr)
            knots = knots*knots
            for i in range(0,len(knots)):
                if i & 1 > 0:
                    knots[i] = knots[i-1]
            
            cp = numpy.random.random(kc);
            cpa = numpy.array([cp])

            aspl = bsplines.BSpline(order);
            aspl.setKnotVectorAndCoefficients(knots,cpa);
            fspl = (knots,cp,order-1)
            
            for a in numpy.arange(aspl.t_min(),aspl.t_max()-1e-15,0.4):
                for i in numpy.arange(aspl.t_min(), aspl.t_max()-1e-15, 0.4):
                    print "Eval at %f\n" % (i)
                    f = fp.splint(a,float(i),fspl)
                    b = aspl.evalI(a,i)
                    self.assertAlmostEqual(b, f, msg="order %d spline integral evaluated on [%f,%f] (%f != %f) was not right" % (order, a,i,float(b),f))
    def test_quadratic_integral_diag(self):
        numpy.random.seed(5)
        for order in range(2,6,1):
            for dim in range(1,4): 
                # Create a spline with three segments
                #A = createUniformKnotBSpline(order,4,dim, knotSpacing = 0.5);
                #A = createExponentialKnotBSpline(order,4,dim, knotSpacing = 1.0);
                A = createRandomKnotBSpline(order,3,dim);
                aspl = A[0]
                for DO in range(0,order):
                    w = numpy.random.random(dim);
                    W = numpy.diag(w);
                    ef = lambda(t): numpy.dot(numpy.asmatrix(aspl.Phi(t,DO)).T , numpy.dot(W, numpy.asmatrix(aspl.Phi(t,DO))))
                    # for each segment
                    for s in range(0,aspl.numValidTimeSegments()):
                        interval = aspl.timeInterval(s)
                        # si.quad can't do matrices...blerg.
                        E = aspl.segmentQuadraticIntegralDiag(w,s,DO)
                        Eest = numpy.zeros(E.shape)
                        for r in range(0,E.shape[0]):
                            for c in range(0,E.shape[1]):
                                efrc = lambda(t): ef(t)[r,c]
                                A = si.quad(efrc,interval[0],interval[1])
                                Eest[r,c] = A[0]
                        #print E
                        #print Eest
                        self.assertMatricesEqual(E, Eest, 1e-8, "Error comparing E and Eest\n")                        
    def test_quadratic_integral_full(self):
        numpy.random.seed(5)
        for order in range(2,6,1):
            for dim in range(1,4): 
                # Create a spline with three segments
                #A = createUniformKnotBSpline(order,4,dim, knotSpacing = 0.5);
                #A = createExponentialKnotBSpline(order,4,dim, knotSpacing = 1.0);
                A = createRandomKnotBSpline(order,3,dim);
                aspl = A[0]
                for DO in range(0,order):
                    W = numpy.random.random([dim,dim]);
                    W = numpy.dot(W.T,W) + numpy.eye(dim)
                    ef = lambda(t): numpy.dot(numpy.asmatrix(aspl.Phi(t,DO)).T , numpy.dot(W, numpy.asmatrix(aspl.Phi(t,DO))))
                    # for each segment
                    for s in range(0,aspl.numValidTimeSegments()):
                        interval = aspl.timeInterval(s)
                        # si.quad can't do matrices...blerg.
                        E = aspl.segmentQuadraticIntegral(W,s,DO)
                        Eest = numpy.zeros(E.shape)
                        for r in range(0,E.shape[0]):
                            for c in range(0,E.shape[1]):
                                efrc = lambda(t): ef(t)[r,c]
                                A = si.quad(efrc,interval[0],interval[1])
                                Eest[r,c] = A[0]
                        #print E
                        #print Eest
                        self.assertMatricesEqual(E, Eest, 1e-8, "Error comparing E and Eest\n")                        
    def test_curve_quadratic_integral_full(self):
        numpy.random.seed(6)
        for order in range(2,6,1):
            for dim in range(1,4): 
                # Create a spline with three segments
                #A = createUniformKnotBSpline(order,4,dim, knotSpacing = 0.5);
                #A = createExponentialKnotBSpline(order,4,dim, knotSpacing = 1.0);
                A = createRandomKnotBSpline(order,3,dim)
                aspl = A[0]
                for DO in range(0,order):
                    W = numpy.random.random([dim,dim]);
                    W = numpy.dot(W.T,W) + numpy.eye(dim)
                    class CurveHelper(object):
                        def __init__(self,aspl):
                            self.aspl = aspl
                        def quad(self,t):
                            L = self.aspl.coefficientVectorLength()
                            XX = numpy.zeros([L,L])
                            S = self.aspl.localCoefficientVectorIndices(t)
                            XX[numpy.ix_(S,S)] = numpy.dot(numpy.asmatrix(self.aspl.Phi(t,DO)).T , numpy.dot(W, numpy.asmatrix(self.aspl.Phi(t,DO))))
                            return XX
                    ch = CurveHelper(aspl)
                    ef = lambda(t): ch.quad(t)
                    # si.quad can't do matrices...blerg.
                    E = aspl.curveQuadraticIntegral(W,DO)
                    Eest = numpy.zeros(E.shape)
                    interval = aspl.timeInterval()
                    # quad has trouble with the discontinuities at the knots.
                    # we can pass the internal knot points as a hint that
                    # it shouldn't worry so much.
                    pts = aspl.knots()
                    pts = pts[pts > interval[0]]
                    pts = pts[pts < interval[1]]
                    for r in range(0,E.shape[0]):
                        for c in range(0,E.shape[1]):
                            efrc = lambda(t): ef(t)[r,c]
                            
                            A = si.quad(efrc,interval[0],interval[1], points=pts)
                            Eest[r,c] = A[0]
                    self.assertMatricesEqual(E, Eest, 1e-6, "Error comparing E and Eest\n")                        
    def test_constant_init(self):
        tmin = 0.0
        tmax = 5.0
        for order in range(2,6):
            for dim in range(1,4):
                for segs in range(1,4):
                    c = numpy.random.random([dim])
                    # Initialize a constant spline
                    aspl = bsplines.BSpline(order)
                    aspl.initConstantSpline(tmin,tmax,segs,c)
                    # Test the time boundaries
                    self.assertAlmostEqual(tmin,aspl.t_min())
                    self.assertAlmostEqual(tmax,aspl.t_max())
                    # Test the value.
                    for t in numpy.arange(aspl.t_min(),aspl.t_max(),0.1):
                        self.assertMatricesEqual(aspl.evalD(t,0),c,1e-15,"Error getting back the constant value")
if __name__ == '__main__':
    import rostest
    rostest.rosrun('splines', 'bspline', TestBSplines)
    #tb = TestBSplines()
    #tb.test_constant_init()
