#!/usr/bin/env python
import aslam_backend
import aslam_splines
import numpy
import scipy
import scipy.interpolate.fitpack as fp
import scipy.integrate as si
import bisect;

import unittest

def createUniformKnotBSpline(order,segments,dim,knotSpacing=1.0):
    aspl = aslam_splines.OptEuclideanBSpline(order, dim)
    # Choose a uniform knot sequence.
    aspl.initConstantUniformSpline(0, segments * knotSpacing, segments, numpy.zeros((dim, 1)))
    kc = aspl.getNumControlVertices();
    cp = numpy.random.random([dim,kc])
    aspl.setControlVertices(cp)
    return (aspl,(aspl.getKnotsVector(),cp,order-1))
def createExponentialKnotBSpline(order,segments,dim,knotSpacing=1.0):
    aspl = aslam_splines.OptEuclideanBSpline(order, dim)
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
    aspl = aslam_splines.OptEuclideanBSpline(order, dim)
    kr = aspl.numKnotsRequired(segments)
    kc = aspl.numCoefficientsRequired(segments);
    # Choose a uniform knot sequence.
    knots = numpy.random.random(kr)*10
    knots.sort()
    cp = numpy.random.random([dim,kc])
    aspl.setKnotVectorAndCoefficients(knots, cp)
    return (aspl,(knots,cp,order-1))
def createRandomRepeatedKnotBSpline(order,segments,dim):
    aspl = aslam_splines.OptEuclideanBSpline(order, dim)
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


def evalSplineWithUpdatedCVs(spline, vertices, vI, cV, orgCV, t):
    vertices[vI] = cV
    spline.setControlVertices(vertices)
    return spline.eval(t);

def numericJacobian(spline, t, knots):
    kn = spline.getFirstRelevantKnot(t);
    iStart = bisect.bisect_left(knots, kn);
    order = spline.getSplineOrder();
    jacobian = numpy.matrix(numpy.zeros((spline.getPointSize(), order)));
    vertices = spline.getControlVertices();
    for i in range(0, order):
        vI = i + iStart;
        orgCV = numpy.matrix(vertices[vI]);
        jacobian[:, i] = scipy.misc.derivative(lambda(cV) : evalSplineWithUpdatedCVs(spline, vertices, vI, cV, orgCV, t), orgCV);
        vertices[vI] = orgCV;
        
    spline.setControlVertices(vertices)
    return jacobian

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
        
class TestOptBSpline(BSplineTestCase):
    def test_bounds(self):
        numpy.random.seed(3)
        for order in range(2,10):
            A = createUniformKnotBSpline(order,3,1);
            aspl = A[0]
            # Now, test that the bounds checking works.
            # These shouldn't raise an exception.
            aspl.eval(aspl.getMinTime())
            aspl.eval(aspl.getMaxTime())
            # These boundary cases should.
            self.assertRaises(RuntimeError, lambda: aspl.eval(aspl.getMinTime() - 1e-15))
            self.assertRaises(RuntimeError, lambda: aspl.eval(aspl.getMaxTime() + 1e-15))
            aspl.eval(aspl.getMaxTime() - 1e-15)
  
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
            aspl = aslam_splines.OptEuclideanBSpline(order, 3)
            self.assertEqual(order, aspl.splineOrder())

            #print "order: %d" % order
            #print "p_0: %s" % p_0
            #print "p_1: %s" % p_1
            # Initialize the spline with these two times 
            aspl.initUniformSpline(numpy.array([t_0, t_1]), numpy.array([p_0,p_1]).transpose(), 1, 0.1);
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
            self.assertEqual(ti[0], aspl.getMinTime())
            self.assertEqual(ti[1], aspl.getMaxTime())

    def test_time_interval2(self):
        numpy.random.seed(6)
        # Test two functions: 
        for order in range(2,10):
            nSegments = 3
            aspl = aslam_splines.OptEuclideanBSpline(order, 3)
            kr = aspl.numKnotsRequired(nSegments)
            kc = aspl.numCoefficientsRequired(nSegments);
            # Choose a uniform knot sequence at 0.0, 1.0, ...
            knots = numpy.linspace(0.0,kr-1, kr)
            cp = numpy.linspace(1.0,kc,kc)
            # build a vector-valued spline
            cpa    = numpy.array([cp,cp*cp,cp*cp*cp])
            aspl.initWithKnotsAndControlVertices(knots, cpa)
            # Check that the time interval function works.
            ti = aspl.timeInterval()
            self.assertEqual(ti[0], aspl.getMinTime())
            self.assertEqual(ti[1], aspl.getMaxTime())
    
    def test_uniform(self):
        numpy.random.seed(1)
        for order in range(2,10):
            aspl = aslam_splines.OptEuclideanBSpline(order, 1)

            kr = aspl.numKnotsRequired(3)
            kc = aspl.numCoefficientsRequired(3);
            # Choose a uniform knot sequence.
            knots = numpy.linspace(0.0,kr*1.0, kr)

            cp    = numpy.random.random([kc])
            cpa    = numpy.array([cp])
            
            aspl.initWithKnotsAndControlVertices(knots, cpa)
            fspl = (knots,cp,order-1)
            for i in numpy.linspace(aspl.getMinTime(),aspl.getMaxTime()-1e-15,10):
                f = fp.spalde(float(i),fspl)
                a = aspl.eval(i)
                for j in range(0,f.shape[0]):
                    a = aspl.evalD(i,j)
                    self.assertAlmostEqual(a, f[j])

    def test_random(self):
        numpy.random.seed(3)
        
        for order in range(2,10):
            aspl = aslam_splines.OptEuclideanBSpline(order, 1)

            kr = aspl.numKnotsRequired(3)
            kc = aspl.numCoefficientsRequired(3);
            knots = numpy.random.random([kr]) * 10
            knots.sort()
            cp    = numpy.random.random([kc])
            cpa    = numpy.array([cp])
            
            aspl.initWithKnotsAndControlVertices(knots, cpa)
            fspl = (knots,cp,order-1)
            for i in numpy.linspace(aspl.getMinTime(),aspl.getMaxTime(),10):
                f = fp.spalde(float(i),fspl)
                a = aspl.eval(i)
                for j in range(0,f.shape[0]):
                    a = aspl.evalD(i,j)
                    self.assertAlmostEqual(a, f[j])

    def test_integral(self):
        for order in range(2,8,2):
            for dt in numpy.arange(0.1,2.0,0.1): 
                # Create a spline with three segments
                aspl = aslam_splines.OptEuclideanBSpline(order, 1)
                kr = aspl.numKnotsRequired(4)
                kc = aspl.numCoefficientsRequired(4);
                # Choose a uniform knot sequence.
                knots = numpy.linspace(0.0, (kr - 1)*dt, kr)
                cp = numpy.random.random(kc);
                cpa = numpy.array([cp])

                aspl.initWithKnotsAndControlVertices(knots, cpa);
                fspl = (knots,cp,order-1)
                for a in numpy.arange(aspl.getMinTime(),aspl.getMaxTime()-1e-15,0.4*dt):
                    for i in numpy.arange(aspl.getMinTime(), aspl.getMaxTime()-1e-15, 0.4*dt):
                        #print "Eval at %f\n" % (i)
                        f = fp.splint(a,float(i),fspl)
                        b = aspl.evalI(a,i)
                        self.assertAlmostEqual(b, f, msg="order %d spline integral evaluated on [%f,%f] (%f != %f) was not right" % (order, a,i,float(b),f))
   
    def test_jacobian(self):
        for order in range(2,8,2):
            for dt in numpy.arange(0.1,2.0,0.1): 
                # Create a spline with three segments
                aspl = aslam_splines.OptEuclideanBSpline(order, 1)
                nSegs = 4
                kr = aspl.numKnotsRequired(nSegs)
                kc = aspl.numCoefficientsRequired(nSegs);
                # Choose a uniform knot sequence.
                knots = numpy.linspace(0.0, (kr - 1)*dt, kr)
                cp = numpy.random.random(kc);
                cpa = numpy.array([cp])

                aspl.initWithKnotsAndControlVertices(knots, cpa)
            
                knots = aspl.getKnotsVector();
                for a in numpy.arange(aspl.getMinTime(),aspl.getMaxTime()-1e-15,0.4*dt):
                    b = aspl.evalJacobian(a)
                    f = numericJacobian(aspl, a, knots);
                    self.assertMatricesEqual(b, f, 1E-6, msg="order %d spline jacobian evaluated at [%f] (%s != %s) was not right" % (order, a, str(b), str(f)))
   
    def test_integral_non_uniform(self):
        for order in range(2,8,2):
            # Create a spline with three segments
            aspl = aslam_splines.OptEuclideanBSpline(order, 1)
            kr = aspl.numKnotsRequired(4)
            kc = aspl.numCoefficientsRequired(4);
            # Choose a non-uniform knot sequence.
            knots = numpy.linspace(0.0, (kr - 1), kr)
            knots = knots*knots
            cp = numpy.random.random(kc);
            cpa = numpy.array([cp])

            aspl = aslam_splines.OptEuclideanBSpline(order, 1);
            aspl.initWithKnotsAndControlVertices(knots, cpa);
            fspl = (knots,cp,order-1)
            
            for a in numpy.arange(aspl.getMinTime(),aspl.getMaxTime()-1e-15,0.4):
                for i in numpy.arange(aspl.getMinTime(), aspl.getMaxTime()-1e-15, 0.4):
                    #print "Eval at %f\n" % (i)
                    f = fp.splint(a,float(i),fspl)
                    b = aspl.evalI(a,i)
                    self.assertAlmostEqual(b, f, msg="order %d spline integral evaluated on [%f,%f] (%f != %f) was not right" % (order, a,i,float(b),f))
    
    def test_constant_init(self):
        tmin = 0.0
        tmax = 5.0
        for order in range(2,6):
            for dim in range(1,4):
                for segs in range(1,4):
                    c = numpy.random.random([dim])
                    # Initialize a constant spline
                    aspl = aslam_splines.OptEuclideanBSpline(order, dim)
                    aspl.initConstantSpline(tmin,tmax,segs,c)
                    # Test the time boundaries
                    self.assertAlmostEqual(tmin,aspl.getMinTime())
                    self.assertAlmostEqual(tmax,aspl.getMaxTime())
                    # Test the value.
                    for t in numpy.arange(aspl.getMinTime(),aspl.getMaxTime(),0.1):
                        self.assertMatricesEqual(aspl.evalD(t,0),c,1e-15,"Error getting back the constant value")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('splines', 'optBSplines', TestOptBSpline)
